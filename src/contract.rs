use soem_lib::{Slave, coe::sdo::SdoError};

pub const VENDOR: u32 = 0x0000_0232;
pub const PRODUCT: u32 = 0x000a_b123;
pub const REVISION: u32 = 9;
pub const OUTPUT_BYTES: usize = 2;
pub const INPUT_BYTES: usize = 14;
pub const IO_BYTES: usize = OUTPUT_BYTES + INPUT_BYTES;


///////////////////////////////////////////////////////////////////////////////////////////
//The unusual 0x1400/0x1401 indices are exactly those supplied in livingup.
//Only uploads are made: this demo never changes PDO assignments through SDO.
///////////////////////////////////////////////////////////////////////////////////////////
pub const COE_CHECKS: &[(u16, u8, usize, u32)] = &[
  (0x1018, 1, 4, VENDOR), (0x1018, 2, 4, PRODUCT), (0x1018, 3, 4, REVISION),
  (0x1c00, 0, 1, 4),
  (0x1c00, 1, 1, 1), (0x1c00, 2, 1, 2),
  (0x1c00, 3, 1, 3), (0x1c00, 4, 1, 4),
  (0x1c12, 0, 1, 2), (0x1c12, 1, 2, 0x1400), (0x1c12, 2, 2, 0x1401),
  (0x1c13, 0, 1, 2), (0x1c13, 1, 2, 0x1a00), (0x1c13, 2, 2, 0x1a01),
  (0x1400, 0, 1, 1), (0x1400, 1, 4, 0x7000_0108),
  (0x1401, 0, 1, 1), (0x1401, 1, 4, 0x7001_0008),
  (0x1a00, 0, 1, 8),
  (0x1a00, 1, 4, 0x6000_0108), (0x1a00, 2, 4, 0x6000_0208),
  (0x1a00, 3, 4, 0x6000_0308), (0x1a00, 4, 4, 0x6000_0408),
  (0x1a00, 5, 4, 0x6000_0508), (0x1a00, 6, 4, 0x6000_0608),
  (0x1a00, 7, 4, 0x6000_0708), (0x1a00, 8, 4, 0x6000_0808),
  (0x1a01, 0, 1, 4),
  (0x1a01, 1, 4, 0x6001_0120), (0x1a01, 2, 4, 0x6001_0208),
  (0x1a01, 3, 4, 0x6001_0301), (0x1a01, 4, 4, 0x0000_0007),
];


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractError
{
  Identity { vendor: u32, product: u32, revision: u32 },
  Sdo { index: u16, sub: u8, error: SdoError },
  Object { index: u16, sub: u8, expected: u32, actual: u32, expected_width: usize, width: usize },
  Layout,
  ImageLength,
}


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
pub fn verify_identity(slave: &Slave) -> Result<(), ContractError>
{
  if (slave.manufacturer, slave.product_code, slave.revision) != (VENDOR, PRODUCT, REVISION)
  {
    return Err(ContractError::Identity
    {
      vendor: slave.manufacturer, product: slave.product_code, revision: slave.revision,
    });
  }
  Ok(())
}


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
pub fn verify_coe<R>(mut read: R) -> Result<(), ContractError>
where R: FnMut(u16, u8, &mut [u8]) -> Result<usize, SdoError>
{
  for &(index, sub, expected_width, expected) in COE_CHECKS
  {
    let mut bytes = [0; 4];
    let width = read(index, sub, &mut bytes)
      .map_err(|error| ContractError::Sdo { index, sub, error })?;
    let actual = u32::from_le_bytes(bytes);
    //Some SOES versions expose an assignment count using a 16-bit container.
    let assignment_count = sub == 0 && matches!(index, 0x1c12 | 0x1c13);
    if actual != expected || (width != expected_width && !(assignment_count && width == 2))
    {
      return Err(ContractError::Object { index, sub, expected, actual, expected_width, width });
    }
  }
  Ok(())
}


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
pub fn verify_layout(slave: &Slave, used: usize, expected_wkc: u16) -> Result<(), ContractError>
{
  let io = &slave.process_data;
  if used != IO_BYTES || expected_wkc != 3
    || io.output_bits != 16 || io.output_bytes != OUTPUT_BYTES as u32
    || io.input_bits != 112 || io.input_bytes != INPUT_BYTES as u32
    || io.output_offsets != 0 || io.input_offset != OUTPUT_BYTES as u32
    || io.output_start_bit != 0 || io.input_start_bit != 0
    || slave.sync_managers[2].start_addr != 0x1400 || slave.sync_managers[2].length != 2
    || slave.sync_managers[3].start_addr != 0x1a00 || slave.sync_managers[3].length != 14
    || slave.first_unused_fmmu != 2
  {
    return Err(ContractError::Layout);
  }
  Ok(())
}


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Inputs
{
  pub array: [u8; 8],
  pub calmdown: i32,
  pub newlight: i8,
  pub boolbool: bool,
}


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
pub fn decode_inputs(image: &[u8]) -> Result<Inputs, ContractError>
{
  let input = image.get(OUTPUT_BYTES..IO_BYTES).ok_or(ContractError::ImageLength)?;
  let mut array = [0; 8];
  array.copy_from_slice(&input[..8]);
  Ok(Inputs
  {
    array,
    calmdown: i32::from_le_bytes(input[8..12].try_into().unwrap()),
    newlight: input[12] as i8,
    boolbool: input[13] & 1 != 0,
  })
}





///////////////////////////////////////////////////////////////////////////////////////////
//Unit test
///////////////////////////////////////////////////////////////////////////////////////////
#[cfg(test)]
mod tests
{
  use super::*;
  use soem_lib::{SlaveId, master::slave::ProcessDataLayout};

  fn upload(index: u16, sub: u8, out: &mut [u8]) -> Result<usize, SdoError>
  {
    let &(_, _, width, value) = COE_CHECKS.iter().find(|&&(i, s, _, _)| i == index && s == sub).unwrap();
    out[..width].copy_from_slice(&value.to_le_bytes()[..width]);
    Ok(width)
  }

  #[test]
  fn exact_dictionary_and_assignment_count_variants_pass()
  {
    assert_eq!(verify_coe(upload), Ok(()));
    assert_eq!(verify_coe(|index, sub, out|
    {
      let width = upload(index, sub, out)?;
      Ok(if sub == 0 && matches!(index, 0x1c12 | 0x1c13) { 2 } else { width })
    }), Ok(()));
  }

  #[test]
  fn wrong_mapping_order_width_or_abort_is_not_silently_accepted()
  {
    for width_error in [true, false]
    {
      let result = verify_coe(|index, sub, out|
      {
        let width = upload(index, sub, out)?;
        if index == 0x1a00 && sub == 1
        {
          if width_error { return Ok(3); }
          out[1] = 2;
        }
        Ok(width)
      });
      assert!(matches!(result, Err(ContractError::Object { index: 0x1a00, sub: 1, .. })));
    }
    assert!(matches!(verify_coe(|_, _, _| Err(SdoError::Abort(0x0602_0000))),
      Err(ContractError::Sdo { error: SdoError::Abort(0x0602_0000), .. })));
  }

  #[test]
  fn decodes_little_endian_signed_values_and_ignores_padding()
  {
    let mut io = [0; IO_BYTES];
    io[2..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
    io[10..14].copy_from_slice(&(-123456i32).to_le_bytes());
    io[14] = (-12i8) as u8;
    io[15] = 0xff;
    assert_eq!(decode_inputs(&io), Ok(Inputs
    { array: [1, 2, 3, 4, 5, 6, 7, 8], calmdown: -123456, newlight: -12, boolbool: true }));
    io[15] = 0xfe;
    assert!(!decode_inputs(&io).unwrap().boolbool);
    assert_eq!(decode_inputs(&io[..15]), Err(ContractError::ImageLength));
  }

  #[test]
  fn identity_and_runtime_layout_must_match_before_cyclic_io()
  {
    let mut slave = Slave::new(SlaveId(1));
    assert!(verify_identity(&slave).is_err());
    slave.manufacturer = VENDOR;
    slave.product_code = PRODUCT;
    slave.revision = REVISION;
    assert_eq!(verify_identity(&slave), Ok(()));
    slave.process_data = ProcessDataLayout
    {
      output_bits: 16, output_bytes: 2, output_offsets: 0, output_start_bit: 0,
      input_bits: 112, input_bytes: 14, input_offset: 2, input_start_bit: 0,
    };
    slave.sync_managers[2].start_addr = 0x1400;
    slave.sync_managers[2].length = 2;
    slave.sync_managers[3].start_addr = 0x1a00;
    slave.sync_managers[3].length = 14;
    slave.first_unused_fmmu = 2;
    assert_eq!(verify_layout(&slave, 16, 3), Ok(()));
    assert_eq!(verify_layout(&slave, 16, 0), Err(ContractError::Layout));
    slave.process_data.input_offset = 1;
    assert_eq!(verify_layout(&slave, 16, 3), Err(ContractError::Layout));
  }
}
