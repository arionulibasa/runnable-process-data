use core::ops::Range;
use soem_lib::master::slave::ProcessDataLayout;

pub const INPUT_CHUNK_BYTES: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImageError
{
  Bounds,
  PackedLayout,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct SlaveRegions
{
  pub outputs: Range<usize>,
  pub inputs: Range<usize>,
}

impl SlaveRegions
{
  //The example requests byte-aligned sequential mapping, not packed mapping.
  pub fn new(io: &ProcessDataLayout, group_outputs: usize, used: usize) -> Result<Self, ImageError>
  {
    if group_outputs > used { return Err(ImageError::Bounds); }
    if io.output_start_bit != 0 || io.input_start_bit != 0 { return Err(ImageError::PackedLayout); }
    let outputs = region(io.output_offsets, io.output_bytes, group_outputs)?;
    let inputs = region(io.input_offset, io.input_bytes, used)?;
    if (!inputs.is_empty() && inputs.start < group_outputs)
      || usize::from(io.output_bits) > outputs.len() * 8
      || usize::from(io.input_bits) > inputs.len() * 8
    {
      return Err(ImageError::Bounds);
    }
    Ok(Self { outputs, inputs })
  }
}

fn region(offset: u32, length: u32, used: usize) -> Result<Range<usize>, ImageError>
{
  let start = usize::try_from(offset).map_err(|_| ImageError::Bounds)?;
  let length = usize::try_from(length).map_err(|_| ImageError::Bounds)?;
  let end = start.checked_add(length).filter(|end| *end <= used).ok_or(ImageError::Bounds)?;
  Ok(start..end)
}

//Each slave keeps its own cursor. Long inputs cannot starve another slave's log.
pub fn input_chunk(length: usize, cursor: &mut usize) -> Range<usize>
{
  let start = if *cursor < length { *cursor } else { 0 };
  let end = start.saturating_add(INPUT_CHUNK_BYTES).min(length);
  *cursor = if end == length { 0 } else { end };
  start..end
}

#[cfg(test)]
mod tests
{
  use super::*;

  #[test]
  fn mixed_layouts_use_all_outputs_then_all_inputs()
  {
    let first = ProcessDataLayout
    {
      output_bits: 16, output_bytes: 2, output_offsets: 0,
      input_bits: 112, input_bytes: 14, input_offset: 5,
      ..Default::default()
    };
    let second = ProcessDataLayout
    {
      output_bits: 24, output_bytes: 3, output_offsets: 2,
      input_bits: 48, input_bytes: 6, input_offset: 19,
      ..Default::default()
    };
    let a = SlaveRegions::new(&first, 5, 25).unwrap();
    let b = SlaveRegions::new(&second, 5, 25).unwrap();
    assert_eq!(a, SlaveRegions { outputs: 0..2, inputs: 5..19 });
    assert_eq!(b, SlaveRegions { outputs: 2..5, inputs: 19..25 });
    let mut image = [0x55; 25];
    image[a.outputs].fill(0);
    image[b.outputs].fill(0);
    assert_eq!(&image[..5], &[0; 5]);
    assert_eq!(&image[a.inputs], &[0x55; 14]);
    assert_eq!(&image[b.inputs], &[0x55; 6]);
  }

  #[test]
  fn one_direction_and_empty_slaves_are_valid_but_bad_ranges_are_not()
  {
    let input = ProcessDataLayout { input_bits: 1, input_bytes: 1, input_offset: 2, ..Default::default() };
    assert_eq!(SlaveRegions::new(&input, 2, 3).unwrap().inputs, 2..3);
    let output = ProcessDataLayout { output_bits: 16, output_bytes: 2, ..Default::default() };
    assert_eq!(SlaveRegions::new(&output, 2, 3).unwrap().outputs, 0..2);
    assert!(SlaveRegions::new(&Default::default(), 2, 3).is_ok());
    assert!(SlaveRegions::new(&input, 2, 2).is_err());
    assert!(SlaveRegions::new(&input, 3, 3).is_err());
    assert!(SlaveRegions::new(&output, 1, 3).is_err());
    assert!(SlaveRegions::new(&ProcessDataLayout { input_offset: u32::MAX, ..input }, 2, 3).is_err());
    assert!(SlaveRegions::new(&ProcessDataLayout { input_start_bit: 1, ..input }, 2, 3).is_err());
  }

  #[test]
  fn chunks_cover_long_inputs_and_wrap_without_hiding_empty_inputs()
  {
    let mut a = 0;
    let mut b = 0;
    assert_eq!(input_chunk(35, &mut a), 0..16);
    assert_eq!(input_chunk(6, &mut b), 0..6);
    assert_eq!(input_chunk(35, &mut a), 16..32);
    assert_eq!(input_chunk(6, &mut b), 0..6);
    assert_eq!(input_chunk(35, &mut a), 32..35);
    assert_eq!(input_chunk(35, &mut a), 0..16);
    assert_eq!(input_chunk(0, &mut b), 0..0);
  }
}
