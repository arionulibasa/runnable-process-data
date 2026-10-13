#![no_std]
#![no_main]

mod clock;
mod console;
mod reset;

use core::cell::RefCell;
use cortex_m_rt::entry;
use embedded_hal::{delay::DelayNs, spi::MODE_0};
use embedded_hal_bus::spi::ExclusiveDevice;
use soem_lib::{Error, Master, SlaveId, SlaveState};
use soem_lib::config::pdo_map::{PdoMapOptions, PdoSource};
use soem_lib::port::{EthercatPort, clock::MonotonicClock, lan9250::Lan9250Port};
use soem_stm32f4_slaveinfo::trace::{Record, Trace, TracePort};
use soem_stm32f4_processdata::{process_image::{ImageError, SlaveRegions, input_chunk}, runtime::{Monitor, MonitorError, Schedule, Telemetry}};
use stm32f4xx_hal::{pac, prelude::*, rcc::Config};
use clock::{ClockRef, Tim2Clock};
use reset::TimedReset;

const MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 1];
//One extra slot is reserved for SOEM's aggregate slave-zero record.
const MAX_PHYSICAL_SLAVES: usize = 8;
const SLAVE_SLOTS: usize = MAX_PHYSICAL_SLAVES + 1;
const IO_MAP_BYTES: usize = 1024;

//A bench starting point, not a proven slave timing requirement. No DC is used.
const CYCLE_US: u32 = 10_000;
const PDO_TIMEOUT_US: u32 = 2_000;
const REGISTER_TIMEOUT_US: u32 = 2_000;
const STATE_POLL_US: u64 = 100_000;
const PRINT_US: u64 = 500_000;
const ENABLE_OP: bool = cfg!(feature = "enable-op");
const USE_SII: bool = cfg!(feature = "sii");

//Changing capacities also requires revisiting frame count and OP poll latency.
const _: () = assert!(IO_MAP_BYTES <= soem_lib::config::pdo_map::MAX_PROCESS_DATA_BYTES as usize);
const _: () = assert!(MAX_PHYSICAL_SLAVES > 0 && MAX_PHYSICAL_SLAVES <= u16::MAX as usize);
const _: () = assert!(STATE_POLL_US * (MAX_PHYSICAL_SLAVES as u64) < 2_000_000);



///////////////////////////////////////////////////////////////////////////////////////////
//It is a type of Master from master/context.rs.
///////////////////////////////////////////////////////////////////////////////////////////
//1024 bytes fit one segment: one LRW, or at most one LRD and one LWR.
type DemoMaster<P, C> = Master<P, C, SLAVE_SLOTS, 1, 2>;


///////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////
#[derive(Debug)]
enum Failure
{
  Stack(&'static str, Error),
  SlaveCount(usize),
  Image { slave: usize, error: ImageError },
  EmptyMapping,
  SlaveStack { slave: usize, stage: &'static str, error: Error },
  Monitor { slave: usize, error: MonitorError },
  Timing { missed: u64, max_work_us: u64 }
}


///////////////////////////////////////////////////////////////////////////////////////////
//Implement Display trait for Failure
//We do this for displaying the error in screen. 
///////////////////////////////////////////////////////////////////////////////////////////
impl core::fmt::Display for Failure
{
  //Required method for Display trait is fmt. We give give the return as core::fmt::Result not core::result::Result. 
  //out is a type of core::fmt::Formatter which a struct.
  fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result
  {
    //We do match Failure fields. 
    //self refer to instance of Failure.
    match self
    {
      Self::Stack(stage, error)             => write!(out, "{}: {:?}", stage, error),
      Self::SlaveCount(count)               => write!(out, "expected 1..={} physical slaves, found {}", MAX_PHYSICAL_SLAVES, count),
      Self::Image { slave, error }          => write!(out, "slave {} IOmap: {:?}", slave, error),
      Self::EmptyMapping                    => write!(out, "group has no process data or no expected WKC"),
      Self::SlaveStack { slave, stage, error } => write!(out, "slave {} {}: {:?}", slave, stage, error),
      Self::Monitor { slave, error }        => write!(out, "slave {} cyclic monitor: {:x?}", slave, error),
      Self::Timing { missed, max_work_us }  => write!(out, "three consecutive cycle overruns; missed starts={}, maximum work={} us", missed, max_work_us),
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
//This is outer attribute. This outer attribute relate to cortex_m_rt.
///////////////////////////////////////////////////////////////////////////////////////////
#[entry]
fn main() -> !
{
  //What does unwrap do?? It takes/gets value from Some(value).
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();
  let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(16.MHz()));
  let gpioa = dp.GPIOA.split(&mut rcc);
  let gpioc = dp.GPIOC.split(&mut rcc);

  console::init(dp.UART5.tx(gpioc.pc12, 115_200.bps(), &mut rcc).unwrap());

  log!("\r\nsoem-lib processdata: STM32F446RE + LAN9250 (generic multi-slave v2)\r\n");
  log!("HSI 16 MHz; UART5 PC12 115200 8N1; SPI1 4 MHz mode 0\r\n");
  log!("Mode={}; mapping={}; period={} us; all output bytes zero; no DC\r\n",
    if ENABLE_OP { "OP ENABLED" } else { "SAFE_OP only" },
    if USE_SII { "SII (requires PDO categories)" } else { "Auto (CoE / SII)" },
    CYCLE_US);
  
  log!("Isolated bench only. Slave watchdog timeout is unknown and is not modified.\r\n");
  log!("Capacity: {} physical slaves, {} IOmap bytes. No device-specific contract.\r\n", MAX_PHYSICAL_SLAVES, IO_MAP_BYTES);

  let timer = Tim2Clock::new(dp.TIM2, &mut rcc);
  let mut clock = ClockRef(&timer);
  let spi = dp.SPI1.spi((Some(gpioa.pa5), Some(gpioa.pa6), Some(gpioa.pa7)), MODE_0, 4.MHz(), &mut rcc);
  let cs = gpioc.pc4.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High);
  let spi = ExclusiveDevice::new(spi, cs, clock).unwrap();
  
  let reset = TimedReset
  {
    pin: gpioc.pc5.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High),
    delay: cortex_m::delay::Delay::new(cp.SYST, rcc.clocks.hclk().raw()),
  };
  
  let int = gpioa.pa2.into_pull_up_input();
  let mut port = Lan9250Port::new(int, reset, spi, clock, MAC);
  log!("Resetting LAN9250 and checking SPI/READY...\r\n");
  
  if let Err(error) = port.device_mut().init(MAC)
  {
    log!("NIC init failed: {:?}\r\n", error);
    idle(clock);
  }

  match port.device_mut().read_reg(0x0050)
  {
    Ok(id) => log!("LAN9250 ID_REV: 0x{:08x}\r\n", id),
    Err(error) => { log!("NIC ID failed: {:?}\r\n", error); idle(clock); }
  }

  if let Err(error) = port.device_mut().write_phy(0, 0x2100)
  {
    log!("PHY configuration failed: {:?}\r\n", error);
    idle(clock);
  }

  log!("Waiting for fixed 100 Mbps full-duplex link (10 seconds)...\r\n");
  let start = clock.now_us();

  loop
  {
    match port.device_mut().link_up()
    {
      Ok(true) => break,
      Ok(false) => {},
      Err(error) => { log!("PHY status failed: {:?}\r\n", error); idle(clock); }
    }
    if clock.elapsed_us(start) >= 10_000_000
    {
      log!("No link. Check powered slave, input port, cable and PHY settings.\r\n");
      idle(clock);
    }
    clock.delay_ms(10);
  }

  let trace = RefCell::new(Trace::new());
  let mut master: DemoMaster<_, _> = Master::new(TracePort::new(port, &trace), clock, MAC);
  let mut iomap = [0; IO_MAP_BYTES];

  if let Err(error) = run(&mut master, &clock, &mut iomap)
  {
    //Save failing traffic before the cleanup request adds more trace events.
    let mut saved: [Option<Record>; 32] = [None; 32];
    for (out, record) in saved.iter_mut().zip(trace.borrow().iter()) { *out = Some(*record); }
    let count = master.slave_count().saturating_sub(1).min(MAX_PHYSICAL_SLAVES);
    let mut prior_al = [(0u16, 0u16); MAX_PHYSICAL_SLAVES];
    for (slot, prior) in prior_al[..count].iter_mut().enumerate()
    {
      if let Ok(slave) = master.slave(SlaveId((slot + 1) as u16))
      {
        *prior = (slave.state.raw(), slave.al_status_code);
      }
    }
    //Do this before blocking UART output. It is best effort, not a safety guarantee.
    let stop = master.write_state(SlaveId::ALL, SlaveState::INIT, REGISTER_TIMEOUT_US);
    
    log!("\r\nSTOP: {}\r\n", error);
    log!("Broadcast INIT request={:?}. No automatic restart.\r\n", stop);
    for (slot, (state, code)) in prior_al[..count].iter().enumerate()
    {
      let id = SlaveId((slot + 1) as u16);
      let actual = master.read_state(id, REGISTER_TIMEOUT_US);
      log!("Slave {}: cached AL={:04x}/{:04x}; INIT readback={:?}\r\n", id.0, state, code, actual);
      if let Ok(slave) = master.slave(id)
      {
        log!("  AL={:04x}/{:04x}; mapped O={} I={} bytes\r\n",
          slave.state.raw(), slave.al_status_code, slave.process_data.output_bytes, slave.process_data.input_bytes);
      }
    }

    log!("Trace before stop request:\r\n");
    
    for record in saved.iter().flatten() { log!("  {}\r\n", record); }
  }
  idle(clock)
}



///////////////////////////////////////////////////////////////////////////////////////////
//Here is the main logic of the process data exist.
///////////////////////////////////////////////////////////////////////////////////////////
fn run<P: EthercatPort, C: MonotonicClock>(master: &mut DemoMaster<P, C>, clock: &C, iomap: &mut [u8; IO_MAP_BYTES]) -> Result<(), Failure>
{
  log!("Discovering slaves and confirming PRE_OP...\r\n");
  let count = master.configure_slaves().map_err(|e| Failure::Stack("discovery", e))?;
  if count == 0 || count > MAX_PHYSICAL_SLAVES { return Err(Failure::SlaveCount(count)); }
  for number in 1..=count
  {
    let slave = master.slave(SlaveId(number as u16)).map_err(|error|
      Failure::SlaveStack { slave: number, stage: "record", error })?;
    log!("Slave {}: {}; vendor=0x{:08x} product=0x{:08x} revision={} serial={} CoE={}\r\n",
      slave.id.0, slave.name.as_str().unwrap_or("<non-UTF8>"), slave.manufacturer,
      slave.product_code, slave.revision, slave.serial, slave.supports_coe());
  }
  log!("Mapping group 0 from slave descriptions; requesting SAFE_OP...\r\n");
  let options = PdoMapOptions
  {
    source: if USE_SII { PdoSource::Sii } else { PdoSource::Auto },
    ..PdoMapOptions::default()
  };
  let used = master.configure_process_data_group_with_options(0, iomap, options).map_err(|e| Failure::Stack("PDO mapping / SAFE_OP", e))?;
  let expected_wkc = master.expected_wkc();
  if used == 0 || expected_wkc == 0 { return Err(Failure::EmptyMapping); }
  let aggregate = master.slave(SlaveId::ALL).map_err(|e| Failure::Stack("group record", e))?;
  let output_bytes = aggregate.process_data.output_bytes as usize;
  if used > iomap.len() || output_bytes > used
  {
    return Err(Failure::Image { slave: 0, error: ImageError::Bounds });
  }
  let mut regions: [SlaveRegions; MAX_PHYSICAL_SLAVES] = core::array::from_fn(|_| SlaveRegions::default());
  let mut monitors = [Monitor::new(ENABLE_OP, expected_wkc); MAX_PHYSICAL_SLAVES];
  let mut states = [(0u16, 0u16); MAX_PHYSICAL_SLAVES];
  for slot in 0..count
  {
    let number = slot + 1;
    let slave = master.slave(SlaveId(number as u16)).map_err(|error|
      Failure::SlaveStack { slave: number, stage: "mapped record", error })?;
    regions[slot] = SlaveRegions::new(&slave.process_data, output_bytes, used)
      .map_err(|error| Failure::Image { slave: number, error })?;
    states[slot] = (slave.state.raw(), slave.al_status_code);
    monitors[slot].observe(slave.state, slave.al_status_code)
      .map_err(|error| Failure::Monitor { slave: number, error })?;
    log!("Slave {}: O={} bytes/{} bits at {}; I={} bytes/{} bits at {}; AL={:04x}/{:04x}\r\n",
      number, regions[slot].outputs.len(), slave.process_data.output_bits, regions[slot].outputs.start,
      regions[slot].inputs.len(), slave.process_data.input_bits, regions[slot].inputs.start,
      states[slot].0, states[slot].1);
  }
  log!("{} slaves SAFE_OP; group O={} I={} bytes; expected WKC={}\r\n",
    count, output_bytes, used - output_bytes, expected_wkc);
  log!("Starting cyclic exchange; UART is now nonblocking. Bad WKC stops the test.\r\n");

  let start = clock.now_us();
  let mut schedule = Schedule::new(start, CYCLE_US);
  let mut telemetry = Telemetry::new();
  let mut next_poll = start;
  let mut next_print = start;
  let mut cycles = 0u64;
  let mut last_start = start;
  let mut max_gap = 0u64;
  let mut max_work = 0u64;
  let mut missed_total = 0u64;
  let mut late_streak = 0u8;
  let mut dropped = 0u64;
  let mut poll_slot = 0usize;
  let mut print_slot = 0usize;
  let mut input_cursors = [0usize; MAX_PHYSICAL_SLAVES];
  
  loop
  {
    let cycle_start = clock.now_us();
    max_gap = max_gap.max(cycle_start - last_start);
    last_start = cycle_start;
    //Only the output partition is cleared; inputs belong to the slaves.
    iomap[..output_bytes].fill(0);
    let report = master.exchange_processdata(&mut iomap[..used], PDO_TIMEOUT_US)
      .map_err(|e| Failure::Stack("PDO exchange", e))?;
    let now = clock.now_us();

    let mut request_op = false;
    for (slot, monitor) in monitors[..count].iter_mut().enumerate()
    {
      request_op |= monitor.cycle(now, report)
        .map_err(|error| Failure::Monitor { slave: slot + 1, error })?;
    }
    if request_op
    {
      //The first valid output exchange precedes OP, as in SOEM simple_ng.
      master.write_state(SlaveId::ALL, SlaveState::OPERATIONAL, REGISTER_TIMEOUT_US)
        .map_err(|e| Failure::Stack("OP request", e))?;
      next_poll = now;
    }

    if now >= next_poll
    {
      //Poll one slave per interval to bound register traffic in a PDO cycle.
      let number = poll_slot + 1;
      let id = SlaveId(number as u16);
      let state = master.read_state(id, REGISTER_TIMEOUT_US)
        .map_err(|error| Failure::SlaveStack { slave: number, stage: "AL poll", error })?;
      let al_code = master.slave(id).map_err(|error|
        Failure::SlaveStack { slave: number, stage: "AL record", error })?.al_status_code;
      states[poll_slot] = (state.raw(), al_code);
      monitors[poll_slot].observe(state, al_code)
        .map_err(|error| Failure::Monitor { slave: number, error })?;
      poll_slot = (poll_slot + 1) % count;
      next_poll = now + STATE_POLL_US;
    }

    cycles += 1;

    if now >= next_print
    {
      //Round-robin slaves, then chunks within each slave. Never silently truncate.
      let inputs = &iomap[regions[print_slot].inputs.clone()];
      let mut cursor = input_cursors[print_slot];
      let chunk = input_chunk(inputs.len(), &mut cursor);
      let (raw_state, al_code) = states[print_slot];
      let queued = telemetry.queue(format_args!("n={} slave={} AL={:04x}/{:04x} WKC={}/{} I[{}..{}/{}]={:02x?} gap={}us work={}us missed={} logdrop={}\r\n",
        cycles, print_slot + 1, raw_state, al_code, report.wkc, report.expected_wkc,
        chunk.start, chunk.end, inputs.len(), &inputs[chunk.clone()], max_gap, max_work, missed_total, dropped));
      if queued
      {
        input_cursors[print_slot] = cursor;
        print_slot = (print_slot + 1) % count;
      }
      else { dropped += 1; }
      next_print = now + PRINT_US;
    }

    let done = clock.now_us();
    max_work = max_work.max(done - cycle_start);
    let (wake, missed) = schedule.complete(done);
    missed_total += missed;
    late_streak = if missed != 0 { late_streak + 1 } else { 0 };
    
    if late_streak >= 3 
    { 
      return Err(Failure::Timing { missed: missed_total, max_work_us: max_work }); 
    }

    while clock.now_us() < wake
    {
      if let Some(byte) = telemetry.peek()
      {
        if console::try_byte(byte) 
        { 
          telemetry.consumed(); 
        }
      }
      core::hint::spin_loop();
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
//ClockRef
///////////////////////////////////////////////////////////////////////////////////////////
fn idle(mut clock: ClockRef<'_>) -> !
{
  log!("Stopped. Reset the master to retry. An INIT request is not a hardware safety guarantee.\r\n");
  loop { clock.delay_ms(1_000); }
}


///////////////////////////////////////////////////////////////////////////////////////////
//Outer attribute
///////////////////////////////////////////////////////////////////////////////////////////
#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> !
{
  //No access to Master here; slave fail-safe behavior must not depend on cleanup.
  log!("\r\nPANIC: {}. No further PDO traffic; hardware outputs are not guaranteed safe.\r\n", info);
  loop { cortex_m::asm::nop(); }
}
