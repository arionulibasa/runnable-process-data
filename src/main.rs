//Build without Rust's standard library; core remains available.
#![no_std]
//Use cortex-m-rt's startup mechanism instead of the normal Rust main entry.
#![no_main]

//Inclue the local module that provides TIM2 timing and delays.
mod clock;
//Include the local module that handles USART2 output and log! macro.
mod console;


//Allow the shared trace buffer to be mutably borrowed with runtime borrow checks.
use core::cell::RefCell;
//Import the attribute that marks the firmware entry function.
use cortex_m_rt::entry;
//Import the delay and SPI traits, together with the SPI mode-0 setting.
use embedded_hal::{delay::DelayNs, spi::{MODE_0, SpiDevice}};
//Combine an SPI bus, chip-select pin, and delay provider into one SPI device.
use embedded_hal_bus::spi::ExclusiveDevice;
//Import the Ethercat master, transport trait, error tyep, slave identifier.
use soem_lib::{Error, Master, FrameTransport, SlaveId, SlaveState};
use soem_lib::config::pdo_map::{PdoMapOptions, PdoSource};
use soem_lib::port::{PortId, clock::MonotonicClock, lan9250::Lan9250Port};
use soem_stm32f4_slaveinfo::trace::{Record, Trace, TracePort};
use soem_stm32f4_processdata::{process_image::{ImageError, SlaveRegions, input_chunk}, runtime::{Monitor, MonitorError, Schedule, Telemetry}};
use stm32f4xx_hal::{pac, prelude::*, rcc::Config};
use clock::{ClockRef, Tim2Clock};
use soem_stm32f4_processdata::reset::TimedReset;


//Represent a controller whose IRQ pin is not connected to an MCU GPIO.
//This struct has no field and owns no hardware pin.
struct NoIrq;

//
unsafe impl lan9250::IntPin for NoIrq {}

//Set the primary controller's six-byte MAC address.
//0x02 indicates a locally administered unicast address.
const MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 1];
//Give the secondary controller a different MAC address. 
const SECONDARY_MAC: [u8; 6] = [0x02, 0, 0, 0, 0, 2];

const MAX_PHYSICAL_SLAVES: usize = 8;
const SLAVE_SLOTS: usize = MAX_PHYSICAL_SLAVES + 1;
//Reserve 1024 bytes for all slaves' output and inputs combined. 
//The actual occupied size determined during PDO mapping.
const IO_MAP_BYTES: usize = 1024;
//Schedule a whole-group process-data exchange every 20,000 us, or 20 ms.
const CYCLE_US: u32 = 20_000;
//Allow 8,000 us for receiving PDO responses, including redundant recovery. 
//The time spent submitting the initial frames is additional.
const PDO_TIMEOUT_US: u32 = 8_000;
//Allow up to 1500 us to observe which ports receive the returning frames
//before choosing certain fallback or paths.
//This observation uses part of the receive budget, not additional time.
const ROUTE_WAIT_US: u32 = 1_500;
//Pass a 4000 us timeout to slave-state register operations, 
//including reading AL state and requesting state changes.
const REGISTER_TIMEOUT_US: u32 = 4_000;
//Poll one slave's AL state and status code every 100,000 us. 
//Rotate through the slaves, so three slaves are each checked 
//approximately once every 300 ms.
const STATE_POLL_US: u64 = 100_000;
//Attempt to queue one slave's diagnostic line every 500,000 us.
//Rotate through the slaves after successfully queueing a line. 
//This controls UART reporting, not how often PDO inputs are updated.
const PRINT_US: u64 = 500_000;
//True when the firmware is compiled with the "enable-op" feature. 
//Allow the cyclic monitor to request OPERATIONAL when its conditions
//are satisfied. Otherwise, keep the test in SAFE_OP.
const ENABLE_OP: bool = cfg!(feature = "enable-op");
//True when the firmware is compiled with the "sii" feature.
//Select PDO descriptions from SII when enabled. 
//Otherwise, use automatic CoE / SII PDO discovery.
const USE_SII: bool = cfg!(feature = "sii");


//Require a nonzero rout-observation interval that is shorter
//than the PDO receive budget, leaving time afterward for recovery.
const _: () = assert!(ROUTE_WAIT_US > 0 && ROUTE_WAIT_US < PDO_TIMEOUT_US);
//Ensure the application's IOmap capacity does not exceed
//the process-data size supported by the mapping library. 
const _: () = assert!(IO_MAP_BYTES <= soem_lib::config::pdo_map::MAX_PROCESS_DATA_BYTES as usize);
//Require at least one physical-slave slot and ensure the configured 
//maximum can be represented by the library's 16-bit slave identifier.
const _: () = assert!(MAX_PHYSICAL_SLAVES > 0 && MAX_PHYSICAL_SLAVES <= u16::MAX as usize);
//Require the nominal time to poll every supported slave to remain
//below to seconds. With eight slaves, this is 800 ms.
//This checks configuration arithmetic; it is not a runtime timing gurantee.
const _: () = assert!(STATE_POLL_US * (MAX_PHYSICAL_SLAVES as u64) < 2_000_000);


//Give this application's Master configuration a shorter name.
//P is the frame transport, and C is the monotonic clock. 
//SLAVE_SLOTS includes the physical slaves plus the aggregate record.
//Reserve capacity for one process-data group and two frame-pool slots.
type DemoMaster<P, C> = Master<P, C, SLAVE_SLOTS, 1, 2>;


//Allow Failure values to be printed using debug formatting, {:?}.
#[derive(Debug)]
enum Failure
{
  //Store an EtherCAT library error together with the operation that failed. 
  //The &'static str holds a fixed stage label, such as "PDO exchange".
  Stack(&'static str, Error),
  //Store the detected slave count when it is outsie the supported range.
  SlaveCount(usize),
  //Identify an IOmap layout or bounds error and the associated slave.
  //Slave 0 is used here for an error involving the aggregate process image.
  Image { slave: usize, error: ImageError },
  //Indicate that mapping produced no process data or no expected WKC.
  EmptyMapping,
  //Store a library error associated with a particular slave.
  //Keep the slave number and operation label to help locate the failure.
  SlaveStack { slave: usize, stage: &'static str, error: Error },
  //Store a failure detected while monitoring a slave's state 
  //or checking cyclic process-data results.
  Monitor { slave: usize, error: MonitorError },
  //Report a stop caused by three consecutive cycle overruns. 
  //Keep the total missed cycle starts and the longest measured cycle work.
  Timing { missed: u64, max_work_us: u64 }
}

//Define readable error messages for Failure using {} formatting.
impl core::fmt::Display for Failure
{
  //Borrow the error and write is message into the supplied formatter. 
  //Return whether formatting succeeded; no allocated String is required.
  //
  //What does '_ mean? It is an inferred lifetime. It tells Rust that the formatter has a 
  //lifetime determined by the surrounding context.
  fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result
  {
    //Select the message corresponding to the stored error variant.
    match self
    {
      //Print the operation label followd by the library error.
      Self::Stack(stage, error)                => write!(out, "{}: {:?}", stage, error),
      //Print the supported physical-slave range and the detected count.
      Self::SlaveCount(count)                  => write!(out, "expected 1..={} physical slaves, found {}", MAX_PHYSICAL_SLAVES, count),
      //Print the affected slave number and its proces-image error.
      Self::Image { slave, error }             => write!(out, "slave {} IOmap: {:?}", slave, error),
      //Explain why an empty or unusable process-data mapping was rejected.
      Self::EmptyMapping                       => write!(out, "group has no process data or no expected WKC"),
      //Print the slave number, faied operation, and library error.
      Self::SlaveStack { slave, stage, error } => write!(out, "slave {} {}: {:?}", slave, stage, error),
      //Print the slave number and monitor error.
      //The {:x?} format displays integer fields in hexadecimal.
      Self::Monitor { slave, error }           => write!(out, "slave {} cyclic monitor: {:x?}", slave, error),
      //Explain the timing stop and include the accumulated timing statistics.
      Self::Timing { missed, max_work_us }     => write!(out, "three consecutive cycle overruns; missed starts={}, maximum work={} us", missed, max_work_us),
    }
  }
}


//For example, Failure::Stack("PDO exchange", Error::Timeout) format as: 
//PDO exchange: Timeout
//This block only formats the error. The code that handles the returned Failure performs the stop and cleanup.


//Mark this function as the firmware entry point called by cortex-m-rt
//after its startup code initializes memory.
#[entry]
//The ! return type means this function never returns.
fn main() -> !
{
  //Take ownership of the STM32 device peripherals. 
  //take() returns None if they were already taken; unwrap would then panic.
  let dp = pac::Peripherals::take().unwrap();
  //Configure the system clock to use the internal HSI oscillator at 16 MHz. 
  //Keep the RCC handle for configuring and enabling peripheral clocks.
  let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(16.MHz()));

  //Split each GPIO peripheral into individual pin handles.
  //The RCC handle lets the HAL enable the corresponding GPIO clocks.
  let gpioa = dp.GPIOA.split(&mut rcc);
  let gpiob = dp.GPIOB.split(&mut rcc);
  let gpioc = dp.GPIOC.split(&mut rcc);

  //Configure PC12 as the primary LAN9250's push-pull chip-select output.
  //Start HIGH because chip select is active LOW.
  let cs_primary = gpioc.pc12.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High);
  
  //Configure PC4 as the secondary LAn9250's chip-select output.
  //Start HIGH so this controller is also deselected.
  let cs_secondary = gpioc.pc4.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High);
  
  //Configure USART2 transmission on PA2 at 115200 baud.
  //The default serial configuration uses 8 data bits, no parity,
  //and one stop bit. Store the transmitter in the console module.
  //unwrap() panics if serial configuration fails.
  console::init(dp.USART2.tx(gpioa.pa2, 115_200.bps(), &mut rcc).unwrap());

  
  //Print the application name and wiring revision. 
  //\r\n sends carriage return and newline to the serial terminal. 
  log!("\r\nsoem-lib processdata: STM32F446RE + LAN9250 (dual-LAN9250 ring v4, SPI3 primary, PA2 console)\r\n");
  
  //Print the clock, console, and intended SPI settings.
  //This message reports the configuration; it does not configure SPI.
  log!("HSI 16 MHz; USART2 PA2 115200 8N1; SPI1/SPI3 4 MHz mode 0\r\n");
  
  //Print the selected operating mode, PDO discovery method, and cycle period. 
  //Also report that outputs remain zero and Distributed Clock are unused.
  log!("Mode={}; mapping={}; period={} us; all output bytes zero; no DC\r\n",
    if ENABLE_OP { "OP ENABLED" } else { "SAFE_OP only" },
    if USE_SII { "SII (requires PDO categories)" } else { "Auto (CoE / SII)" },
    CYCLE_US);
  
  log!("Isolated bench only. Slave watchdog timeout is unknown and is not modified.\r\n");
  log!("Capacity: {} physical slaves, {} IOmap bytes. No device-specific contract.\r\n", MAX_PHYSICAL_SLAVES, IO_MAP_BYTES);

  log!("Primary: SPI3 PB3/PB4/PB5 CS=PC12 RESET=PC11 IRQ=PB7 (SPI polling)\r\n");
  log!("Secondary: SPI1 PA5/PA6/PA7 CS=PC4 RESET=PC5 IRQ=disconnected (SPI polling)\r\n");
  log!("Receive budget={} us; route observation limit={} us\r\n", PDO_TIMEOUT_US, ROUTE_WAIT_US);
  log!("Mailbox frame receive budget={} us; overall SDO budget={} us\r\n", soem_lib::constants::EC_TIMEOUT_MBX_FRAME_US, soem_lib::constants::EC_TIMEOUT_RXM_US);

  //Create the TIM2-based microsecond clock in static storage without a heap. 
  //The 'static reference remains valid for the firmware's entire execution.
  //singleton! allows this storage to be initialized only once.
  let timer: &'static Tim2Clock = cortex_m::singleton!(: Tim2Clock = Tim2Clock::new(dp.TIM2, &mut rcc)).unwrap();

  //Create a lightweight handle for reading time and performing delays.
  //ClockRef is Copy, so its copies all refer to the same TIM2 clock.
  let mut clock = ClockRef(timer);
  
  //Configure primary SPI3 at 4 Mhz using SPI mode 0.
  //The pin tuple is ordered as SCLK, MISO, MOSI. 
  //Select alternate function 6 explicitly for PB3, PB4, and PB5.
  let spi_primary = dp.SPI3.spi((Some(gpiob.pb3.into_alternate::<6>()), Some(gpiob.pb4.into_alternate::<6>()), Some(gpiob.pb5.into_alternate::<6>())), MODE_0, 4.MHz(), &mut rcc);

  //Configure secondary SPI1 at 4 MHz using SPI mode 0. 
  let spi_secondary = dp.SPI1.spi((Some(gpioa.pa5), Some(gpioa.pa6), Some(gpioa.pa7)), MODE_0, 4.MHz(), &mut rcc);
  //
  let spi_primary = ExclusiveDevice::new(spi_primary, cs_primary, clock).unwrap();
  let spi_secondary = ExclusiveDevice::new(spi_secondary, cs_secondary, clock).unwrap();
  
  let reset_primary = TimedReset
  {
    pin: gpioc.pc11.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High),
    delay: clock,
  };

  let reset_secondary = TimedReset
  {
    pin: gpioc.pc5.into_push_pull_output_in_state(stm32f4xx_hal::gpio::PinState::High),
    delay: clock,
  };


  //Construct the primary LAN9250 port with its IRQ pin, reset handler, 
  //SPI device, clock, and MAC address.
  //PB7 is configured as a pull-up input; this application uses SPI polling.
  //Hardware initialization is performed by initialize_nic() below.
  let mut primary = Lan9250Port::new(gpiob.pb7.into_pull_up_input(), reset_primary, spi_primary, clock, MAC);


  //Construct the secondary LAN9250 port using its own hardware resources. 
  //NoIrq represents it physically disconnected interrupt signal.
  let mut secondary = Lan9250Port::new(NoIrq, reset_secondary, spi_secondary, clock, SECONDARY_MAC);
  
  //
  if let Err(error) = initialize_nic("Primary", &mut primary, MAC)
  {
    log!("Primary NIC initialization failed: {:?}\r\n", error);
    idle(clock);
  }
  
  if let Err(error) = initialize_nic("Secondary", &mut secondary, SECONDARY_MAC)
  {
    log!("Secondary NIC initialization failed: {:?}\r\n", error);
    idle(clock);
  }

  log!("Waiting for BOTH fixed 100 Mbps full-duplex links (10 seconds)...\r\n");
  let start = clock.now_us();
  
  loop
  {
    let first = match primary.device_mut().link_up()
    {
      Ok(up) => up,
      Err(error) => { log!("Primary PHY status failed: {:?}\r\n", error); idle(clock); }
    };
    
    let second = match secondary.device_mut().link_up()
    {
      Ok(up) => up,
      Err(error) => { log!("Secondary PHY status failed: {:?}\r\n", error); idle(clock); }
    };
    
    if first && second 
    { 
      break; 
    }
    
    if clock.elapsed_us(start) >= 10_000_000
    {
      log!("Link wait expired: Primary={} Secondary={}. Check both ends of the slave chain.\r\n", first, second);
      idle(clock);
    }
    
    clock.delay_ms(10);
  }

  log!("Both links up. Primary -> slaves -> Secondary.\r\n");

  let trace = RefCell::new(Trace::new());
  let primary = TracePort::new_on(primary, &trace, PortId::Primary);
  let secondary = TracePort::new_on(secondary, &trace, PortId::Secondary);
  
  let mut master: DemoMaster<_, _> = match Master::new_redundant(primary, secondary, clock, MAC, ROUTE_WAIT_US)
  {
    Ok(master) => master,
    Err(error) => { log!("Redundant master initialization failed: {:?}\r\n", error); idle(clock); }
  };
  
  let mut iomap = [0; IO_MAP_BYTES];

  if let Err(error) = run(&mut master, &clock, &mut iomap)
  {
    let mut saved: [Option<Record>; 32] = [None; 32];
    
    for (out, record) in saved.iter_mut().zip(trace.borrow().iter()) 
    { 
      *out = Some(*record); 
    }
    
    let count = master.slave_count().saturating_sub(1).min(MAX_PHYSICAL_SLAVES);
    let mut prior_al = [(0u16, 0u16); MAX_PHYSICAL_SLAVES];
    
    for (slot, prior) in prior_al[..count].iter_mut().enumerate()
    {
      if let Ok(slave) = master.slave(SlaveId((slot + 1) as u16))
      {
        *prior = (slave.state.raw(), slave.al_status_code);
      }
    }
    let last_completion = master.last_redundant_response();
    
    let stop = master.write_state(SlaveId::ALL, SlaveState::INIT, REGISTER_TIMEOUT_US);
    
    log!("\r\nSTOP: {}\r\n", error);
    log!("Broadcast INIT request={:?}. No automatic restart.\r\n", stop);
    log!("Last completed datagram before stop (may predate failure): {:?}\r\n", last_completion);
    
    for (slot, (state, code)) in prior_al[..count].iter().enumerate()
    {
      let id = SlaveId((slot + 1) as u16);
      let actual = master.read_state(id, REGISTER_TIMEOUT_US);
      log!("Slave {}: cached AL={:04x}/{:04x}; INIT readback={:?}\r\n", id.0, state, code, actual);
      
      if let Ok(slave) = master.slave(id)
      {
        log!("  AL={:04x}/{:04x}; mapped O={} I={} bytes\r\n", slave.state.raw(), slave.al_status_code, slave.process_data.output_bytes, slave.process_data.input_bytes);
      }
    }

    log!("Trace before stop request:\r\n");
    
    for record in saved.iter().flatten() { log!("  {}\r\n", record); }
  }

  idle(clock)
}



///////////////////////////////////////////////////////////////////////////////////////////
//Initialize one LAN9250 controller and configure its PHY.
//
//I is the interrupt-pin type.
//R is the reset-handler type.
//S is the SPI-device type.
//C is the clock type stored in the port.
//
//name identifies the controller in diagnostic messages.
//port provides mutable access to the controller.
//mac supplies the siz-byte MAC addrss to program.
//
//Return Ok(()) on success, or a LAN9250 driver error.
//S::Error and RLLError preserve the underlying SPI and reset error types.
///////////////////////////////////////////////////////////////////////////////////////////
fn initialize_nic<I, R, S, C>(name: &str, port: &mut Lan9250Port<lan9250::Lan9250<I, R, S>, C>, mac: [u8; 6]) -> Result<(), lan9250::Error<S::Error, R::Error>>
where I: lan9250::IntPin, R: lan9250::ResetPin, S: SpiDevice
{
  log!("Resetting {} LAN9250 and checking SPI/READY...\r\n", name);
  
  let device = port.device_mut();
  device.init(mac)?;
  let id = device.read_reg(0x0050)?;
  
  log!("{} LAN9250 ID_REV: 0x{:08x}\r\n", name, id);
  device.write_phy(0, 0x2100)
}




///////////////////////////////////////////////////////////////////////////////////////////
//Run slave discovery, PDO configuration, and cyclic process-data exchange.
//
//P must provide frame transport using two frame-pool slots.
//C must provide a monotonic clock for measuring elapsed time.
//
//Borrow the master mutably because communication updates its internal state.
//Borrow the clock for timing and the IOmap mutably for process-data updates.
//Return Failure if initialization or cyclic operation fails.
///////////////////////////////////////////////////////////////////////////////////////////
fn run<P: FrameTransport<2>, C: MonotonicClock>(master: &mut DemoMaster<P, C>, clock: &C, iomap: &mut [u8; IO_MAP_BYTES]) -> Result<(), Failure>
{
  //Anounce slave discovery and the initial target Ethercat state.
  log!("Discovering slaves and confirming PRE_OP...\r\n");

  //Discover physical slaves, assign configured address, read SII metadata,
  //configure their mailboxes, and confirm that they reach PRE_OP.
  //The returned count excludes the aggregate record at slave index 0.
  //Convert a library error into an application Failure labeled "discovery".
  //The ? returns that Failure immediately if discovery fails.
  let count = master.configure_slaves().map_err(|e| Failure::Stack("discovery", e))?;

  //Reject an empty network or a count above the applications's capacity.
  if count == 0 || count > MAX_PHYSICAL_SLAVES { return Err(Failure::SlaveCount(count)); }
  
  //Visit physical slaves numbered 1 through count, including count tself.
  for number in 1..=count
  {
    //Convert the number into the library's SlaveId type and borrow its record.
    //This accesses metadata already stored byt discovery.
    //If lookup fails, preserve the slave number operation label.
    let slave = master.slave(SlaveId(number as u16)).map_err(|error| Failure::SlaveStack { slave: number, stage: "record", error })?;
    
    //Print the slave identifier, name, vendor ID, product code, 
    //revision, serial number, and advertised CoE mailbox support.
    //
    //slave.id.0 access the numeric value wrapped by SlaveId.
    //If the stored name is not valid UTF-8, print "<non-UTF8" instead.
    //Vendor and product identifiers are printed as eight-digit hexadecimal.
    log!("Slave {}: {}; vendor=0x{:08x} product=0x{:08x} revision={} serial={} CoE={}\r\n",
      slave.id.0, slave.name.as_str().unwrap_or("<non-UTF8>"), slave.manufacturer,
      slave.product_code, slave.revision, slave.serial, slave.supports_coe());
  }
  
  //Announce configuration of process-data group 0.
  //The mapper uses the slaves' PDO descriptions and confirms SAFE_OP. 
  log!("Mapping group 0 from slave descriptions; requesting SAFE_OP...\r\n");
  
  //Build the options passed to the PDO mapper.
  let options = PdoMapOptions
  {
    //Use SII EEPROM PDO descriptions when the "sii" features is enabled. 
    //Otherwise, try CoE first and use SII when CoE mapping is unavailable.
    //or both CoE PDO directions contain zero bits.
    source: if USE_SII 
    { 
      PdoSource::Sii 
    } 
    else 
    { 
      PdoSource::Auto 
    },
    
    //Keep the remaining options at their defaults:
    //packed = false and segment_bytes = the library's maximum.
    ..PdoMapOptions::default()
  };
  
  //Discover each slave's PDO layout and assign its IOmap regions.
  //Configure the slave's process-data SyncManagers and FMMUs,
  //then request and confirm SAFE_OP for the selected slaves.
  //ON success, zero the occupied IOmap region and return its byte count.
  //Convert any error into Failure identifying this configuration stage.
  let used = master.configure_process_data_group_with_options(0, iomap, options).map_err(|e| Failure::Stack("PDO mapping / SAFE_OP", e))?;
  
  //Read the expected working counter calculated for the configured group.
  //Cyclic monitoring later compares received WKC agains this value.
  let expected_wkc = master.expected_wkc();
  

  //Reject a mapping with no occupied process-data bytes
  //or no expected working counter contribution.
  if used == 0 || expected_wkc == 0 
  { 
    return Err(Failure::EmptyMapping); 
  }

  //Read slave record 0, which holds aggregate process-data information.
  //This is a software summary record, not a physical slave.
  let aggregate = master.slave(SlaveId::ALL).map_err(|e| Failure::Stack("group record", e))?;
  
  //Get the combined output-region size for all slaves.
  //Convert it to usize for use in array and slice indexing.
  let output_bytes = aggregate.process_data.output_bytes as usize;
  
  //Check that the occupied image fits the supplied buffer
  //and that its output region does not exceed the total occupied size.
  if used > iomap.len() || output_bytes > used
  {
    //Report an aggregate IOmap bounds error using slave number 0.
    return Err(Failure::Image { slave: 0, error: ImageError::Bounds });
  }
  
  //Reserve input and output byte ranges for every supported physical slave.
  //from_fn() creates a separate default SlaveRegions value for each slot.
  //The default ranges are empty; te loop below fills discoverd slaves' entries.
  let mut regions: [SlaveRegions; MAX_PHYSICAL_SLAVES] = core::array::from_fn(|_| SlaveRegions::default());

  //Create one monitor per supported physical slave.
  //Each monitor tracks that slave's state and optional transition to OP. 
  //All monitors use the same expected WKC for the whole group's PDO exchange.
  let mut monitors = [Monitor::new(ENABLE_OP, expected_wkc); MAX_PHYSICAL_SLAVES];

  //Reserve a cached AL state and AL status-code pair for each slave.
  //Initialize both values to zero until populated below.
  let mut states = [(0u16, 0u16); MAX_PHYSICAL_SLAVES];
  

  //Initialize entries only for the physical slaves actually discovered. 
  //The range excludes count, giving array indexes 0 through count - 1. 
  for slot in 0..count
  {
    //Convert the zero-based array index into a physical slave number.
    let number = slot + 1;
    
    //Borrow the slave record containing the completed PDO mapping. 
    //Attach the slave number and "mapped record" label if lookup fails.
    let slave = master.slave(SlaveId(number as u16)).map_err(|error| Failure::SlaveStack { slave: number, stage: "mapped record", error })?;

    //Build and validate this slave's output and input byte ranges.
    //Check their bounds, byte alignment, and placement in the group IOmap.
    //Return a slave-specific image error it its layout is invalid.
    regions[slot] = SlaveRegions::new(&slave.process_data, output_bytes, used).map_err(|error| Failure::Image { slave: number, error })?;

    //Cache the AL state and status code recorded during configuration.
    //This copies existing metadata; it does not perform another register read.
    states[slot] = (slave.state.raw(), slave.al_status_code);

    //Check the recorded state and status code using this slave's monitor.
    //At this stage, require SAFE_OP with no AL error or nonzero status code.
    //An OP request, if enabled, happens later during cyclic exchange.
    monitors[slot].observe(slave.state, slave.al_status_code).map_err(|error| Failure::Monitor { slave: number, error })?;

    
    //Print this slave's output and input sizes and starting IOmap offsets.
    //Byte lengths describe occupied storage; bit counts describe mapped data.
    //Print the cached AL state and status code in hexadecimal.
    log!("Slave {}: O={} bytes/{} bits at {}; I={} bytes/{} bits at {}; AL={:04x}/{:04x}\r\n",
      number, regions[slot].outputs.len(), slave.process_data.output_bits, regions[slot].outputs.start,
      regions[slot].inputs.len(), slave.process_data.input_bits, regions[slot].inputs.start,
      states[slot].0, states[slot].1);
  }
  

  //Print the number of configured slaves and the group's total PDO sizes.
  //The input size is the total occupied size minus the output-region size.
  log!("{} slaves SAFE_OP; group O={} I={} bytes; expected WKC={}\r\n", count, output_bytes, used - output_bytes, expected_wkc);

  //Announce that cyclic exchange will use nonblocking UART reporting 
  //and that an invalid working counter will stop the test.
  //This log call itself still uses the blocking startup logging function.
  log!("Starting cyclic exchange; UART is now nonblocking. Bad WKC stops the test.\r\n");



  //Read the current time as the reference for starting cyclic operation. 
  let start = clock.now_us();

  //Create the periodic schedule using the configured cycle interval.
  //Its first scheduled next-cycle start is start + CYCLE_US. 
  let mut schedule = Schedule::new(start, CYCLE_US);

  //Crete an empty, fixed-capacity buffer for one UART diagnostic message.
  //Queued bytes are transmitted gradually during the cycle's remaining time.
  let mut telemetry = Telemetry::new();

  //Make the first slave-state poll due during the first cycle.
  let mut next_poll = start;


  //Make the first diagnostic message due during the first cycle.
  let mut next_print = start;

  //Count cycles that reach the reporting stage after the exchange,
  //monitor checks, and any scheduled state poll have succeeded.
  let mut cycles = 0u64;
  
  //Keep the previous cycle's start time for measuring start-to-start gaps.
  //Use the initial reference time until the first cycle begins.
  let mut last_start = start;

  //Track the largest observed interval between cycle starts, in microseconds.
  let mut max_gap = 0u64;

  //Track the longest cycle work duration, in microseconds.
  //This includes PDO exchange, monitoring, and preparing diagnostic output,
  //but excludes the final waiting period that drains UART bytes.
  let mut max_work = 0u64;

  //Accumulate the number of scheduled cycle starts skipped due to overruns.
  let mut missed_total = 0u64;

  //Count consecutinve cycles whose work misses a scheduled start. 
  //A cycle without a missed start resets this count to zero.
  //Three consecutive overruns cause the test to stop.
  let mut late_streak = 0u8;

  //Count diagnostic messages that could not be queued.
  //This counts dropped UART reports, not lost Ethercat frames.
  let mut dropped = 0u64;

  //Select the first physical slave for the next AL state poll.
  //Array slot 0 corresponds to physical slave 1.
  let mut poll_slot = 0usize;

  //Select the first physical slave for the next UART diagnostic message.
  //Polling and printing advance independently.
  let mut print_slot = 0usize;

  //Keep a separate input-print position for each supported slave. 
  let mut input_cursors = [0usize; MAX_PHYSICAL_SLAVES];
  
  //Repeat process-data exchange until an error returns from run().
  loop
  {
    //Record when this cycle begins.
    let cycle_start = clock.now_us();

    //Keep the largest observed interval between consecutive cycle starts.
    max_gap = max_gap.max(cycle_start - last_start);
    
    //Save this start time for the next cycle's gap calculation.
    last_start = cycle_start;
    
    //Set all slaves' output bytes to zero before sending this cycle.
    //This slice covers only the output region, not the input region.
    iomap[..output_bytes].fill(0);
    
    //Exchange the occupied process-data image for the whole group.
    //The library sends the requests and receives their responses.
    //For a valid exchange, it copie received input bytes into the IOmap.
    //Return immediately if the exchange produces a library error.
    let report = master.exchange_processdata(&mut iomap[..used], PDO_TIMEOUT_US).map_err(|e| Failure::Stack("PDO exchange", e))?;
    
    //Save the last completed redundant data's route for diagnosics.
    //map() extracts the route from Some(response), or preserves None.
    //Capture it before later register operations can change this information.
    let pdo_route = master.last_redundant_response().map(|response| response.route);
    
    //Read the time after PDO exchange for monitoring and scheduled tasks.
    let now = clock.now_us();

    //Start with no request to transition the slaves to OPERATIONAL. 
    let mut request_op = false;
    
    //Check every discovered slave's monitor using the same group PDO report.
    //enumerate provides the array slot alongside its mutable monitor.
    for (slot, monitor) in monitors[..count].iter_mut().enumerate()
    {
      //Validate the exchange and check any pending OP-transition timeout.
      //A monitor returns true when it first requests OP, if OP is enabled.
      //Boolean |= keeps request_op true if any monitor requests it. 
      //Return a slave-specific Failure if monitoring detects an error. 
      request_op |= monitor.cycle(now, report).map_err(|error| Failure::Monitor { slave: slot + 1, error })?;
    }
    
    //Send one broadcast OP request if at least one monitor requested it.
    //With the default SAFE_OP-only configuration, this block is skipped.
    if request_op
    {
      //Request OPERATIONAL for all slaves.
      //Actual state 
      master.write_state(SlaveId::ALL, SlaveState::OPERATIONAL, REGISTER_TIMEOUT_US)
        .map_err(|e| Failure::Stack("OP request", e))?;
      next_poll = now;
    }


    if now >= next_poll
    {
      let number = poll_slot + 1;
      let id = SlaveId(number as u16);
      let state = master.read_state(id, REGISTER_TIMEOUT_US).map_err(|error| Failure::SlaveStack { slave: number, stage: "AL poll", error })?;
      let al_code = master.slave(id).map_err(|error| Failure::SlaveStack { slave: number, stage: "AL record", error })?.al_status_code;
      
      states[poll_slot] = (state.raw(), al_code);
      monitors[poll_slot].observe(state, al_code).map_err(|error| Failure::Monitor { slave: number, error })?;
      poll_slot = (poll_slot + 1) % count;
      next_poll = now + STATE_POLL_US;
    }

    cycles += 1;

    if now >= next_print
    {
      let inputs = &iomap[regions[print_slot].inputs.clone()];
      let mut cursor = input_cursors[print_slot];
      let chunk = input_chunk(inputs.len(), &mut cursor);
      let (raw_state, al_code) = states[print_slot];
      let queued = telemetry.queue(format_args!("n={} slave={} AL={:04x}/{:04x} WKC={}/{} lastPDO={:?} I[{}..{}/{}]={:02x?} gap={}us work={}us missed={} logdrop={}\r\n",
        cycles, print_slot + 1, raw_state, al_code, report.wkc, report.expected_wkc, pdo_route,
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


fn idle(mut clock: ClockRef<'_>) -> !
{
  log!("Stopped. Reset the master to retry. An INIT request is not a hardware safety guarantee.\r\n");
  loop { clock.delay_ms(1_000); }
}


#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> !
{
  log!("\r\nPANIC: {}. No further PDO traffic; hardware outputs are not guaranteed safe.\r\n", info);
  loop { cortex_m::asm::nop(); }
}
