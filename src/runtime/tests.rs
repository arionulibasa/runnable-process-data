#[cfg(test)]

use super::*;

fn good() -> ProcessDataReport
{
  ProcessDataReport { wkc: 3, expected_wkc: 3, frames_sent: 1, frames_received: 1 }
}

#[test]
fn safeop_default_never_requests_op_and_bad_cycles_are_rejected()
{
  let mut monitor = Monitor::new(false, 3);
  assert_eq!(monitor.cycle(0, good()), Ok(false));
  assert_eq!(monitor.observe(SlaveState::SAFE_OP, 0), Ok(()));
  assert!(monitor.observe(SlaveState::OPERATIONAL, 0).is_err());
  for bad in [
    ProcessDataReport { wkc: 0, ..good() },
    ProcessDataReport { frames_received: 0, ..good() },
    ProcessDataReport { wkc: 0, expected_wkc: 0, frames_sent: 0, frames_received: 0 },
  ]
  {
    assert!(monitor.cycle(0, bad).is_err());
  }
}

#[test]
fn op_requires_valid_outputs_first_and_actual_state_confirmation()
{
  let mut monitor = Monitor::new(true, 3);
  assert_eq!(monitor.observe(SlaveState::SAFE_OP, 0), Ok(()));
  assert!(monitor.cycle(0, ProcessDataReport { wkc: 0, ..good() }).is_err());
  assert!(monitor.observe(SlaveState::OPERATIONAL, 0).is_err());
  assert_eq!(monitor.cycle(10, good()), Ok(true));
  assert_eq!(monitor.cycle(100, good()), Ok(false));
  assert_eq!(monitor.observe(SlaveState::SAFE_OP, 0), Ok(()));
  assert_eq!(monitor.cycle(2_000_010, good()), Err(MonitorError::OpTimeout));
  assert_eq!(monitor.observe(SlaveState::OPERATIONAL, 0), Ok(()));
  assert_eq!(monitor.cycle(3_000_000, good()), Ok(false));
  assert!(monitor.observe(SlaveState::SAFE_OP, 0).is_err());
  assert!(monitor.observe(SlaveState::from_raw(0x14), 0x001b).is_err());
}

#[test]
fn group_wkc_is_configured_and_slave_state_tracking_is_independent()
{
  //A bidirectional slave plus an input-only slave: not necessarily 3 * count.
  let report = ProcessDataReport { wkc: 4, expected_wkc: 4, ..good() };
  let mut monitors = [Monitor::new(true, 4); 2];
  for monitor in &mut monitors { assert_eq!(monitor.cycle(0, report), Ok(true)); }
  assert_eq!(monitors[0].observe(SlaveState::OPERATIONAL, 0), Ok(()));
  assert_eq!(monitors[1].observe(SlaveState::SAFE_OP, 0), Ok(()));
  assert_eq!(monitors[0].cycle(2_000_000, report), Ok(false));
  assert_eq!(monitors[1].cycle(2_000_000, report), Err(MonitorError::OpTimeout));
  assert!(monitors[0].observe(SlaveState::SAFE_OP, 0).is_err());
  assert!(monitors[0].cycle(0, good()).is_err());
  assert!(monitors[0].cycle(0, ProcessDataReport { wkc: 3, ..report }).is_err());
  assert!(Monitor::new(false, 0).cycle(0,
    ProcessDataReport { wkc: 0, expected_wkc: 0, ..good() }).is_err());
}

#[test]
fn deadlines_do_not_drift_or_catch_up_in_bursts()
{
  let mut schedule = Schedule::new(0, 10_000);
  assert_eq!(schedule.complete(300), (10_000, 0));
  assert_eq!(schedule.complete(20_000), (20_000, 0));
  assert_eq!(schedule.complete(31_000), (40_000, 1));
  assert_eq!(schedule.complete(75_000), (80_000, 3));
}

#[test]
fn longest_ring_telemetry_record_fits_the_bounded_queue()
{
  use soem_lib::master::redundancy::Route;
  let mut queue = Telemetry::new();
  assert!(queue.queue(format_args!("n={} slave={} AL={:04x}/{:04x} WKC={}/{} lastPDO={:?} I[{}..{}/{}]={:02x?} gap={}us work={}us missed={} logdrop={}\r\n",
    u64::MAX, 8, u16::MAX, u16::MAX, u32::MAX, u32::MAX,
    Some(Route::OriginalViaSecondary), 1008, 1024, 1024, [0xffu8; 16],
    u64::MAX, u64::MAX, u64::MAX, u64::MAX)));
}

#[test]
fn telemetry_is_bounded_and_busy_queue_is_not_overwritten()
{
  let mut queue = Telemetry::new();
  assert!(queue.queue(format_args!("abc")));
  assert!(!queue.queue(format_args!("new")));
  for byte in b"abc"
  {
    assert_eq!(queue.peek(), Some(*byte));
    queue.consumed();
  }
  assert_eq!(queue.peek(), None);
  assert!(!queue.queue(format_args!("{:400}", "overflow")));
  assert_eq!(queue.peek(), None);
  assert!(queue.queue(format_args!("ok")));
  assert_eq!(queue.peek(), Some(b'o'));
}

