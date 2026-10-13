use core::fmt;
use soem_lib::{SlaveState, master::process_data::ProcessDataReport};

//One bounded UART snapshot. A busy/overflowing queue drops telemetry, never PDOs.
pub struct Telemetry
{
  bytes: [u8; 320],
  len: usize,
  sent: usize,
}

impl Default for Telemetry
{
  fn default() -> Self { Self::new() }
}

impl Telemetry
{
  pub const fn new() -> Self { Self { bytes: [0; 320], len: 0, sent: 0 } }

  pub fn queue(&mut self, args: fmt::Arguments<'_>) -> bool
  {
    if self.peek().is_some() { return false; }
    self.len = 0;
    self.sent = 0;
    if fmt::write(self, args).is_err()
    {
      self.len = 0;
      return false;
    }
    true
  }

  pub fn peek(&self) -> Option<u8> { self.bytes.get(self.sent).copied().filter(|_| self.sent < self.len) }
  pub fn consumed(&mut self) { if self.sent < self.len { self.sent += 1; } }
}

impl fmt::Write for Telemetry
{
  fn write_str(&mut self, value: &str) -> fmt::Result
  {
    let end = self.len.checked_add(value.len()).filter(|end| *end <= self.bytes.len()).ok_or(fmt::Error)?;
    self.bytes[self.len..end].copy_from_slice(value.as_bytes());
    self.len = end;
    Ok(())
  }
}

pub struct Schedule
{
  next: u64,
  period: u64,
}

impl Schedule
{
  pub fn new(start: u64, period_us: u32) -> Self
  {
    assert!(period_us != 0);
    Self { next: start + u64::from(period_us), period: u64::from(period_us) }
  }

  //Skip missed starts instead of producing a burst of catch-up frames.
  pub fn complete(&mut self, now: u64) -> (u64, u64)
  {
    let missed = now.saturating_sub(self.next).div_ceil(self.period);
    let wake = self.next + missed * self.period;
    self.next = wake + self.period;
    (wake, missed)
  }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MonitorError
{
  Cycle(ProcessDataReport),
  State { raw: u16, al_code: u16 },
  OpTimeout,
}

#[derive(Clone, Copy)]
pub struct Monitor
{
  enable_op: bool,
  expected_wkc: u32,
  requested_at: Option<u64>,
  confirmed: bool,
}

impl Monitor
{
  pub fn new(enable_op: bool, expected_wkc: u16) -> Self
  {
    Self { enable_op, expected_wkc: u32::from(expected_wkc), requested_at: None, confirmed: false }
  }

  pub fn cycle(&mut self, now: u64, report: ProcessDataReport) -> Result<bool, MonitorError>
  {
    if !report.is_valid() || report.frames_sent == 0 || self.expected_wkc == 0
      || report.expected_wkc != self.expected_wkc
    {
      return Err(MonitorError::Cycle(report));
    }
    if self.enable_op && self.requested_at.is_none()
    {
      self.requested_at = Some(now);
      return Ok(true);
    }
    if self.enable_op && !self.confirmed
      && now.saturating_sub(self.requested_at.unwrap()) >= 2_000_000
    {
      return Err(MonitorError::OpTimeout);
    }
    Ok(false)
  }

  pub fn observe(&mut self, state: SlaveState, al_code: u16) -> Result<(), MonitorError>
  {
    if state.has_error() || al_code != 0
    {
      return Err(MonitorError::State { raw: state.raw(), al_code });
    }
    let expected = if self.enable_op && self.requested_at.is_some() { SlaveState::OPERATIONAL } else { SlaveState::SAFE_OP };
    if state == expected
    {
      self.confirmed |= self.enable_op && state == SlaveState::OPERATIONAL;
      return Ok(());
    }
    if self.enable_op && self.requested_at.is_some() && !self.confirmed && state == SlaveState::SAFE_OP
    {
      return Ok(());
    }
    Err(MonitorError::State { raw: state.raw(), al_code })
  }
}

#[cfg(test)]
mod tests
{
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
}
