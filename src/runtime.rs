use core::fmt;
use soem_lib::{SlaveState, master::process_data::ProcessDataReport};

//One bounded UART snapshot. A busy/overflowing queue drops telemetry, never PDOs.
pub struct Telemetry
{
  bytes: [u8; 384],
  len: usize,
  sent: usize,
}

impl Default for Telemetry
{
  fn default() -> Self { Self::new() }
}

impl Telemetry
{
  pub const fn new() -> Self { Self { bytes: [0; 384], len: 0, sent: 0 } }

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
#[path = "runtime/tests.rs"]
mod tests;
