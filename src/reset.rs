use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub struct TimedReset<P, D>
{
  pub pin: P,
  pub delay: D,
}

//Owns the reset pin and a delay provider. The LAN9250 ResetPin API is an unsafe
//trait; this implementation supplies a complete, active-low reset sequence.
unsafe impl<P: OutputPin + 'static, D: DelayNs + 'static> lan9250::ResetPin for TimedReset<P, D>
{
  type Error = P::Error;

  fn reset(&mut self) -> Result<(), Self::Error>
  {
    //DS00001913A table 19-11: >=200 us low, and >=25 ms after stable power.
    //Assumes the LAN9250 supplies are already valid when firmware starts.
    self.pin.set_low()?;
    self.delay.delay_ms(50);
    self.pin.set_high()?;
    self.delay.delay_ms(100);
    Ok(())
  }
}

#[cfg(test)]
mod tests
{
  extern crate std;
  use super::*;
  use std::{cell::RefCell, rc::Rc, vec, vec::Vec};

  struct Pin { events: Rc<RefCell<Vec<u32>>>, fail_low: bool, fail_high: bool }
  impl embedded_hal::digital::ErrorType for Pin
  {
    type Error = embedded_hal::digital::ErrorKind;
  }
  impl OutputPin for Pin
  {
    fn set_low(&mut self) -> Result<(), Self::Error>
    {
      if self.fail_low { return Err(Self::Error::Other); }
      self.events.borrow_mut().push(0);
      Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error>
    {
      if self.fail_high { return Err(Self::Error::Other); }
      self.events.borrow_mut().push(1);
      Ok(())
    }
  }
  struct Delay(Rc<RefCell<Vec<u32>>>);
  impl DelayNs for Delay
  {
    fn delay_ns(&mut self, ns: u32) { self.0.borrow_mut().push(ns); }
  }

  #[test]
  fn both_resets_can_share_a_delay_source_and_preserve_pulse_order()
  {
    use lan9250::ResetPin;
    let events = Rc::new(RefCell::new(Vec::new()));
    for _ in 0..2
    {
      let mut reset = TimedReset
      {
        pin: Pin { events: events.clone(), fail_low: false, fail_high: false },
        delay: Delay(events.clone()),
      };
      reset.reset().unwrap();
    }
    assert_eq!(*events.borrow(), vec![0, 50_000_000, 1, 100_000_000, 0, 50_000_000, 1, 100_000_000]);
  }

  #[test]
  fn reset_pin_errors_are_propagated_without_finishing_the_sequence()
  {
    use lan9250::ResetPin;
    for fail_low in [true, false]
    {
      let events = Rc::new(RefCell::new(Vec::new()));
      let mut reset = TimedReset
      {
        pin: Pin { events: events.clone(), fail_low, fail_high: !fail_low },
        delay: Delay(events.clone()),
      };
      assert_eq!(reset.reset(), Err(embedded_hal::digital::ErrorKind::Other));
      assert_eq!(*events.borrow(), if fail_low { vec![] } else { vec![0, 50_000_000] });
    }
  }
}
