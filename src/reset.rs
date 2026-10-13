use cortex_m::delay::Delay;
use embedded_hal::digital::OutputPin;

pub struct TimedReset<P>
{
  pub pin: P,
  pub delay: Delay,
}

//Owns the reset pin and SysTick delay. The LAN9250 ResetPin API is an unsafe
//trait; this implementation supplies a complete, active-low reset sequence.
unsafe impl<P: OutputPin + 'static> lan9250::ResetPin for TimedReset<P>
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
