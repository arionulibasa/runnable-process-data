use core::{cell::RefCell, fmt::{Arguments, Write}};
use cortex_m::interrupt::{self, Mutex};
use stm32f4xx_hal::{pac::UART5, serial::Tx};

static UART: Mutex<RefCell<Option<Tx<UART5>>>> = Mutex::new(RefCell::new(None));

pub fn init(tx: Tx<UART5>)
{
  interrupt::free(|cs| UART.borrow(cs).replace(Some(tx)));
}

//Blocking: use only before cyclic exchange or after the stop request.
pub fn print(args: Arguments<'_>)
{
  interrupt::free(|cs|
  {
    if let Ok(mut uart) = UART.borrow(cs).try_borrow_mut()
    {
      if let Some(tx) = uart.as_mut() { let _ = tx.write_fmt(args); }
    }
  });
}

//One nonblocking attempt. Called only in the cyclic loop's remaining time.
pub fn try_byte(byte: u8) -> bool
{
  interrupt::free(|cs|
  {
    if let Ok(mut uart) = UART.borrow(cs).try_borrow_mut()
    {
      if let Some(tx) = uart.as_mut()
      {
        return embedded_hal_nb::serial::Write::write(tx, byte).is_ok();
      }
    }
    false
  })
}

#[macro_export]
macro_rules! log
{
  ($($arg:tt)*) => { $crate::console::print(core::format_args!($($arg)*)) };
}
