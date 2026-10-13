use core::cell::Cell;
use embedded_hal::delay::DelayNs;
use soem_lib::port::clock::MonotonicClock;
use stm32f4xx_hal::{pac::TIM2, prelude::*, rcc::Rcc, timer::CounterUs};

//Counter::start uses ARR = duration - 1, so this period is MAX, not 2^32.
const PERIOD_US: u32 = u32::MAX;

pub struct Tim2Clock
{
  timer: CounterUs<TIM2>,
  previous: Cell<u32>,
  total: Cell<u64>,
}

impl Tim2Clock
{
  pub fn new(timer: TIM2, rcc: &mut Rcc) -> Self
  {
    let mut timer = timer.counter_us(rcc);
    timer.start(PERIOD_US.micros()).unwrap();
    Self { timer, previous: Cell::new(0), total: Cell::new(0) }
  }

  //Main-thread only. Poll at least once per timer period (about 71 minutes).
  //The idle loop continues polling after discovery so wraparound stays tracked.
  fn now_us(&self) -> u64
  {
    let current = self.timer.now().ticks();
    let previous = self.previous.replace(current);
    let delta = if current >= previous
    {
      current - previous
    }
    else
    {
      PERIOD_US - previous + current
    };
    let total = self.total.get().wrapping_add(u64::from(delta));
    self.total.set(total);
    total
  }
}

#[derive(Clone, Copy)]
pub struct ClockRef<'a>(pub &'a Tim2Clock);

impl MonotonicClock for ClockRef<'_>
{
  fn now_us(&self) -> u64
  {
    self.0.now_us()
  }
}

impl DelayNs for ClockRef<'_>
{
  fn delay_ns(&mut self, ns: u32)
  {
    let duration = ns / 1_000 + u32::from(ns % 1_000 != 0);
    let start = self.now_us();
    while self.elapsed_us(start) < duration
    {
      core::hint::spin_loop();
    }
  }
}
