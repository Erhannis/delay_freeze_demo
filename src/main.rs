#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

use rtic_monotonics::rp2040::prelude::*;

rp2040_timer_monotonic!(Mono);

#[rtic::app(
  // TODO: Replace `some_hal::pac` with the path to the PAC
  device = rp2040_hal::pac,
  // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
  // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
  dispatchers = [SW0_IRQ], // Note that if you change this, update start of core 1
)]
mod app {
  pub const MSG_SIZE: usize = 1;
  pub const CHAN_SIZE: usize = 10;

  use crate::{Mono, XOSC_CRYSTAL_FREQ};

  use defmt::{debug, error, info, trace};
  use heapless::Vec;
  use defmt_rtt as _;
  use rand_mt::Mt64;

  use panic_probe as _;
  
  use rtic_monotonics::{rtic_time::monotonic::TimerQueueBasedInstant, Monotonic};
  use thingbuf::mpsc::{StaticChannel, StaticReceiver, StaticSender};
  
  use rp2040_hal::{self as bsp, fugit::ExtU64, multicore::{Multicore, Stack}};
  
  use bsp::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
  };
  
  // Shared resources go here
  #[shared]
  struct Shared {
  }

  // Local resources go here
  #[local]
  struct Local {
  }

  #[init]
  fn init(cx: init::Context) -> (Shared, Local) {
    info!("Program start");
    let mut peripherals = cx.device;
    let mut sio = Sio::new(peripherals.SIO);

    Mono::start(peripherals.TIMER, &peripherals.RESETS);
  
    let (rms, rmr) = RX_MSG_CHANNEL.split();

    looper::spawn().ok();
    msg_receiver::spawn(rmr).ok();

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
      /*
      If I don't have this, the reader pushes messages to the channel, but the processor
      doesn't wake up and see it until the next time an event triggers (e.g. a task timer).
      I don't know if there are unintended side effects.
      */
      unsafe { rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::SW0_IRQ); }
      msg_emitter(rms);
    });

    (
      Shared {
      },
      Local {
      },
    )
  }

  #[idle()]
  fn idle(mut cx: idle::Context) -> ! {
    defmt::info!("idle");
    loop {
      // cortex_m::asm::wfi(); //THINK ??
    }
  }

  #[task(priority = 1)]
  async fn looper(mut ctx: looper::Context) -> () {
    loop {
      trace!("delay...");
      Mono::delay(10_u64.millis()).await;
      trace!("...delay done @{}", Mono::now().ticks()); //DUMMY Sometimes this never gets called and the loop just stops???
    }
  }
  
  #[task(priority = 1)]
  async fn msg_receiver(
    mut ctx: msg_receiver::Context,
    rx_msg_in: StaticReceiver<Vec<u8, MSG_SIZE>>,
  ) {
    loop {
      trace!("msg_receiver await msg @{}", Mono::now().ticks());
      match rx_msg_in.recv().await {
        Some(msg) => {
          info!("msg_receiver rx msg @{} [{}]", Mono::now().ticks(), msg.len());
        },
        None =>  {
          error!("msg_receiver rx error, channel closed?");
        },
      }          
    }
  }

  static RX_MSG_CHANNEL: StaticChannel<Vec<u8, MSG_SIZE>, CHAN_SIZE> = StaticChannel::new();
  static mut CORE1_STACK: Stack<4096> = Stack::new();
  //SHAME Blehhhh, typing clutter
  fn msg_emitter(rx_msg_out: StaticSender<Vec<u8, MSG_SIZE>>) {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut _sio = Sio::new(pac.SIO);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog).ok().unwrap();
    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut rng = Mt64::new_unseeded();
    loop {
      // Timing seems to be important.  500ms flat delay didn't trigger the problem.
      _delay.delay_us((rng.next_u64() & 10_000_u64) as u32);

/*
Seems like the timing needs to be such that delay restart happens after pushing to channel, and before it's received.

[TRACE] Pushing to channel @11616381
[TRACE] ...delay done @11616425
[TRACE] delay...
[INFO ] msg_receiver rx msg @11616473 [0]

It doesn't appear sufficient, not by the logs - I had that pattern 11 times one run, but only the last one triggered the problem.

It may be related to sharing SW0_IRQ on both cores.
*/

      let buffer = [];
      let mut msg = Vec::<u8, MSG_SIZE>::new();
      for &byte in buffer.iter().take(MSG_SIZE) {
        msg.push(byte).ok();
      }
      trace!("Pushing to channel @{}", Mono::now().ticks());
      rx_msg_out.try_send(msg).ok();
    }
  }
}
