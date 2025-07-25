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
    rx_msg_in: StaticReceiver<Vec<u32, MSG_SIZE>>,
    rx_msg_total: u32,
    proc_msg_out: StaticSender<Vec<u32, MSG_SIZE>>,
  }

  #[init]
  fn init(cx: init::Context) -> (Shared, Local) {
    info!("Program start");
    let mut peripherals = cx.device;
    let mut sio = Sio::new(peripherals.SIO);

    Mono::start(peripherals.TIMER, &peripherals.RESETS);
  
    let (rms, rmr) = RX_MSG_CHANNEL.split();
    let (pms, pmr) = PROC_MSG_CHANNEL.split();

    looper::spawn(pmr).ok();
    dummy_looper::spawn(pms.clone()).ok();

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
      /*
      If I don't have this, the reader pushes messages to the channel, but the processor
      doesn't wake up and see it until the next time an event triggers (e.g. a task timer).
      I don't know if there are unintended side effects.
      */
      unsafe { rp2040_hal::pac::NVIC::unmask(rp2040_hal::pac::Interrupt::SW1_IRQ); }
      msg_emitter(rms);
    });

    (
      Shared {
      },
      Local {
        rx_msg_in: rmr,
        rx_msg_total: 0,
        proc_msg_out: pms,
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

  static PROC_MSG_CHANNEL: StaticChannel<Vec<u32, MSG_SIZE>, CHAN_SIZE> = StaticChannel::new();
  #[task(priority = 1)]
  async fn looper(
    mut ctx: looper::Context,
    mut proc_msg_in: StaticReceiver<Vec<u32, MSG_SIZE>>,
  ) -> () {
    loop {
      // trace!("delay...");
      // Mono::delay(10_u64.millis()).await;
      // trace!("...delay done @{}", Mono::now().ticks()); //DUMMY Sometimes this never gets called and the loop just stops???
      trace!("await proc msg...");
      match proc_msg_in.recv().await {
        Some(msg) => {
          trace!("rx proc msg {}", msg[0]);
        },
        None => {
          error!("rx proc msg fail, channel closed?");
        },
      }
    }
  }

  #[task(priority = 1)]
  async fn dummy_looper(
    mut ctx: dummy_looper::Context,
    mut proc_msg_out: StaticSender<Vec<u32, MSG_SIZE>>,
  ) -> () {
    let mut tx_msg_total: u32 = 0;

    loop {
      // trace!("delay...");
      Mono::delay(1000_u64.millis()).await;
      // trace!("...delay done @{}", Mono::now().ticks()); //DUMMY Sometimes this never gets called and the loop just stops???
      let mut msg = Vec::<u32, MSG_SIZE>::new();
      msg.push(tx_msg_total).ok();
      tx_msg_total += 1;

      trace!("Dummy pushing {} to channel @{}", msg[0], Mono::now().ticks());
      proc_msg_out.try_send(msg).ok();
    }
  }

  #[task(priority = 1, binds = SW1_IRQ, local = [rx_msg_in, rx_msg_total, proc_msg_out])]
  fn msg_receiver(
    mut ctx: msg_receiver::Context,
  ) {
    trace!("msg_receiver await msg @{}", Mono::now().ticks());
    //THINK I could A. loop until I get the expected message, in case it's possible for it to be delayed, or B. loop until I STOP getting messages, in case I missed one before.  ...I guess I could C. loop until I got at least one, then keep looping until they stop.  Hmm.
    let mut count = 0;
    loop {
      match ctx.local.rx_msg_in.try_recv() {
        Ok(msg) => {
          info!("msg_receiver rx msg @{} [{}] ({})", Mono::now().ticks(), msg.len(), msg[0]);
          if msg[0] != *ctx.local.rx_msg_total {
            error!("msg_receiver msg wrong, {} != {}", msg[0], *ctx.local.rx_msg_total);
          }
          *ctx.local.rx_msg_total += 1;
          count += 1;
          info!("msg_receiver tx msg proc");
          ctx.local.proc_msg_out.try_send(msg).ok(); //BUG This does not wake up the task waiting for messages, whereas the msg from dummy_looper does.
          // rp2040_hal::pac::NVIC::pend(rp2040_hal::pac::Interrupt::SW0_IRQ);
        },
        Err(_) =>  {
          // error!("msg_receiver rx error, channel closed?");
          if count > 0 {
            if count > 1 {
              *ctx.local.rx_msg_total += 1; //DUMMY Trigger an error
            }
            // Received at least one message; return
            info!("msg_receiver rx group of {}", count);
            return;
          } else {
            *ctx.local.rx_msg_total += 1; //DUMMY Trigger an error
          }
        },
      }
    }     
  }

  static RX_MSG_CHANNEL: StaticChannel<Vec<u32, MSG_SIZE>, CHAN_SIZE> = StaticChannel::new();
  static mut CORE1_STACK: Stack<4096> = Stack::new();
  //SHAME Blehhhh, typing clutter
  fn msg_emitter(rx_msg_out: StaticSender<Vec<u32, MSG_SIZE>>) {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut _sio = Sio::new(pac.SIO);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, pac.XOSC, pac.CLOCKS, pac.PLL_SYS, pac.PLL_USB, &mut pac.RESETS, &mut watchdog).ok().unwrap();
    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut tx_msg_total: u32 = 0;
    let mut rng = Mt64::new_unseeded();
    loop {
      // Timing seems to be important.  500ms flat delay didn't trigger the problem.
      _delay.delay_us((rng.next_u64() & 1_000_000_u64) as u32);

/*
Seems like the timing needs to be such that delay restart happens after pushing to channel, and before it's received.

[TRACE] Pushing to channel @11616381
[TRACE] ...delay done @11616425
[TRACE] delay...
[INFO ] msg_receiver rx msg @11616473 [0]

It doesn't appear sufficient, not by the logs - I had that pattern 11 times one run, but only the last one triggered the problem.

It may be related to sharing SW0_IRQ on both cores.
*/

      let mut msg = Vec::<u32, MSG_SIZE>::new();
      msg.push(tx_msg_total).ok();
      tx_msg_total += 1;
      trace!("Pushing to channel @{}", Mono::now().ticks());
      rx_msg_out.try_send(msg).ok();
      rp2040_hal::pac::NVIC::pend(rp2040_hal::pac::Interrupt::SW1_IRQ);
    }
  }
}
