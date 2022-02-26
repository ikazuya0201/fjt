#![no_std]
#![no_main]
// #![feature(alloc_error_handler)]

mod init;
mod types;

// use core::alloc::Layout;
use core::fmt::Write;
use core::panic::PanicInfo;

// use alloc_cortex_m::CortexMHeap;
use cortex_m::interrupt::free;
use cortex_m_rt::entry;
use jlink_rtt::Output;
use spin::Lazy;
use stm32f4xx_hal::interrupt;
use uom::si::{electric_potential::volt, f32::ElectricPotential};

use init::{tick_on, BAG, SOLVER};

// static TIMER_TIM5: Mutex<RefCell<Option<Timer<pac::TIM5>>>> = Mutex::new(RefCell::new(None));

// #[global_allocator]
// static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut out = Output::new();
    writeln!(out, "{:?}", info).ok();
    unsafe {
        BAG.operator.force_unlock();
        init::COMMANDER.force_unlock();
        init::WALLS.force_unlock();
    }
    let mut operator = BAG.operator.lock();
    operator.stop();
    operator.turn_on_panic_led();
    writeln!(out, "{:?}", operator).ok();
    writeln!(out, "{:?}", init::COMMANDER.lock()).ok();
    writeln!(out, "{}", init::WALLS.lock()).ok();
    loop {}
}

#[interrupt]
fn TIM5() {
    free(|_| {
        let mut operator = BAG.operator.lock();
        operator.control();
        operator.clear_interrupt();
    });
}

#[interrupt]
fn TIM7() {
    let mut pollar = BAG.pollar.lock();
    pollar.poll();
    pollar.clear_interrupt();
}

#[entry]
fn main() -> ! {
    // let start = cortex_m_rt::heap_start() as usize;
    // let size = 32768; // in bytes
    // unsafe { ALLOCATOR.init(start, size) }

    // initialization
    Lazy::force(&SOLVER);
    Lazy::force(&BAG);

    {
        let mut lock = BAG.operator.lock();
        lock.assert_battery_voltage(ElectricPotential::new::<volt>(3.8));
        core::mem::drop(lock);
    }

    tick_on();
    let mut out = Output::new();
    writeln!(out, "hey!").ok();

    loop {
        SOLVER.lock().search();
    }
}

// #[alloc_error_handler]
// fn oom(layout: Layout) -> ! {
//     panic!("alloc error: {:?}", layout);
// }
