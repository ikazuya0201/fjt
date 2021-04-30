#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;
mod alias;
mod init;
mod macros;
mod sensors;

use core::alloc::Layout;
use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic::PanicInfo;

use alloc_cortex_m::CortexMHeap;
use components::prelude::*;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use jlink_rtt::Output;
use lazy_static::lazy_static;
use stm32f4xx_hal::{
    interrupt, stm32,
    timer::{Event, Timer},
};

use init::{init_bag, Bag};

lazy_static! {
    static ref BAG: Bag = init_bag();
}

static TIMER_TIM5: Mutex<RefCell<Option<Timer<stm32::TIM5>>>> = Mutex::new(RefCell::new(None));

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let mut out = Output::new();
    writeln!(out, "{:?}", info).ok();
    loop {}
}

#[interrupt]
fn TIM5() {
    free(|cs| {
        if let Some(ref mut tim5) = TIMER_TIM5.borrow(cs).borrow_mut().deref_mut() {
            tim5.clear_interrupt(Event::TimeOut);
        }
        BAG.operator.tick().expect("Should panic");
    });
}

#[entry]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    let size = 8192; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    BAG.operator.run().ok();

    free(|_cs| {
        cortex_m::peripheral::NVIC::unpend(interrupt::TIM5);
        unsafe {
            cortex_m::interrupt::enable();
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
        }
    });

    loop {
        BAG.operator.run().ok();
    }
}

#[alloc_error_handler]
fn oom(layout: Layout) -> ! {
    panic!("alloc error: {:?}", layout);
}
