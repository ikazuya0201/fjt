use cortex_m::interrupt::free;
use mousecore::administrator::InterruptManager as IInterruptManager;
use stm32f4xx_hal::interrupt;

pub struct InterruptManager;

impl IInterruptManager for InterruptManager {
    fn turn_on(&self) {
        free(|_cs| {
            cortex_m::peripheral::NVIC::unpend(interrupt::TIM5);
            unsafe {
                cortex_m::interrupt::enable();
                cortex_m::peripheral::NVIC::unmask(interrupt::TIM5);
            }
        });
    }

    fn turn_off(&self) {
        free(|_cs| {
            cortex_m::interrupt::disable();
            cortex_m::peripheral::NVIC::mask(interrupt::TIM5);
            cortex_m::peripheral::NVIC::pend(interrupt::TIM5);
        });
    }
}
