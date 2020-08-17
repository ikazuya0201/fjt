mod agent;
mod maze;
mod search_operator;

use core::cell::RefCell;

use quantities::{Frequency, Time};
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc},
    delay::Delay,
    i2c::I2c,
    prelude::*,
    stm32,
    timer::Timer,
};

use crate::alias::{Agent, Maze, SearchOperator, SensorI2c, Solver, Voltmeter};
use agent::init_agent;
use maze::init_maze;

pub struct Storage<'a> {
    // search_operator: SearchOperator,
    pub agent: Agent<'a>,
    pub maze: Maze,
    // solver: Solver,
    pub voltmeter: RefCell<Voltmeter>,
    pub i2c: RefCell<SensorI2c>,
}

pub fn init<'a>() -> Storage<'a> {
    // let cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();
    // let device_peripherals = stm32::Peripherals::take().unwrap();

    // let rcc = device_peripherals.RCC.constrain();

    // let clocks = rcc
    //     .cfgr
    //     .hclk(100_000_000.hz())
    //     .sysclk(100_000_000.hz())
    //     .pclk1(50_000_000.hz())
    //     .pclk2(100_000_000.hz())
    //     .freeze();

    // let period = Time::from_seconds(0.001);

    // let mut delay = Delay::new(cortex_m_peripherals.SYST, clocks);
    // let gpioa = device_peripherals.GPIOA.split();
    // let gpiob = device_peripherals.GPIOB.split();
    // let gpioc = device_peripherals.GPIOC.split();
    // let gpioh = device_peripherals.GPIOH.split();

    // let mut timer = Timer::tim9(device_peripherals.TIM9, 1.khz(), clocks);

    // let voltmeter = {
    //     let adc = Adc::adc1(device_peripherals.ADC1, true, AdcConfig::default());
    //     let pa7 = gpioa.pa7.into_analog();
    //     RefCell::new(Voltmeter::new(adc, pa7, period, Frequency::from_hertz(1.0)))
    // };

    // let i2c = {
    //     let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    //     let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
    //     let i2c_pins = (scl, sda);
    //     RefCell::new(I2c::i2c1(
    //         device_peripherals.I2C1,
    //         i2c_pins,
    //         400.khz(),
    //         clocks,
    //     ))
    // };

    Storage {
        maze: init_maze(),
        agent: init_agent(),
        voltmeter,
        i2c,
    }
}
