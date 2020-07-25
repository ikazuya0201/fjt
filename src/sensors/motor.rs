use core::cell::RefCell;

use components::sensors::Motor;
use embedded_hal::pwm::PwmPin;
use nb::block;
use quantities::{Quantity, Voltage};

pub struct VoltmeterError;

pub trait Voltmeter {
    fn get_voltage(&mut self) -> Voltage;
}

pub struct IMotor<'a, P1, P2, V>
where
    P1: PwmPin,
    P2: PwmPin,
    V: Voltmeter,
{
    pwm_pin1: P1,
    pwm_pin2: P2,
    battery_voltmeter: &'a RefCell<V>,
}

impl<'a, P1, P2, V> IMotor<'a, P1, P2, V>
where
    P1: PwmPin,
    P2: PwmPin,
    V: Voltmeter,
{
    pub fn new(pwm_pin1: P1, pwm_pin2: P2, battery_voltmeter: &'a RefCell<V>) -> Self {
        Self {
            pwm_pin1,
            pwm_pin2,
            battery_voltmeter,
        }
    }
}

impl<'a, P1, P2, V> Motor for IMotor<'a, P1, P2, V>
where
    P1: PwmPin<Duty = u16>,
    P2: PwmPin<Duty = u16>,
    V: Voltmeter,
    <P1 as PwmPin>::Error: core::fmt::Debug,
    <P2 as PwmPin>::Error: core::fmt::Debug,
{
    fn apply(&mut self, voltage: Voltage) {
        let battery_voltage = self.battery_voltmeter.borrow_mut().get_voltage();
        if voltage.is_positive() {
            let mut ratio = voltage / battery_voltage;
            if ratio > 1.0 {
                ratio = 1.0;
            }
            self.pwm_pin1
                .try_set_duty(
                    ((1.0 - ratio) * self.pwm_pin1.try_get_max_duty().unwrap() as f32) as u16,
                )
                .ok();
            self.pwm_pin2
                .try_set_duty(self.pwm_pin2.try_get_max_duty().unwrap())
                .ok();
        } else {
            let mut ratio = -(voltage / battery_voltage);
            if ratio > 1.0 {
                ratio = 1.0;
            }
            self.pwm_pin1
                .try_set_duty(self.pwm_pin1.try_get_max_duty().unwrap())
                .ok();
            self.pwm_pin2
                .try_set_duty(
                    ((1.0 - ratio) * self.pwm_pin2.try_get_max_duty().unwrap() as f32) as u16,
                )
                .ok();
        }
    }
}
