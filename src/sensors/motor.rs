use alloc::rc::Rc;
use core::cell::RefCell;

use components::sensors::Motor;
use embedded_hal::PwmPin;
use quantities::{Quantity, Voltage};

pub trait Voltmeter {
    fn get_voltage(&mut self) -> Voltage;
}

pub struct IMotor<P1, P2, V>
where
    P1: PwmPin,
    P2: PwmPin,
    V: Voltmeter,
{
    pwm_pin1: P1,
    pwm_pin2: P2,
    battery_voltmeter: Rc<RefCell<V>>,
}

impl<P1, P2, V> IMotor<P1, P2, V>
where
    P1: PwmPin,
    P2: PwmPin,
    V: Voltmeter,
{
    pub fn new(pwm_pin1: P1, pwm_pin2: P2, battery_voltmeter: Rc<RefCell<V>>) -> Self {
        Self {
            pwm_pin1,
            pwm_pin2,
            battery_voltmeter,
        }
    }
}

impl<P1, P2, V> Motor for IMotor<P1, P2, V>
where
    P1: PwmPin<Duty = u16>,
    P2: PwmPin<Duty = u16>,
    V: Voltmeter,
{
    fn apply(&mut self, voltage: Voltage) {
        let battery_voltage = self.battery_voltmeter.borrow_mut().get_voltage();
        if voltage.is_positive() {
            let mut ratio = voltage / battery_voltage;
            if ratio > 1.0 {
                ratio = 1.0;
            }
            self.pwm_pin1
                .set_duty(((1.0 - ratio) * self.pwm_pin1.get_max_duty() as f32) as u16);
            self.pwm_pin2.set_duty(self.pwm_pin2.get_max_duty());
        } else {
            let mut ratio = -(voltage / battery_voltage);
            if ratio > 1.0 {
                ratio = 1.0;
            }
            self.pwm_pin1.set_duty(self.pwm_pin1.get_max_duty());
            self.pwm_pin2
                .set_duty(((1.0 - ratio) * self.pwm_pin2.get_max_duty() as f32) as u16);
        }
    }
}
