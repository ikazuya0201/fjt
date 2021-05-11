use alloc::rc::Rc;
use core::cell::RefCell;
use core::marker::PhantomData;

use crate::wait_ok;
use components::{sensors::DistanceSensor, types::data::Pose, utils::sample::Sample};
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
    digital::v2::OutputPin,
};
use nb::block;
use uom::si::{f32::Length, length::meter};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Waiting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VL6180XError;

pub struct VL6180X<T, U>
where
    T: Write + WriteRead,
    U: OutputPin,
{
    i2c: Rc<RefCell<T>>,
    enable_pin: U,
    device_address: u8,
    state: State,
    pose: Pose,
    correction_weight: f32,
}

impl<T, U> VL6180X<T, U>
where
    T: Write + WriteRead,
    U: OutputPin,
    VL6180XError: From<<T as Write>::Error> + From<<T as WriteRead>::Error>,
{
    //register addresses
    const SYSTEM__INTERRUPT_CLEAR: u16 = 0x0015;
    const SYSTEM__FRESH_OUT_OF_RESET: u16 = 0x0016;
    const SYSTEM__INTERRUPT_CONFIG_GPIO: u16 = 0x0014;
    const SYSRANGE__START: u16 = 0x0018;
    const RESULT__INTERRUPT_STATUS_GPIO: u16 = 0x004F;
    const RESULT__RANGE_VAL: u16 = 0x0062;
    const I2C_SLAVE__DEVICE_ADDRESS: u16 = 0x0212;

    //default address of VL6180X
    const DEFAULT_ADDRESS: u8 = 0x29;

    //value of SYSTEM_FRESH_OUT_OF_RESET register in idle mode
    const IDLE_VALUE: u8 = 0x01;

    //standard deviation
    const STANDARD_DEVIATION: Length = Length {
        dimension: PhantomData,
        units: PhantomData,
        value: 0.05,
    };

    pub fn new<'b, V: DelayMs<u32>>(
        i2c: Rc<RefCell<T>>,
        enable_pin: U,
        delay: &'b mut V,
        new_address: u8, //7bit address
        pose: Pose,
        correction_weight: f32,
    ) -> Self {
        let mut tof = Self {
            i2c,
            enable_pin,
            device_address: Self::DEFAULT_ADDRESS,
            state: State::Idle,
            pose,
            correction_weight,
        };

        tof.init(delay, new_address);

        tof
    }

    pub fn init<'b, V: DelayMs<u32>>(&mut self, delay: &'b mut V, new_address: u8) {
        wait_ok!(self.enable_pin.set_low());

        wait_ok!(self.enable_pin.set_high()); //enable vl6180x

        delay.delay_ms(1); //wait for waking up

        wait_ok!(block!(self.check_if_working()));

        wait_ok!(self.configure_tuning());

        wait_ok!(self.configure_range_mode());

        wait_ok!(self.update_device_address(new_address));
    }

    fn update_device_address(&mut self, new_address: u8) -> Result<(), VL6180XError> {
        self.write_to_register(Self::I2C_SLAVE__DEVICE_ADDRESS, new_address)?;
        self.device_address = new_address;
        Ok(())
    }

    fn check_if_working(&mut self) -> nb::Result<(), VL6180XError> {
        let data = self.read_from_registers(Self::SYSTEM__FRESH_OUT_OF_RESET)?;
        if data == Self::IDLE_VALUE {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn configure_range_mode(&mut self) -> Result<(), VL6180XError> {
        //enable internal interrupt of range mode
        self.write_to_register(Self::SYSTEM__INTERRUPT_CONFIG_GPIO, 0x04)?;
        Ok(())
    }

    fn configure_tuning(&mut self) -> Result<(), VL6180XError> {
        self.write_to_register(0x0207, 0x01)?;
        self.write_to_register(0x0208, 0x01)?;
        self.write_to_register(0x0133, 0x01)?;
        self.write_to_register(0x0096, 0x00)?;
        self.write_to_register(0x0097, 0xFD)?;
        self.write_to_register(0x00e3, 0x00)?;
        self.write_to_register(0x00e4, 0x04)?;
        self.write_to_register(0x00e5, 0x02)?;
        self.write_to_register(0x00e6, 0x01)?;
        self.write_to_register(0x00e7, 0x03)?;
        self.write_to_register(0x00f5, 0x02)?;
        self.write_to_register(0x00D9, 0x05)?;
        self.write_to_register(0x00DB, 0xCE)?;
        self.write_to_register(0x00DC, 0x03)?;
        self.write_to_register(0x00DD, 0xF8)?;
        self.write_to_register(0x009f, 0x00)?;
        self.write_to_register(0x00a3, 0x3c)?;
        self.write_to_register(0x00b7, 0x00)?;
        self.write_to_register(0x00bb, 0x3c)?;
        self.write_to_register(0x00b2, 0x09)?;
        self.write_to_register(0x00ca, 0x09)?;
        self.write_to_register(0x0198, 0x01)?;
        self.write_to_register(0x01b0, 0x17)?;
        self.write_to_register(0x01ad, 0x00)?;
        self.write_to_register(0x00FF, 0x05)?;
        self.write_to_register(0x0100, 0x05)?;
        self.write_to_register(0x0199, 0x05)?;
        self.write_to_register(0x0109, 0x07)?;
        self.write_to_register(0x010a, 0x30)?;
        self.write_to_register(0x003f, 0x46)?;
        self.write_to_register(0x01a6, 0x1b)?;
        self.write_to_register(0x01ac, 0x3e)?;
        self.write_to_register(0x01a7, 0x1f)?;
        self.write_to_register(0x0103, 0x01)?;
        self.write_to_register(0x0030, 0x00)?;
        self.write_to_register(0x001b, 0x0A)?;
        self.write_to_register(0x003e, 0x0A)?;
        self.write_to_register(0x0131, 0x04)?;
        self.write_to_register(0x0011, 0x10)?;
        self.write_to_register(0x0014, 0x24)?;
        self.write_to_register(0x0031, 0xFF)?;
        self.write_to_register(0x00d2, 0x01)?;
        self.write_to_register(0x00f2, 0x01)?;
        Ok(())
    }

    fn write_to_register(&mut self, address: u16, data: u8) -> Result<(), VL6180XError> {
        let (address_h, address_l) = self.split_register_address(address);
        let mut i2c = self.i2c.borrow_mut();
        i2c.write(self.device_address, &[address_h, address_l, data])?;
        Ok(())
    }

    fn read_from_registers<'b>(&mut self, address: u16) -> Result<u8, VL6180XError> {
        let (address_h, address_l) = self.split_register_address(address);
        let mut i2c = self.i2c.borrow_mut();
        let mut buffer = [0];
        i2c.write_read(self.device_address, &[address_h, address_l], &mut buffer)?;
        Ok(buffer[0])
    }

    fn split_register_address(&self, address: u16) -> (u8, u8) {
        //(higher,lower)
        ((address >> 8) as u8, address as u8)
    }
}

impl<T, U> DistanceSensor for VL6180X<T, U>
where
    T: Write + WriteRead,
    U: OutputPin,
    VL6180XError: From<<T as Write>::Error> + From<<T as WriteRead>::Error>,
{
    type Error = VL6180XError;

    fn pose(&self) -> Pose {
        self.pose
    }

    fn get_distance(&mut self) -> nb::Result<Sample<Length>, Self::Error> {
        use State::*;

        match self.state {
            Idle => {
                //start polling
                self.write_to_register(Self::SYSRANGE__START, 0x01)?;
                self.state = Waiting;
                Err(nb::Error::WouldBlock)
            }
            Waiting => {
                let data = self.read_from_registers(Self::RESULT__INTERRUPT_STATUS_GPIO)?;
                if data & 0x04 != 0x04 {
                    return Err(nb::Error::WouldBlock);
                }
                //get result
                let distance = self.read_from_registers(Self::RESULT__RANGE_VAL)?;

                //teach VL6180X to finish polling
                self.write_to_register(Self::SYSTEM__INTERRUPT_CLEAR, 0x07)?;

                self.state = Idle;

                Ok(Sample {
                    mean: self.correction_weight * Length::new::<meter>(distance as f32 / 1000.0),
                    standard_deviation: Self::STANDARD_DEVIATION,
                })
            }
        }
    }
}
