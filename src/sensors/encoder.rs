use components::sensors::{Encoder, EncoderError};
use embedded_hal::qei::Qei;
use quantities::{Angle, Distance};

pub struct MA702GQ<Q>
where
    Q: Qei,
{
    qei: Q,
    wheel_radius: Distance,
    before_count: u16,
}

impl<Q> MA702GQ<Q>
where
    Q: Qei,
{
    const RESOLUTION_PER_ROTATION: f32 = 1024.0;

    pub fn new(qei: Q, wheel_radius: Distance) -> Self {
        Self {
            qei,
            wheel_radius,
            before_count: 0,
        }
    }

    #[inline]
    fn get_lower_bits(&self, val: u32) -> u16 {
        (val & core::u16::MAX as u32) as u16
    }
}

impl<Q> Encoder for MA702GQ<Q>
where
    Q: Qei,
    Q::Count: Into<u32>,
    nb::Error<EncoderError>: From<<Q as Qei>::Error>,
{
    fn get_relative_distance(&mut self) -> nb::Result<Distance, EncoderError> {
        let after_count = self.get_lower_bits(self.qei.try_count()?.into());
        let relative_count = if after_count > self.before_count {
            (after_count - self.before_count) as i16
        } else {
            -((self.before_count - after_count) as i16)
        };
        self.before_count = after_count;
        Ok(
            Angle::from_rotation(relative_count as f32 / Self::RESOLUTION_PER_ROTATION)
                * self.wheel_radius,
        )
    }
}
