use core::f32::consts::PI;

use components::traits::Math as TMath;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Math;

impl TMath for Math {
    fn sqrtf(val: f32) -> f32 {
        libm::sqrtf(val)
    }

    fn sinf(val: f32) -> f32 {
        let val = Self::rem_euclidf(val, PI);
        unsafe { crate::bindings::arm_sin_f32(val) }
    }

    fn cosf(val: f32) -> f32 {
        let val = Self::rem_euclidf(val, PI);
        unsafe { crate::bindings::arm_cos_f32(val) }
    }

    fn expf(val: f32) -> f32 {
        libm::expf(val)
    }

    fn atan2f(x: f32, y: f32) -> f32 {
        libm::atan2f(x, y)
    }

    fn rem_euclidf(lhs: f32, rhs: f32) -> f32 {
        libm::fmodf(lhs, rhs)
    }
}
