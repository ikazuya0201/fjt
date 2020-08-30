use alloc::rc::Rc;
use core::cell::RefCell;

use components::{
    data_types::{State, Target},
    traits::Logger,
};
use heapless::{ArrayLength, Vec};
use uom::si::length::meter;

pub type LogData = (f32, f32, f32, f32);

pub type Log<N> = Vec<LogData, N>;

pub struct ILogger<N>
where
    N: ArrayLength<LogData>,
{
    vec: Rc<RefCell<Log<N>>>,
}

impl<N> ILogger<N>
where
    N: ArrayLength<LogData>,
{
    pub fn new(vec: Rc<RefCell<Log<N>>>) -> Self {
        Self { vec }
    }
}

impl<N> Logger for ILogger<N>
where
    N: ArrayLength<LogData>,
{
    fn log(&self, state: &State, target: &Target) {
        self.vec
            .borrow_mut()
            .push((
                target.x.x.get::<meter>(),
                target.y.x.get::<meter>(),
                state.x.x.get::<meter>(),
                state.y.x.get::<meter>(),
            ))
            .ok(); //ignore error
    }
}
