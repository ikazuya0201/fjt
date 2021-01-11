use alloc::rc::Rc;
use core::cell::RefCell;

use components::{
    data_types::{MoveTarget, State, Target},
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
    target: RefCell<MoveTarget>,
}

impl<N> ILogger<N>
where
    N: ArrayLength<LogData>,
{
    #[allow(unused)]
    pub fn new(vec: Rc<RefCell<Log<N>>>) -> Self {
        Self {
            vec,
            target: RefCell::new(Default::default()),
        }
    }
}

impl<N> Logger for ILogger<N>
where
    N: ArrayLength<LogData>,
{
    fn log(&self, state: &State, target: &Target) {
        let target = {
            match target {
                Target::Moving(target) => {
                    self.target.replace(target.clone());
                    *target
                }
                _ => self.target.borrow().clone(),
            }
        };
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
