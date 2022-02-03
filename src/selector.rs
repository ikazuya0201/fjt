use mousecore::administrator::Selector as ISelector;
use mousecore::defaults::operator_store::Mode;

pub struct Selector;

impl ISelector<Mode> for Selector {
    fn reset(&self) {}

    fn mode(&self) -> Mode {
        Mode::Idle
    }

    fn is_enabled(&self) -> bool {
        false
    }
}
