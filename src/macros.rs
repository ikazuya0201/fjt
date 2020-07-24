#[macro_export]
macro_rules! wait_ok {
    ($e: expr) => {
        while $e.is_err() {}
    };
}
