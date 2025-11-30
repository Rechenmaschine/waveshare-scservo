use crate::registers::{SclInternal};

mod mock;
mod registers;

pub struct SCLDevice<I> {
    inner: SclInternal<I>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockInterface;

    #[test]
    fn test_device_creation() {
        let mut device = SclInternal::new(MockInterface { inner: () });

        let x = device.servo_major_version().read().unwrap().version_number();


    }
}
