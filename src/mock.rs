use embedded_io::{ErrorType, Read, Write};

pub struct MockInterface<I> {
    pub inner: I,
}

impl<I> ErrorType for MockInterface<I> {
    type Error = embedded_io::ErrorKind;
}

impl<I> Read for MockInterface<I> {
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

impl<I> Write for MockInterface<I> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<I> device_driver::RegisterInterface for MockInterface<I> {
    type Error = ();
    type AddressType = u8;

    fn write_register(
        &mut self,
        _address: Self::AddressType,
        _size_bits: u32,
        _data: &[u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }

    fn read_register(
        &mut self,
        _address: Self::AddressType,
        _size_bits: u32,
        _data: &mut [u8],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}
