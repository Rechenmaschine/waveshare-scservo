pub struct MockInterface<I> {
    pub inner: I,
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
        todo!("")
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
