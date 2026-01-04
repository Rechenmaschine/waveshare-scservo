use embedded_io::ErrorType;
use embedded_io::{Read as BlockingRead, Write as BlockingWrite};
use embedded_io_async::{Read as AsyncRead, Write as AsyncWrite};

pub struct MockInterface<I> {
    #[allow(dead_code)]
    pub inner: I,
}

impl<I> ErrorType for MockInterface<I> {
    type Error = embedded_io::ErrorKind;
}

impl<I> BlockingRead for MockInterface<I> {
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

impl<I> BlockingWrite for MockInterface<I> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<I> AsyncRead for MockInterface<I> {
    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

impl<I> AsyncWrite for MockInterface<I> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
