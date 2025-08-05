#[derive(Debug, PartialEq)]
pub enum TLV320DAC3100Error<E> {
    I2C(E),
    InvalidArgument,
}