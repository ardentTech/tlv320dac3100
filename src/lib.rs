#![no_std]

pub(crate) mod bits;
pub mod driver;
pub mod error;
pub mod registers;
pub mod typedefs;

pub use driver::TLV320DAC3100;