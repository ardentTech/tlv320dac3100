#[derive(Debug, PartialEq)]
pub enum CodecClkin {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    PllClk = 0x3,
}
impl TryFrom<u8> for CodecClkin {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(CodecClkin::Mclk),
            0x1 => Ok(CodecClkin::Bclk),
            0x2 => Ok(CodecClkin::Gpio1),
            0x3 => Ok(CodecClkin::PllClk),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum LeftDataPath {
    Off = 0x0,
    Left = 0x1,
    Right = 0x2,
    Both = 0x3
}

#[derive(Debug, PartialEq)]
pub enum PllClkin {
    Mclk = 0x0,
    Bclk = 0x1,
    Gpio1 = 0x2,
    Din = 0x3
}
impl TryFrom<u8> for PllClkin {
    type Error = ();
    // TODO unit test
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(PllClkin::Mclk),
            0x1 => Ok(PllClkin::Bclk),
            0x2 => Ok(PllClkin::Gpio1),
            0x3 => Ok(PllClkin::Din),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum RightDataPath {
    Off = 0x0,
    Right = 0x1,
    Left = 0x2,
    Both = 0x3
}

#[derive(Debug, PartialEq)]
pub enum SoftStepping {
    OneStepPerPeriod = 0x0,
    OneStepPerTwoPeriods = 0x1,
    Disabled = 0x2
}
