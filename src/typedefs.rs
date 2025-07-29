#[derive(Debug, PartialEq)]
pub enum CdivClkin {
    Mclk = 0x0,
    Bclk = 0x1,
    Din = 0x2,
    PllClk = 0x3,
    DacClk = 0x4,
    DacModClk = 0x5,
}
impl TryFrom<u8> for CdivClkin {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(CdivClkin::Mclk),
            0x1 => Ok(CdivClkin::Bclk),
            0x2 => Ok(CdivClkin::Din),
            0x3 => Ok(CdivClkin::PllClk),
            0x4 => Ok(CdivClkin::DacClk),
            0x5 => Ok(CdivClkin::DacModClk),
            _ => Err(())
        }
    }
}

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
pub enum CodecInterface {
    I2S = 0x0,
    DSP = 0x1,
    RJF = 0x2,
    LJF = 0x3,
}
impl TryFrom<u8> for CodecInterface {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(CodecInterface::I2S),
            0x1 => Ok(CodecInterface::DSP),
            0x2 => Ok(CodecInterface::RJF),
            0x3 => Ok(CodecInterface::LJF),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum CodecInterfaceWordLength {
    Word16Bits = 0x0,
    Word20Bits = 0x1,
    Word24Bits = 0x2,
    Word32Bits = 0x3,
}
impl TryFrom<u8> for CodecInterfaceWordLength {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(CodecInterfaceWordLength::Word16Bits),
            0x1 => Ok(CodecInterfaceWordLength::Word20Bits),
            0x2 => Ok(CodecInterfaceWordLength::Word24Bits),
            0x3 => Ok(CodecInterfaceWordLength::Word32Bits),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HeadphoneOutputVoltage {
    Common1_35V = 0x00,
    Common1_5V = 0x01,
    Common1_65V = 0x02,
    Common1_8V = 0x03,
}
impl TryFrom<u8> for HeadphoneOutputVoltage {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(HeadphoneOutputVoltage::Common1_35V),
            0x1 => Ok(HeadphoneOutputVoltage::Common1_5V),
            0x2 => Ok(HeadphoneOutputVoltage::Common1_65V),
            0x3 => Ok(HeadphoneOutputVoltage::Common1_8V),
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
