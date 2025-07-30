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
pub enum DacLeftOutputMixerRouting {
    None = 0x0,
    LeftChannelMixerAmplifier = 0x1,
    HplDriver = 0x2
}

#[derive(Debug, PartialEq)]
pub enum DacRightOutputMixerRouting {
    None = 0x0,
    RightChannelMixerAmplifier = 0x1,
    HprDriver = 0x2
}

#[derive(Debug, PartialEq)]
pub enum HpOutputVoltage {
    Common1_35V = 0x00,
    Common1_5V = 0x01,
    Common1_65V = 0x02,
    Common1_8V = 0x03,
}
impl TryFrom<u8> for HpOutputVoltage {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(HpOutputVoltage::Common1_35V),
            0x1 => Ok(HpOutputVoltage::Common1_5V),
            0x2 => Ok(HpOutputVoltage::Common1_65V),
            0x3 => Ok(HpOutputVoltage::Common1_8V),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HpPowerOn {
    Time0us = 0x0,
    Time15_3us = 0x1,
    Time153us = 0x2,
    Time1_53ms = 0x3,
    Time15_3ms = 0x4,
    Time76_2ms = 0x5,
    Time153ms = 0x6,
    Time304ms = 0x7,
    Time610ms = 0x8,
    Time1_22s = 0x9,
    Time3_04s = 0xa,
    Time6_1s = 0xb,
}

#[derive(Debug, PartialEq)]
pub enum HpRampUp {
    Time0ms = 0x0,
    Time0_98ms = 0x1,
    Time1_95ms = 0x2,
    Time3_9ms = 0x3,
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

#[derive(Debug, PartialEq)]
pub enum VolumeControlHysteresis {
    HysteresisNone = 0x0,
    Hysteresis1Bit = 0x1,
    Hysteresis2Bits = 0x2,
}
impl TryFrom<u8> for VolumeControlHysteresis {
    type Error = ();
    // TODO unit test
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(VolumeControlHysteresis::HysteresisNone),
            0x1 => Ok(VolumeControlHysteresis::Hysteresis1Bit),
            0x2 => Ok(VolumeControlHysteresis::Hysteresis2Bits),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum VolumeControlThroughput {
    Rate15_625Hz = 0x0, // 10.68 Hz (RC)
    Rate31_25Hz = 0x1,  // 21.35 Hz (RC)
    Rate62_5Hz = 0x2,   // 42.71 Hz (RC)
    Rate125Hz = 0x3,    // 85.20 Hz (RC)
    Rate250Hz = 0x4,    // 170.0 Hz (RC)
    Rate500Hz = 0x5,    // 340.0 Hz (RC)
    Rate1kHz = 0x6,     // 680.0 Hz (RC)
    Rate2kHz = 0x7,     // 1.37 kHz (RC)
}
impl TryFrom<u8> for VolumeControlThroughput {
    type Error = ();
    // TODO unit test
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(VolumeControlThroughput::Rate15_625Hz),
            0x1 => Ok(VolumeControlThroughput::Rate31_25Hz),
            0x2 => Ok(VolumeControlThroughput::Rate62_5Hz),
            0x3 => Ok(VolumeControlThroughput::Rate125Hz),
            0x4 => Ok(VolumeControlThroughput::Rate250Hz),
            0x5 => Ok(VolumeControlThroughput::Rate500Hz),
            0x6 => Ok(VolumeControlThroughput::Rate1kHz),
            0x7 => Ok(VolumeControlThroughput::Rate2kHz),
            _ => Err(())
        }
    }
}