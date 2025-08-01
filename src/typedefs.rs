// // TODO unit test all try_froms

#[derive(Debug, PartialEq)]
pub enum BdivClkin {
    DacClk = 0x0,
    DacModClk = 0x1,
}

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
pub enum DinControl {
    Disabled = 0x0,
    Enabled = 0x1,
    Gpi = 0x2,
}

#[derive(Debug, PartialEq)]
pub enum Gpio1Mode {
    Disabled = 0x0,
    Input = 0x1,
    Gpi = 0x2,
    Gpo = 0x3,
    ClkOut = 0x4,
    Int1 = 0x5,
    Int2 = 0x6,
    BclkOut = 0x8,
    WclkOut = 0x9,
}

#[derive(Debug, PartialEq)]
pub enum HeadsetButtonPressDebounce {
    Debounce0ms = 0x0,
    Debounce8ms = 0x1, // sampled with 8ms clock
    Debounce16ms = 0x2, // sampled with 16ms clock
    Debounce32ms = 0x3, // sampled with 32ms clock
}
impl TryFrom<u8> for HeadsetButtonPressDebounce {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(HeadsetButtonPressDebounce::Debounce0ms),
            0x1 => Ok(HeadsetButtonPressDebounce::Debounce8ms),
            0x2 => Ok(HeadsetButtonPressDebounce::Debounce16ms),
            0x3 => Ok(HeadsetButtonPressDebounce::Debounce32ms),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HeadsetDetected {
    None = 0x0,
    WithoutMic = 0x1,
    WithMic = 0x3,
}
impl TryFrom<u8> for HeadsetDetected {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(HeadsetDetected::None),
            0x1 => Ok(HeadsetDetected::WithoutMic),
            0x3 => Ok(HeadsetDetected::WithMic),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HeadsetDetectionDebounce {
    Debounce16ms = 0x0, // sampled with 2ms clock
    Debounce32ms = 0x1, // sampled with 4ms clock
    Debounce64ms = 0x2, // sampled with 8ms clock
    Debounce128ms = 0x3, // sampled with 16ms clock
    Debounce256ms = 0x4, // sampled with 32ms clock
    Debounce512ms = 0x5, // sampled with 64ms clock
}
impl TryFrom<u8> for HeadsetDetectionDebounce {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(HeadsetDetectionDebounce::Debounce16ms),
            0x1 => Ok(HeadsetDetectionDebounce::Debounce32ms),
            0x2 => Ok(HeadsetDetectionDebounce::Debounce64ms),
            0x3 => Ok(HeadsetDetectionDebounce::Debounce128ms),
            0x4 => Ok(HeadsetDetectionDebounce::Debounce256ms),
            0x5 => Ok(HeadsetDetectionDebounce::Debounce512ms),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum HpMode {
    Default = 0x0,
    CurrentBoost = 0x1,
    CurrentBoostX2 = 0x3,
}

#[derive(Debug, PartialEq)]
pub enum HpScdDebounce {
    Time0us = 0x0,
    Time8us = 0x1,
    Time16us = 0x2,
    Time32us = 0x3,
    Time64us = 0x4,
    Time128us = 0x5,
    Time256us = 0x6,
    Time512us = 0x7,
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
pub enum MicBiasOutput {
    PoweredDown = 0x0,
    Powered2V = 0x1,
    Powered2_5V = 0x2,
    PoweredAVDD = 0x3,
}

#[derive(Debug, PartialEq)]
pub enum OutputStage {
    Gain6dB = 0x0,
    Gain12dB = 0x1,
    Gain18dB = 0x2,
    Gain24dB = 0x3,
}
impl TryFrom<u8> for OutputStage {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(OutputStage::Gain6dB),
            0x1 => Ok(OutputStage::Gain12dB),
            0x2 => Ok(OutputStage::Gain18dB),
            0x3 => Ok(OutputStage::Gain24dB),
            _ => Err(())
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum PgaRampDown {
    Time0ms = 0x0,
    Time3_04ms = 0x1,
    Time7_62ms = 0x2,
    Time12_2ms = 0x3,
    Time15_3ms = 0x4,
    Time19_8ms = 0x5,
    Time24_4ms = 0x6,
    Time30_5ms = 0x7,
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
pub enum PrimaryBclkOutput {
    InternalBclk = 0x0,
    SecondaryBclk = 0x1,
}

#[derive(Debug, PartialEq)]
pub enum RightBeepMode {
    IndependentControl = 0x0,
    LeftToRight = 0x1,
    RightToLeft = 0x2,
}

#[derive(Debug, PartialEq)]
pub enum SecondaryBclkOutput {
    PrimaryBclk = 0x0,
    InternalBclk = 0x1,
}

#[derive(Debug, PartialEq)]
pub enum PrimaryWclkOutput {
    InternalDacFs = 0x0,
    SecondaryWclk = 0x2,
}

#[derive(Debug, PartialEq)]
pub enum SecondaryWclkOutput {
    PrimaryWclk = 0x0,
    InternalDacFs = 0x1,
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
pub enum VolumeControl {
    IndependentChannels = 0x0,
    LeftToRight = 0x1,
    RightToLeft = 0x2,
}

#[derive(Debug, PartialEq)]
pub enum VolumeControlHysteresis {
    HysteresisNone = 0x0,
    Hysteresis1Bit = 0x1,
    Hysteresis2Bits = 0x2,
}
impl TryFrom<u8> for VolumeControlHysteresis {
    type Error = ();
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