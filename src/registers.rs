pub const PAGE_CONTROL: u8 = 0x00;
// page 0
pub const SOFTWARE_RESET: u8 = 0x01;
pub const OT_FLAG: u8 = 0x03;
pub const CLOCK_GEN_MUXING: u8 = 0x04;
pub const PLL_P_AND_R_VALUES: u8 = 0x05;
pub const PLL_J_VALUE: u8 = 0x06;
pub const PLL_D_VALUE_MSB: u8 = 0x07;
pub const PLL_D_VALUE_LSB: u8 = 0x08;
pub const DAC_NDAC_VAL: u8 = 0x0b;
pub const DAC_MDAC_VAL: u8 = 0x0c;
pub const DAC_DOSR_VAL_MSB: u8 = 0x0d;
pub const DAC_DOSR_VAL_LSB: u8 = 0x0e;
pub const CLKOUT_MUX: u8 = 0x19;
pub const CLKOUT_M_VAL: u8 = 0x1a;
pub const CODEC_INTERFACE_CONTROL_1: u8 = 0x1b;
pub const DAC_INTERRUPT_FLAGS_STICKY_BITS: u8 = 0x2c;
pub const INT1_CONTROL_REGISTER: u8 = 0x30;
pub const GPIO1_IN_OUT_PIN_CONTROL: u8 = 0x33;
pub const DAC_PROCESSING_BLOCK_SECTION: u8 = 0x3c;

pub const DAC_DATA_PATH_SETUP: u8 = 0x3f;
pub const DAC_VOLUME_CONTROL: u8 = 0x40;
pub const DAC_LEFT_VOLUME_CONTROL: u8 = 0x41;
pub const DAC_RIGHT_VOLUME_CONTROL: u8 = 0x42;
pub const HEADSET_DETECTION: u8 = 0x43;
pub const VOL_MICDET_PIN_SAR_ADC: u8 = 0x74;

// page 1
pub const HEADPHONE_DRIVERS: u8 = 0x1f;
pub const HP_OUTPUT_DRIVERS_POP_REMOVAL_SETTINGS: u8 = 0x21;
pub const DAC_L_AND_DAC_R_OUTPUT_MIXER_ROUTING: u8 = 0x23;
pub const LEFT_ANALOG_VOLUME_TO_HPL: u8 = 0x24;
pub const RIGHT_ANALOG_VOLUME_TO_HPR: u8 = 0x25;
pub const LEFT_ANALOG_VOLUME_TO_SPK: u8 = 0x26;
pub const HPL_DRIVER: u8 = 0x28;
pub const HPR_DRIVER: u8 = 0x29;
pub const CLASS_D_SPK_DRIVER: u8 = 0x2a;
pub const MICBIAS: u8 = 0x2e;