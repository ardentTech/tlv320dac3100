pub(crate) fn get_bits(byte: u8, bits: u8, lsb_offset: u8) -> u8 {
    (byte >> lsb_offset) & ((1 << (bits)) - 1)
}

pub(crate) fn set_bits(byte: &mut u8, data: u8, lsb_offset: u8, mask: u8) {
    *byte &= !mask;
    *byte |= (data << lsb_offset) & mask
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn get_bits_8_bits_0_offset() {
        let byte = 0b1011_1101;
        let bits = get_bits(byte, 8, 0);
        assert_eq!(bits, 0b1011_1101);
    }

    #[test]
    fn get_bits_0_bits_0_offset() {
        let byte = 0b1011_1101;
        let bits = get_bits(byte, 0, 0);
        assert_eq!(bits, 0b0);
    }

    #[test]
    #[should_panic]
    fn get_bits_1_bit_7_offset_overflow() {
        let byte = 0b1011_1101;
        let bits = get_bits(byte, 1, 7);
        assert_eq!(bits, 0b011_1101);
    }

    #[test]
    #[should_panic]
    fn get_bits_1_bit_8_offset_overflow() {
        let byte = 0b1011_1101;
        let bits = get_bits(byte, 1, 8);
        assert_eq!(bits, 0b0);
    }

    #[test]
    fn update_bits() {
        let mut byte = 0b1010_0001;
        set_bits(&mut byte, 0b11, 2, 0b0000_1100);
        assert_eq!(0b1010_1101, byte);
        set_bits(&mut byte, 0b10, 5, 0b0110_0000);
        assert_eq!(0b1100_1101, byte);
    }
}