//! Cyclic Redundancy Check (CRC) peripheral driver

use crate::pac::CRC_ENGINE;
use crate::syscon::ClockControl;
use crate::syscon::Syscon;
use core::hash::Hasher;

/// CRC polynomial enum.
/// this CRC engine only supports
/// CRC-CCITT, CRC-16, and CRC-32
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcPolynomial {
    /// CRC-CCITT: x16 + x12 + x5 + 1   
    CRC_CCITT,
    /// CRC-16: x16 + x15 + x2 + 1
    CRC_16,
    /// CRC-32: x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x + 1
    CRC_32,
}

/// CRC data bit order enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcDataBitOrder {
    /// Bit Order Reversed
    BitOrderReversed,
    /// Bit Order Normal
    BitOrderNormal,
}

/// CRC data 1's complement enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcDataComplement {
    /// Data 1's complement
    DataComplementEnabled,
    /// Data 1's complement disabled
    DataComplementDisabled,
}

/// CRC sum bit order enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcSumOrder {
    /// Sum Bit Order Reversed
    SumOrderReversed,
    /// Sum Bit Order Normal
    SumOrderNormal,
}

/// CRC sum 1's complement enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcSumComplement {
    /// Sum 1's complement
    SumComplementEnabled,
    /// Sum 1's complement disabled
    SumComplementDisabled,
}

/// CRC config struct
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// CRC polynomial
    polynomial: CrcPolynomial,
    /// CRC seed value
    seed: u32,
    /// CRC data bit order
    data_bit_order: CrcDataBitOrder,
    /// CRC data 1's complement
    data_complement: CrcDataComplement,
    /// CRC sum bit order
    sum_order: CrcSumOrder,
    /// CRC sum 1's complement
    sum_complement: CrcSumComplement,
}

/// Invalig CRC config error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidConfig;

/// Config implementation for modifiers
impl Config {
    /// Creates a new Config with the given parameters
    pub fn new(
        polynomial: CrcPolynomial,
        seed: u32,
        data_bit_order: CrcDataBitOrder,
        data_complement: CrcDataComplement,
        sum_order: CrcSumOrder,
        sum_complement: CrcSumComplement,
    ) -> Self {
        Self {
            polynomial,
            seed,
            data_bit_order,
            data_complement,
            sum_order,
            sum_complement,
        }
    }
    /// modify the polynomial of self
    pub fn polynomial(mut self, polynomial: CrcPolynomial) -> Self {
        self.polynomial = polynomial;
        self
    }
    /// modify the seed of self
    pub fn seed(mut self, seed: u32) -> Self {
        self.seed = seed;
        self
    }
    /// modify the data bit order of self
    pub fn data_bit_order(mut self, data_bit_order: CrcDataBitOrder) -> Self {
        self.data_bit_order = data_bit_order;
        self
    }

    /// modify the data complement of self
    pub fn data_complement(mut self, data_complement: CrcDataComplement) -> Self {
        self.data_complement = data_complement;
        self
    }

    /// modify the sum bit order of self
    pub fn sum_order(mut self, sum_order: CrcSumOrder) -> Self {
        self.sum_order = sum_order;
        self
    }

    /// modify the sum complement of self
    pub fn sum_complement(mut self, sum_complement: CrcSumComplement) -> Self {
        self.sum_complement = sum_complement;
        self
    }

    /// CCITT configuration
    pub fn crc_ccitt() -> Self {
        Self {
            polynomial: CrcPolynomial::CRC_CCITT,
            seed: 0x0000_FFFF,
            data_bit_order: CrcDataBitOrder::BitOrderNormal,
            data_complement: CrcDataComplement::DataComplementDisabled,
            sum_order: CrcSumOrder::SumOrderNormal,
            sum_complement: CrcSumComplement::SumComplementDisabled,
        }
    }

    /// CRC-16 configuration
    pub fn crc_16() -> Self {
        Self {
            polynomial: CrcPolynomial::CRC_16,
            seed: 0x0000_0000,
            data_bit_order: CrcDataBitOrder::BitOrderReversed,
            data_complement: CrcDataComplement::DataComplementDisabled,
            sum_order: CrcSumOrder::SumOrderReversed,
            sum_complement: CrcSumComplement::SumComplementDisabled,
        }
    }

    /// CRC-32 configuration
    pub fn crc_32() -> Self {
        Self {
            polynomial: CrcPolynomial::CRC_32,
            seed: 0xFFFF_FFFF,
            data_bit_order: CrcDataBitOrder::BitOrderReversed,
            data_complement: CrcDataComplement::DataComplementDisabled,
            sum_order: CrcSumOrder::SumOrderReversed,
            sum_complement: CrcSumComplement::SumComplementEnabled,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            polynomial: CrcPolynomial::CRC_CCITT,
            seed: 0x0000_FFFF,
            data_bit_order: CrcDataBitOrder::BitOrderNormal,
            data_complement: CrcDataComplement::DataComplementDisabled,
            sum_order: CrcSumOrder::SumOrderNormal,
            sum_complement: CrcSumComplement::SumComplementDisabled,
        }
    }
}
/// Extension trait to constrain the CRC peripheral.
pub trait CrcExt {
    /// Constrains the CRC peripheral to play nicely with the other abstractions
    fn crc(self, config: Config, syscon: &mut Syscon) -> Result<Crc, InvalidConfig>;
}

/// Crc driver struct
pub struct Crc {
    crc: CRC_ENGINE,
    config: Config,
}

impl CrcExt for CRC_ENGINE {
    /// Initializes the CRC peripheral
    fn crc(self, config: Config, syscon: &mut Syscon) -> Result<Crc, InvalidConfig> {
        // enable the clock in SYSCON
        self.enable_clock(syscon);

        let mut obj = Crc { crc: self, config };

        obj.configure(config);
        Ok(obj)
    }
}

impl Crc {
    /// Configures the CRC peripheral
    pub fn configure(&mut self, config: Config) {
        self.config = config;
        // fill MODE register
        self.crc.mode.write(|w| unsafe {
            w.crc_poly()
                .bits(match config.polynomial {
                    CrcPolynomial::CRC_CCITT => 0b00,
                    CrcPolynomial::CRC_16 => 0b01,
                    CrcPolynomial::CRC_32 => 0b10,
                })
                .bit_rvs_data()
                .bit(config.data_bit_order == CrcDataBitOrder::BitOrderReversed)
                .cmpl_data()
                .bit(config.data_complement == CrcDataComplement::DataComplementEnabled)
                .bit_rvs_sum()
                .bit(config.sum_order == CrcSumOrder::SumOrderReversed)
                .cmpl_sum()
                .bit(config.sum_complement == CrcSumComplement::SumComplementEnabled)
        });
        // set seed register
        self.crc
            .seed
            .write(|w| unsafe { w.crc_seed().bits(config.seed) });
    }

    /// Reset the CRC state machine
    pub fn reset(&mut self) {
        self.configure(self.config);
    }

    /// feed the crc with data
    pub fn feed(&mut self, data: &[u8]) {
        unsafe {
            let (prefix, middle, suffix) = data.align_to::<u32>();

            for &byte in prefix {
                self.crc.data8().write(|w| w.data8().bits(byte));
            }

            for &word in middle {
                self.crc.data32().write(|w| w.data32().bits(word));
            }

            for &byte in suffix {
                self.crc.data8().write(|w| w.data8().bits(byte));
            }
        }
    }

    /// Get the result of the CRC, depending on the polynomial chosen only a certain amount of the
    /// bits are the result. This will reset the CRC peripheral after use.
    #[inline]
    pub fn result(&mut self) -> u32 {
        let ret = self.peek_result();

        self.reset();

        ret
    }

    /// Get a peed at the result of the CRC, depending on the polynomial chosen only a certain
    /// amount of the bits are the result.
    #[inline]
    pub fn peek_result(&self) -> u32 {
        self.crc.sum().read().crc_sum().bits()
    }

    /// release the peripheral
    pub fn release(self) -> CRC_ENGINE {
        self.crc
    }
}

impl Hasher for Crc {
    #[inline]
    fn finish(&self) -> u64 {
        // `peek_result` as `core::hash::Hasher` required that the `finish` method does not reset
        // the hasher.
        self.peek_result() as u64
    }

    #[inline]
    fn write(&mut self, data: &[u8]) {
        self.feed(data);
    }
}
