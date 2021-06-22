//! Configuration structs for sending and receiving
//!
//! This module houses the datastructures that control how frames are transmitted and received.
//! The configs are passed to the send and receive functions.

use crate::Error;
use embedded_hal::{blocking::spi, digital::v2::OutputPin};

/// Transmit configuration
pub struct TxConfig {
    /// Sets the bitrate of the transmission.
    pub bitrate: BitRate,
    /// Sets the ranging bit in the transmitted frame.
    /// This has no effect on the capabilities of the DW1000.
    pub ranging_enable: bool,
    /// Sets the PRF value of the transmission.
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    /// The length of the preamble.
    pub preamble_length: PreambleLength,
    /// The channel that the DW1000 will transmit at.
    pub channel: UwbChannel,
    /// The SFD sequence that is used to transmit a frame.
    pub sfd_sequence: SfdSequence,
}

impl Default for TxConfig {
    fn default() -> Self {
        TxConfig {
            bitrate: Default::default(),
            ranging_enable: false,
            pulse_repetition_frequency: Default::default(),
            preamble_length: Default::default(),
            channel: Default::default(),
            sfd_sequence: Default::default(),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Receive configuration
pub struct RxConfig {
    /// The bitrate that will be used for reception.
    pub bitrate: BitRate,
    /// Enable frame filtering
    ///
    /// If true, only frames directly addressed to this node and broadcasts will
    /// be received.
    ///
    /// Defaults to `true`.
    pub frame_filtering: bool,
    /// Sets the PRF value of the reception
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    /// The expected preamble length.
    ///
    /// This affects the chosen PAC size.
    /// This should be the same as the preamble length that is used to send the messages.
    /// It is not a filter, though, so other preamble lengths may still be received.
    pub expected_preamble_length: PreambleLength,
    /// The channel that the DW1000 will listen at.
    pub channel: UwbChannel,
    /// The type of SFD sequence that will be scanned for.
    pub sfd_sequence: SfdSequence,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            bitrate: Default::default(),
            frame_filtering: true,
            pulse_repetition_frequency: Default::default(),
            expected_preamble_length: Default::default(),
            channel: Default::default(),
            sfd_sequence: Default::default(),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// The bitrate at which a message is transmitted
pub enum BitRate {
    /// 110 kilobits per second.
    /// This is an unofficial extension from decawave.
    Kbps110 = 0b00,
    /// 850 kilobits per second.
    Kbps850 = 0b01,
    /// 6.8 megabits per second.
    Kbps6800 = 0b10,
}

impl Default for BitRate {
    fn default() -> Self {
        BitRate::Kbps6800
    }
}

impl BitRate {
    /// Gets the recommended drx_tune0b value for the bitrate and sfd.
    pub fn get_recommended_drx_tune0b(&self, sfd_sequence: SfdSequence) -> u16 {
        // Values are taken from Table 30 of the DW1000 User Manual.
        match (self, sfd_sequence) {
            (BitRate::Kbps110, SfdSequence::IEEE) => 0x000A,
            (BitRate::Kbps110, _) => 0x0016,
            (BitRate::Kbps850, SfdSequence::IEEE) => 0x0001,
            (BitRate::Kbps850, _) => 0x0006,
            (BitRate::Kbps6800, SfdSequence::IEEE) => 0x0001,
            (BitRate::Kbps6800, _) => 0x0002,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// The PRF value
pub enum PulseRepetitionFrequency {
    /// 16 megahertz
    Mhz16 = 0b01,
    /// 64 megahertz
    Mhz64 = 0b10,
}

impl Default for PulseRepetitionFrequency {
    fn default() -> Self {
        PulseRepetitionFrequency::Mhz16
    }
}

impl PulseRepetitionFrequency {
    /// Gets the recommended value for the drx_tune1a register based on the PRF
    pub fn get_recommended_drx_tune1a(&self) -> u16 {
        // Values taken from Table 31 of the DW1000 User Manual.
        match self {
            PulseRepetitionFrequency::Mhz16 => 0x0087,
            PulseRepetitionFrequency::Mhz64 => 0x008D,
        }
    }

    /// Gets the recommended value for the drx_tune2 register based on the PRF and PAC size
    pub fn get_recommended_drx_tune2<SPI, CS>(&self, pac_size: u8) -> Result<u32, Error<SPI, CS>>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS: OutputPin,
    {
        // Values taken from Table 33 of the DW1000 User Manual.
        match (self, pac_size) {
            (PulseRepetitionFrequency::Mhz16, 8) => Ok(0x311A002D),
            (PulseRepetitionFrequency::Mhz64, 8) => Ok(0x313B006B),
            (PulseRepetitionFrequency::Mhz16, 16) => Ok(0x331A0052),
            (PulseRepetitionFrequency::Mhz64, 16) => Ok(0x333B00BE),
            (PulseRepetitionFrequency::Mhz16, 32) => Ok(0x351A009A),
            (PulseRepetitionFrequency::Mhz64, 32) => Ok(0x353B015E),
            (PulseRepetitionFrequency::Mhz16, 64) => Ok(0x371A011D),
            (PulseRepetitionFrequency::Mhz64, 64) => Ok(0x373B0296),
            // The PAC size is something we didn't expect
            _ => Err(Error::InvalidConfiguration),
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// An enum that specifies the length of the preamble.
///
/// Longer preambles improve the reception quality and thus range.
/// This comes at the cost of longer transmission times and thus power consumption and bandwidth use.
///
/// For the bit pattern, see table 16 in the user manual. Two bits TXPSR,then two bits PE.
pub enum PreambleLength {
    /// 64 symbols of preamble.
    /// Only supported at Bitrate::Kbps6800.
    Symbols64 = 0b0100,
    /// 128 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    Symbols128 = 0b0101,
    /// 256 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    Symbols256 = 0b0110,
    /// 512 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    Symbols512 = 0b0111,
    /// 1024 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    Symbols1024 = 0b1000,
    /// 1536 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    /// Unofficial extension from decawave.
    Symbols1536 = 0b1001,
    /// 2048 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    /// Unofficial extension from decawave.
    Symbols2048 = 0b1010,
    /// 4096 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    Symbols4096 = 0b1100,
}

impl Default for PreambleLength {
    fn default() -> Self {
        PreambleLength::Symbols128
    }
}

impl PreambleLength {
    /// Gets the recommended PAC size based on the preamble length.
    pub fn get_recommended_pac_size(&self) -> u8 {
        // Values are taken from Table 6 of the DW1000 User manual
        match self {
            PreambleLength::Symbols64 => 8,
            PreambleLength::Symbols128 => 8,
            PreambleLength::Symbols256 => 16,
            PreambleLength::Symbols512 => 16,
            PreambleLength::Symbols1024 => 32,
            PreambleLength::Symbols1536 => 64,
            PreambleLength::Symbols2048 => 64,
            PreambleLength::Symbols4096 => 64,
        }
    }

    /// Gets the recommended drx_tune1b register value based on the preamble length and the bitrate.
    pub fn get_recommended_drx_tune1b<SPI, CS>(
        &self,
        bitrate: BitRate,
    ) -> Result<u16, Error<SPI, CS>>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS: OutputPin,
    {
        // Values are taken from Table 32 of the DW1000 User manual
        match (self, bitrate) {
            (PreambleLength::Symbols64, BitRate::Kbps6800) => Ok(0x0010),
            (PreambleLength::Symbols128, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols256, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols512, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols1024, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols128, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols256, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols512, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols1024, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols1536, BitRate::Kbps110) => Ok(0x0064),
            (PreambleLength::Symbols2048, BitRate::Kbps110) => Ok(0x0064),
            (PreambleLength::Symbols4096, BitRate::Kbps110) => Ok(0x0064),
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Gets the recommended dxr_tune4h register value based on the preamble length.
    pub fn get_recommended_dxr_tune4h(&self) -> u16 {
        // Values are taken from Table 34 of the DW1000 User manual
        match self {
            PreambleLength::Symbols64 => 0x0010,
            _ => 0x0028,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// An enum that allows the selection between different SFD sequences
///
/// The difference between the two Decawave sequences is that there are two ways
/// to enable it in the chip. Decawave will only set the DWSFD bit and
/// DecawaveAlt set the DWSFD and the \[T,R\]NSSFD bits.
pub enum SfdSequence {
    /// The standard sequence defined by the IEEE standard.
    /// Most likely the best choice for 6.8 Mbps connections.
    IEEE,
    /// A sequence defined by Decawave that is supposed to be more robust.
    /// This is an unofficial addition.
    /// Most likely the best choice for 110 Kbps connections.
    Decawave,
    /// A sequence defined by Decawave that is supposed to be more robust.
    /// This is an unofficial addition.
    /// Most likely the best choice for 850 Kbps connections.
    DecawaveAlt,
    /// Uses the sequence that is programmed in by the user.
    /// This is an unofficial addition.
    User,
}

impl Default for SfdSequence {
    fn default() -> Self {
        SfdSequence::IEEE
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// All the available UWB channels.
///
/// Note that while a channel may have more bandwidth than ~900 Mhz, the DW1000 can only send up to ~900 Mhz
pub enum UwbChannel {
    /// Channel 1
    /// - Center frequency: 3494.4 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel1 = 1,
    /// Channel 2
    /// - Center frequency: 3993.6 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel2 = 2,
    /// Channel 3
    /// - Center frequency: 4492.8 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel3 = 3,
    /// Channel 4
    /// - Center frequency: 3993.6 Mhz
    /// - Bandwidth: 1331.2 Mhz
    Channel4 = 4,
    /// Channel 5
    /// - Center frequency: 6489.6 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel5 = 5,
    /// Channel 7
    /// - Center frequency: 6489.6 Mhz
    /// - Bandwidth: 1081.6 Mhz
    Channel7 = 7,
}

impl Default for UwbChannel {
    fn default() -> Self {
        UwbChannel::Channel5
    }
}

impl UwbChannel {
    /// Is this a narrow channel?
    pub fn is_narrow(&self) -> bool {
        match self {
            UwbChannel::Channel4 | UwbChannel::Channel7 => false,
            UwbChannel::Channel1 | UwbChannel::Channel2 | UwbChannel::Channel3 | UwbChannel::Channel5 => true,
        }
    }
    /// Gets the recommended preamble code
    pub fn get_recommended_preamble_code(&self, prf_value: PulseRepetitionFrequency) -> u8 {
        // Many have overlapping possibilities, so the numbers have been chosen so that there's no overlap here
        match (self, prf_value) {
            (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz16) => 1,
            (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz16) => 3,
            (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz16) => 5,
            (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz16) => 7,
            (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz16) => 4,
            (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz16) => 8,
            (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz64) => 9,
            (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz64) => 10,
            (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz64) => 11,
            (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz64) => 17,
            (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz64) => 12,
            (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz64) => 18,
        }
    }

    /// Gets the recommended value for the rf_txctrl register
    pub fn get_recommended_rf_txctrl(&self) -> u32 {
        // Values based on Table 38 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x00005C40,
            UwbChannel::Channel2 => 0x00045CA0,
            UwbChannel::Channel3 => 0x00086CC0,
            UwbChannel::Channel4 => 0x00045C80,
            UwbChannel::Channel5 => 0x001E3FE0,
            UwbChannel::Channel7 => 0x001E7DE0,
        }
    }

    /// Gets the recommended value for the tc_pgdelay register
    pub fn get_recommended_tc_pgdelay(&self) -> u8 {
        // Values based on Table 40 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0xC9,
            UwbChannel::Channel2 => 0xC2,
            UwbChannel::Channel3 => 0xC5,
            UwbChannel::Channel4 => 0x95,
            UwbChannel::Channel5 => 0xC0,
            UwbChannel::Channel7 => 0x93,
        }
    }

    /// Gets the recommended value for the fs_pllcfg register
    pub fn get_recommended_fs_pllcfg(&self) -> u32 {
        // Values based on Table 43 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x09000407,
            UwbChannel::Channel2 | UwbChannel::Channel4 => 0x08400508,
            UwbChannel::Channel3 => 0x08401009,
            UwbChannel::Channel5 | UwbChannel::Channel7 => 0x0800041D,
        }
    }

    /// Gets the recommended value for the fs_plltune register
    pub fn get_recommended_fs_plltune(&self) -> u8 {
        // Values based on Table 44 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x1E,
            UwbChannel::Channel2 | UwbChannel::Channel4 => 0x26,
            UwbChannel::Channel3 => 0x56,
            UwbChannel::Channel5 | UwbChannel::Channel7 => 0xBE,
        }
    }

    /// Gets the recommended value for the rf_rxctrlh register
    pub fn get_recommended_rf_rxctrlh(&self) -> u8 {
        // Values based on Table 37 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1
            | UwbChannel::Channel2
            | UwbChannel::Channel3
            | UwbChannel::Channel5 => 0xD8,
            UwbChannel::Channel4 | UwbChannel::Channel7 => 0xBC,
        }
    }
}
