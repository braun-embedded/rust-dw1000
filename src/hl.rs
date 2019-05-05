//! High-level interface to the DW1000
//!
//! The entry point to this API is the [DW1000] struct. Please refer to the
//! documentation there for more details.
//!
//! This module implements a high-level interface to the DW1000. This is the
//! recommended way to access the DW1000 using this crate, unless you need the
//! greater flexibility provided by the [register-level interface].
//!
//! [register-level interface]: ../ll/index.html


use core::{
    fmt,
    num::Wrapping,
};

use embedded_hal::{
    blocking::spi,
    digital::OutputPin,
};
use nb;
use ssmarshal;

use crate::{
    ll,
    mac,
    time::{
        Duration,
        Instant,
    },
};


/// Entry point to the DW1000 driver API
pub struct DW1000<SPI, CS, State> {
    ll:     ll::DW1000<SPI, CS>,
    seq:    Wrapping<u8>,
    _state: State,
}

impl<SPI, CS> DW1000<SPI, CS, Uninitialized>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS:  OutputPin,
{
    /// Create a new instance of `DW1000`
    ///
    /// Requires the SPI peripheral and the chip select pin that are connected
    /// to the DW1000.
    pub fn new(
        spi        : SPI,
        chip_select: CS,
    )
        -> Self
    {
        DW1000 {
            ll:     ll::DW1000::new(spi, chip_select),
            seq:    Wrapping(0),
            _state: Uninitialized,
        }
    }

    /// Initialize the DW1000
    ///
    /// The DW1000's default configuration is somewhat inconsistent, and the
    /// user manual (section 2.5.5) has a long list of default configuration
    /// values that should be changed to guarantee everything works correctly.
    /// This method does just that.
    ///
    /// Please note that this method assumes that you kept the default
    /// configuration. It is generally recommended not to change configuration
    /// before calling this method.
    pub fn init(mut self) -> Result<DW1000<SPI, CS, Ready>, Error<SPI>> {
        // Set AGC_TUNE1. See user manual, section 2.5.5.1.
        self.ll.agc_tune1().write(|w| w.value(0x8870))?;

        // Set AGC_TUNE2. See user manual, section 2.5.5.2.
        self.ll.agc_tune2().write(|w| w.value(0x2502A907))?;

        // Set DRX_TUNE2. See user manual, section 2.5.5.3.
        self.ll.drx_tune2().write(|w| w.value(0x311A002D))?;

        // Set NTM. See user manual, section 2.5.5.4. This improves performance
        // in line-of-sight conditions, but might not be the best choice if non-
        // line-of-sight performance is important.
        self.ll.lde_cfg1().modify(|_, w| w.ntm(0xD))?;

        // Set LDE_CFG2. See user manual, section 2.5.5.5.
        self.ll.lde_cfg2().write(|w| w.value(0x1607))?;

        // Set TX_POWER. See user manual, section 2.5.5.6.
        self.ll.tx_power().write(|w| w.value(0x0E082848))?;

        // Set RF_TXCTRL. See user manual, section 2.5.5.7.
        self.ll.rf_txctrl().modify(|_, w|
            w
                .txmtune(0b1111)
                .txmq(0b111)
        )?;

        // Set TC_PGDELAY. See user manual, section 2.5.5.8.
        self.ll.tc_pgdelay().write(|w| w.value(0xC0))?;

        // Set FS_PLLTUNE. See user manual, section 2.5.5.9.
        self.ll.fs_plltune().write(|w| w.value(0xBE))?;

        // Set LDELOAD. See user manual, section 2.5.5.10.
        self.ll.pmsc_ctrl0().modify(|_, w| w.sysclks(0b01))?;
        self.ll.otp_ctrl().modify(|_, w| w.ldeload(0b1))?;
        while self.ll.otp_ctrl().read()?.ldeload() == 0b1 {}
        self.ll.pmsc_ctrl0().modify(|_, w| w.sysclks(0b00))?;

        // Set LDOTUNE. See user manual, section 2.5.5.11.
        self.ll.otp_addr().write(|w| w.value(0x004))?;
        self.ll.otp_ctrl().modify(|_, w|
            w
                .otprden(0b1)
                .otpread(0b1)
        )?;
        while self.ll.otp_ctrl().read()?.otpread() == 0b1 {}
        let ldotune_low = self.ll.otp_rdat().read()?.value();
        if ldotune_low != 0 {
            self.ll.otp_addr().write(|w| w.value(0x005))?;
            self.ll.otp_ctrl().modify(|_, w|
                w
                    .otprden(0b1)
                    .otpread(0b1)
            )?;
            while self.ll.otp_ctrl().read()?.otpread() == 0b1 {}
            let ldotune_high = self.ll.otp_rdat().read()?.value();

            let ldotune = ldotune_low as u64 | (ldotune_high as u64) << 32;
            self.ll.ldotune().write(|w| w.value(ldotune))?;
        }

        Ok(DW1000 {
            ll:     self.ll,
            seq:    self.seq,
            _state: Ready,
        })
    }
}

impl<SPI, CS> DW1000<SPI, CS, Ready>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS:  OutputPin,
{
    /// Sets the RX and TX antenna delays
    pub fn set_antenna_delay(&mut self, rx_delay: u16, tx_delay: u16)
        -> Result<(), Error<SPI>>
    {
        self.ll
            .lde_rxantd()
            .write(|w| w.value(rx_delay))?;
        self.ll
            .tx_antd()
            .write(|w| w.value(tx_delay))?;

        Ok(())
    }

    /// Returns the TX antenna delay
    pub fn get_tx_antenna_delay(&mut self)
        -> Result<Duration, Error<SPI>>
    {
        let tx_antenna_delay = self.ll.tx_antd().read()?.value();

        // Since `tx_antenna_delay` is `u16`, the following will never panic.
        let tx_antenna_delay = Duration::new(tx_antenna_delay.into()).unwrap();

        Ok(tx_antenna_delay)
    }

    /// Sets the network id and address used for sending and receiving
    pub fn set_address(&mut self, pan_id: mac::PanId, addr: mac::ShortAddress)
        -> Result<(), Error<SPI>>
    {
        self.ll
            .panadr()
            .write(|w|
                w
                    .pan_id(pan_id.0)
                    .short_addr(addr.0)
            )?;

        Ok(())
    }

    /// Returns the network id and address used for sending and receiving
    pub fn get_address(&mut self)
        -> Result<mac::Address, Error<SPI>>
    {
        let panadr = self.ll.panadr().read()?;

        Ok(mac::Address::Short(
            mac::PanId(panadr.pan_id()),
            mac::ShortAddress(panadr.short_addr()),
        ))
    }

    /// Returns the current system time
    pub fn sys_time(&mut self) -> Result<Instant, Error<SPI>> {
        let sys_time = self.ll.sys_time().read()?.value();

        // Since hardware timestamps fit within 40 bits, the following should
        // never panic.
        Ok(Instant::new(sys_time).unwrap())
    }

    /// Send an IEEE 802.15.4 MAC frame
    ///
    /// The `data` argument is wrapped into an IEEE 802.15.4 MAC frame and sent
    /// to `destination`.
    ///
    /// This operation can be delayed to aid in distance measurement, by setting
    /// `delayed_time` to `Some(instant)`. If you want to send the frame as soon
    /// as possible, just pass `None` instead.
    ///
    /// This method starts the transmission and returns immediately thereafter.
    /// Use the returned [`TxFuture`], to wait for the transmission to finish
    /// and check its result.
    pub fn send(&mut self,
        data:         &[u8],
        destination:  mac::Address,
        delayed_time: Option<Instant>,
    )
        -> Result<TxFuture<SPI, CS>, Error<SPI>>
    {
        // Clear event counters
        self.ll.evc_ctrl().write(|w| w.evc_clr(0b1))?;
        while self.ll.evc_ctrl().read()?.evc_clr() == 0b1 {}

        // (Re-)Enable event counters
        self.ll.evc_ctrl().write(|w| w.evc_en(0b1))?;
        while self.ll.evc_ctrl().read()?.evc_en() == 0b1 {}

        // Sometimes, for unknown reasons, the DW1000 gets stuck in RX mode.
        // Starting the transmitter won't get it to enter TX mode, which means
        // all subsequent send operations will fail. Let's disable the
        // transceiver and force the chip into IDLE mode to make sure that
        // doesn't happen.
        self.force_idle()?;

        let seq = self.seq.0;
        self.seq += Wrapping(1);

        let frame = mac::Frame {
            header: mac::Header {
                frame_type:      mac::FrameType::Data,
                version:         mac::FrameVersion::Ieee802154_2006,
                security:        mac::Security::None,
                frame_pending:   false,
                ack_request:     false,
                pan_id_compress: false,
                destination:     destination,
                source:          self.get_address()?,
                seq:             seq,
            },
            content: mac::FrameContent::Data,
            payload: data,
            footer: [0; 2],
        };

        delayed_time.map(|time| {
            self.ll
                .dx_time()
                .write(|w|
                    w.value(time.value())
                )
        });

        // Prepare transmitter
        let mut len = 0;
        self.ll
            .tx_buffer()
            .write(|w| {
                len += frame.encode(&mut w.data(), mac::WriteFooter::No);
                w
            })?;
        self.ll
            .tx_fctrl()
            .modify(|_, w| {
                let tflen = len as u8 + 2;
                w
                    .tflen(tflen) // data length + two-octet CRC
                    .tfle(0)      // no non-standard length extension
                    .txboffs(0)   // no offset in TX_BUFFER
            })?;

        // Start transmission
        self.ll
            .sys_ctrl()
            .modify(|_, w|
                if delayed_time.is_some() { w.txdlys(0b1) } else { w }
                    .txstrt(0b1)
            )?;

        Ok(TxFuture(self))
    }

    /// Attempt to receive an IEEE 802.15.4 MAC frame
    ///
    /// Initializes the receiver, then returns an [`RxFuture`] that allows the
    /// caller to wait for a message.
    ///
    /// Only frames addressed to this device will be received.
    pub fn receive(&mut self)
        -> Result<RxFuture<SPI, CS>, Error<SPI>>
    {
        // For unknown reasons, the DW1000 gets stuck in RX mode without ever
        // receiving anything, after receiving one good frame. Reset the
        // receiver to make sure its in a valid state before attempting to
        // receive anything.
        self.ll
            .pmsc_ctrl0()
            .modify(|_, w|
                w.softreset(0b1110) // reset receiver
            )?;
        self.ll
            .pmsc_ctrl0()
            .modify(|_, w|
                w.softreset(0b1111) // clear reset
            )?;

        // We're already resetting the receiver in the previous step, and that's
        // good enough to make my example program that's both sending and
        // receiving work very reliably over many hours (that's not to say it
        // comes unreliable after those hours, that's just when my test
        // stopped). However, I've seen problems with an example program that
        // only received, never sent, data. That got itself into some weird
        // state where it couldn't receive anymore.
        // I suspect that's because that example didn't have the following line
        // of code, while the send/receive example had that line of code, being
        // called from `send`.
        // While I haven't, as of this writing, run any hours-long tests to
        // confirm this does indeed fix the receive-only example, it seems
        // (based on my eyeball-only measurements) that the RX/TX example is
        // dropping fewer frames now.
        self.force_idle()?;

        // Enable frame filtering
        self.ll
            .sys_cfg()
            .modify(|_, w|
                w
                    .ffen(0b1) // enable frame filtering
                    .ffab(0b1) // receive beacon frames
                    .ffad(0b1) // receive data frames
                    .ffaa(0b1) // receive acknowledgement frames
                    .ffam(0b1) // receive MAC command frames
            )?;

        // Set PLLLDT bit in EC_CTRL. According to the documentation of the
        // CLKPLL_LL bit in SYS_STATUS, this bit needs to be set to ensure the
        // reliable operation of the CLKPLL_LL bit. Since I've seen that bit
        // being set, I want to make sure I'm not just seeing crap.
        self.ll
            .ec_ctrl()
            .modify(|_, w|
                w.pllldt(0b1)
            )?;

        // Now that PLLLDT is set, clear all bits in SYS_STATUS that depend on
        // it for reliable operation. After that is done, these bits should work
        // reliably.
        self.ll
            .sys_status()
            .write(|w|
                w
                    .cplock(0b1)
                    .clkpll_ll(0b1)
            )?;

        // If we were going to receive at 110 kbps, we'd need to set the RXM110K
        // bit in the System Configuration register. We're expecting to receive
        // at 850 kbps though, so the default is fine. See section 4.1.3 for a
        // detailed explanation.

        self.ll
            .sys_ctrl()
            .modify(|_, w|
                w.rxenab(0b1)
            )?;

        Ok(RxFuture(self))
    }


    /// Force the DW1000 into IDLE mode
    ///
    /// Any ongoing RX/TX operations will be aborted.
    pub fn force_idle(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.ll.sys_ctrl().write(|w| w.trxoff(0b1))?;
        while self.ll.sys_ctrl().read()?.trxoff() == 0b1 {}

        Ok(())
    }

    /// Clear all interrupt flags
    pub fn clear_interrupts(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.ll.sys_mask().write(|w| w)?;
        Ok(())
    }

    /// Wait for the transmission to finish
    ///
    /// It is recommended to use `TxFuture::wait()` instead.
    pub fn wait_transmission(&mut self)
        -> nb::Result<(), Error<SPI>>
    {
        // Check Half Period Warning Counter. If this is a delayed transmission,
        // this will indicate that the delay was too short, and the frame was
        // sent too late.
        let evc_hpw = self.ll()
            .evc_hpw()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?
            .value();
        if evc_hpw != 0 {
            return Err(nb::Error::Other(Error::DelayedSendTooLate));
        }

        // Check Transmitter Power-Up Warning Counter. If this is a delayed
        // transmission, this indicates that the transmitter was still powering
        // up while sending, and the frame preamble might not have transmit
        // correctly.
        let evc_tpw = self.ll()
            .evc_tpw()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?
            .value();
        if evc_tpw != 0 {
            return Err(nb::Error::Other(Error::DelayedSendPowerUpWarning));
        }

        // ATTENTION:
        // If you're changing anything about which SYS_STATUS flags are being
        // checked in this method, also make sure to update `enable_interrupts`.
        let sys_status = self.ll()
            .sys_status()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        // Has the frame been sent?
        if sys_status.txfrs() == 0b0 {
            // Frame has not been sent
            return Err(nb::Error::WouldBlock);
        }

        // Frame sent. Reset all progress flags.
        self.ll()
            .sys_status()
            .write(|w|
                w
                    .txfrb(0b1) // Transmit Frame Begins
                    .txprs(0b1) // Transmit Preamble Sent
                    .txphs(0b1) // Transmit PHY Header Sent
                    .txfrs(0b1) // Transmit Frame Sent
            )
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        Ok(())
    }

    /// Enables interrupts for the events that `wait` checks
    ///
    /// It is recommended to use `TxFuture::enable_interrupts()` instead
    pub fn enable_interrupts_transmission(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.ll().sys_mask().write(|w| w.mtxfrs(0b1))?;
        Ok(())
    }

    /// Wait for receive operation to finish
    ///
    /// It is recommended to use `RxFuture::wait()` instead.
    pub fn wait_reception<'b>(&mut self, buffer: &'b mut [u8])
        -> nb::Result<Message<'b>, Error<SPI>>
    {
        // ATTENTION:
        // If you're changing anything about which SYS_STATUS flags are being
        // checked in this method, also make sure to update `enable_interrupts`.
        let sys_status = self.ll()
            .sys_status()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        // Is a frame ready?
        if sys_status.rxdfr() == 0b0 {
            // No frame ready. Check for errors.
            if sys_status.rxfce() == 0b1 {
                return Err(nb::Error::Other(Error::Fcs));
            }
            if sys_status.rxphe() == 0b1 {
                return Err(nb::Error::Other(Error::Phy));
            }
            if sys_status.rxrfsl() == 0b1 {
                return Err(nb::Error::Other(Error::ReedSolomon));
            }
            if sys_status.rxrfto() == 0b1 {
                return Err(nb::Error::Other(Error::FrameWaitTimeout));
            }
            if sys_status.rxovrr() == 0b1 {
                return Err(nb::Error::Other(Error::Overrun));
            }
            if sys_status.rxpto() == 0b1 {
                return Err(nb::Error::Other(Error::PreambleDetectionTimeout));
            }
            if sys_status.rxsfdto() == 0b1 {
                return Err(nb::Error::Other(Error::SfdTimeout));
            }
            // Some error flags that sound like valid errors aren't checked here,
            // because experience has shown that they seem to occur spuriously
            // without preventing a good frame from being received. Those are:
            // - LDEERR: Leading Edge Detection Processing Error
            // - RXPREJ: Receiver Preamble Rejection

            // No errors detected. That must mean the frame is just not ready
            // yet.
            return Err(nb::Error::WouldBlock);
        }

        // Frame is ready. Continue.

        // Wait until LDE processing is done. Before this is finished, the RX
        // time stamp is not available.
        if sys_status.ldedone() == 0b0 {
            return Err(nb::Error::WouldBlock);
        }
        let rx_time = self.ll()
            .rx_time()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?
            .rx_stamp();

        // `rx_time` comes directly from the register, which should always
        // contain a 40-bit timestampt. Unless the hardware or its documentation
        // are buggy, the following should never panic.
        let rx_time = Instant::new(rx_time).unwrap();

        // Reset status bits. This is not strictly necessary, but it helps, if
        // you have to inspect SYS_STATUS manually during debugging.
        self.ll()
            .sys_status()
            .write(|w|
                w
                    .rxprd(0b1)   // Receiver Preamble Detected
                    .rxsfdd(0b1)  // Receiver SFD Detected
                    .ldedone(0b1) // LDE Processing Done
                    .rxphd(0b1)   // Receiver PHY Header Detected
                    .rxphe(0b1)   // Receiver PHY Header Error
                    .rxdfr(0b1)   // Receiver Data Frame Ready
                    .rxfcg(0b1)   // Receiver FCS Good
                    .rxfce(0b1)   // Receiver FCS Error
                    .rxrfsl(0b1)  // Receiver Reed Solomon Frame Sync Loss
                    .rxrfto(0b1)  // Receiver Frame Wait Timeout
                    .ldeerr(0b1)  // Leading Edge Detection Processing Error
                    .rxovrr(0b1)  // Receiver Overrun
                    .rxpto(0b1)   // Preamble Detection Timeout
                    .rxsfdto(0b1) // Receiver SFD Timeout
                    .rxrscs(0b1)  // Receiver Reed-Solomon Correction Status
                    .rxprej(0b1)  // Receiver Preamble Rejection
            )
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        // Read received frame
        let rx_finfo = self.ll()
            .rx_finfo()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;
        let rx_buffer = self.ll()
            .rx_buffer()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error)))?;

        let len = rx_finfo.rxflen() as usize;

        if buffer.len() < len {
            return Err(nb::Error::Other(
                Error::BufferTooSmall { required_len: len }
            ))
        }

        buffer[..len].copy_from_slice(&rx_buffer.data()[..len]);

        let frame = mac::Frame::decode(&buffer[..len], true)
            .map_err(|error| nb::Error::Other(Error::Frame(error)))?;

        Ok(Message {
            rx_time,
            frame,
        })
    }

    /// Enables interrupts for the events that `wait` checks
    ///
    /// It is recommended to use RxFuture::enable_interrupts()` instead
    pub fn enable_interrupts_reception(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.ll()
            .sys_mask()
            .write(|w|
                w
                    .mrxdfr(0b1)
                    .mrxfce(0b1)
                    .mrxphe(0b1)
                    .mrxrfsl(0b1)
                    .mrxrfto(0b1)
                    .mrxovrr(0b1)
                    .mrxpto(0b1)
                    .mrxsfdto(0b1)
                    .mldedone(0b1)
            )?;

        Ok(())
    }
}

impl<SPI, CS, State> DW1000<SPI, CS, State> {
    /// Provides direct access to the register-level API
    ///
    /// Be aware that by using the register-level API, you can invalidate
    /// various assumptions that the high-level API makes about the operation of
    /// the DW1000. Don't use the register-level and high-level APIs in tandem,
    /// unless you know what you're doing.
    pub fn ll(&mut self) -> &mut ll::DW1000<SPI, CS> {
        &mut self.ll
    }
}


/// Represents a transmission that might not have completed yet
pub struct TxFuture<'r, SPI, CS>(&'r mut DW1000<SPI, CS, Ready>);

impl<'r, SPI, CS> TxFuture<'r, SPI, CS>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS:  OutputPin,
{
    /// Wait for the transmission to finish
    ///
    /// This method returns an `nb::Result` to indicate whether the transmission
    /// has finished, or whether it is still ongoing. You can use this to busily
    /// wait for the transmission to finish, for example using `nb`'s `block!`
    /// macro, or you can use it in tandem with [`TxFuture::enable_interrupts`]
    /// and the DW1000 IRQ output to wait in a more energy-efficient manner.
    ///
    /// Handling the DW1000's IRQ output line is out of the scope of this
    /// driver, but please note that if you're using the DWM1001 module or
    /// DWM1001-Dev board, that the `dwm1001` crate has explicit support for
    /// this.
    pub fn wait(&mut self)
        -> nb::Result<(), Error<SPI>>
    {
        self.0.wait_transmission()
    }

    /// Enables interrupts for the events that `wait` checks
    ///
    /// Overwrites any interrupt flags that were previously set.
    pub fn enable_interrupts(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.0.enable_interrupts_transmission()
    }
}


/// Represents a receive operation that might not have finished yet
pub struct RxFuture<'r, SPI, CS>(&'r mut DW1000<SPI, CS, Ready>);

impl<'r, SPI, CS> RxFuture<'r, SPI, CS>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        CS:  OutputPin,
{
    /// Wait for receive operation to finish
    ///
    /// This method returns an `nb::Result` to indicate whether the transmission
    /// has finished, or whether it is still ongoing. You can use this to busily
    /// wait for the transmission to finish, for example using `nb`'s `block!`
    /// macro, or you can use it in tandem with [`RxFuture::enable_interrupts`]
    /// and the DW1000 IRQ output to wait in a more energy-efficient manner.
    ///
    /// Handling the DW1000's IRQ output line is out of the scope of this
    /// driver, but please note that if you're using the DWM1001 module or
    /// DWM1001-Dev board, that the `dwm1001` crate has explicit support for
    /// this.
    pub fn wait<'b>(&mut self, buffer: &'b mut [u8])
        -> nb::Result<Message<'b>, Error<SPI>>
    {
        self.0.wait_reception(buffer)
    }

    /// Enables interrupts for the events that `wait` checks
    ///
    /// Overwrites any interrupt flags that were previously set.
    pub fn enable_interrupts(&mut self)
        -> Result<(), Error<SPI>>
    {
        self.0.enable_interrupts_reception()
    }
}


/// An error that can occur when sending or receiving data
pub enum Error<SPI>
    where SPI: spi::Transfer<u8> + spi::Write<u8>
{
    /// Error occured while using SPI bus
    Spi(ll::Error<SPI>),

    /// Receiver FCS error
    Fcs,

    /// PHY header error
    Phy,

    /// Buffer too small
    BufferTooSmall {
        /// Indicates how large a buffer would have been required
        required_len: usize,
    },

    /// Receiver Reed Solomon Frame Sync Loss
    ReedSolomon,

    /// Receiver Frame Wait Timeout
    FrameWaitTimeout,

    /// Receiver Overrun
    Overrun,

    /// Preamble Detection Timeout
    PreambleDetectionTimeout,

    /// Receiver SFD Timeout
    SfdTimeout,

    /// Frame could not be decoded
    Frame(mac::DecodeError),

    /// A delayed frame could not be sent in time
    ///
    /// Please note that the frame was still sent. Replies could still arrive,
    /// and if it was a ranging frame, the resulting range measurement will be
    /// wrong.
    DelayedSendTooLate,

    /// Transmitter could not power up in time for delayed send
    ///
    /// The frame was still transmitted, but the first bytes of the preamble
    /// were likely corrupted.
    DelayedSendPowerUpWarning,

    /// An error occured while serializing or deserializing data
    Ssmarshal(ssmarshal::Error),
}

impl<SPI> From<ll::Error<SPI>> for Error<SPI>
    where SPI: spi::Transfer<u8> + spi::Write<u8>
{
    fn from(error: ll::Error<SPI>) -> Self {
        Error::Spi(error)
    }
}

impl<SPI> From<ssmarshal::Error> for Error<SPI>
    where SPI: spi::Transfer<u8> + spi::Write<u8>
{
    fn from(error: ssmarshal::Error) -> Self {
        Error::Ssmarshal(error)
    }
}

// We can't derive this implementation, as `Debug` is only implemented
// conditionally for `ll::Debug`.
impl<SPI> fmt::Debug for Error<SPI>
    where
        SPI: spi::Transfer<u8> + spi::Write<u8>,
        <SPI as spi::Transfer<u8>>::Error: fmt::Debug,
        <SPI as spi::Write<u8>>::Error: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::Spi(error) =>
                write!(f, "Spi({:?})", error),
            Error::Fcs =>
                write!(f, "Fcs"),
            Error::Phy =>
                write!(f, "Phy"),
            Error::BufferTooSmall { required_len } =>
                write!(
                    f,
                    "BufferTooSmall {{ required_len: {:?} }}",
                    required_len,
                ),
            Error::ReedSolomon =>
                write!(f, "ReedSolomon"),
            Error::FrameWaitTimeout =>
                write!(f, "FrameWaitTimeout"),
            Error::Overrun =>
                write!(f, "Overrun"),
            Error::PreambleDetectionTimeout =>
                write!(f, "PreambleDetectionTimeout"),
            Error::SfdTimeout =>
                write!(f, "SfdTimeout"),
            Error::Frame(error) =>
                write!(f, "Frame({:?})", error),
            Error::DelayedSendTooLate =>
                write!(f, "DelayedSendTooLate"),
            Error::DelayedSendPowerUpWarning =>
                write!(f, "DelayedSendPowerUpWarning"),
            Error::Ssmarshal(error) =>
                write!(f, "Ssmarshal({:?})", error),
        }
    }
}


/// Indicates that the `DW1000` instance is not initialized yet
pub struct Uninitialized;

/// Indicates that the `DW1000` instance is ready to be used
pub struct Ready;


/// An incoming message
#[derive(Debug)]
pub struct Message<'l> {
    /// The time the message was received
    ///
    /// This time is based on the local system time, as defined in the SYS_TIME
    /// register.
    pub rx_time: Instant,

    /// The MAC frame
    pub frame: mac::Frame<'l>,
}
