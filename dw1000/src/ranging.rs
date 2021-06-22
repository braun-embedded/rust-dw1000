//! Implementation of double-sided two-way ranging
//!
//! This ranging technique is described in the DW1000 user manual, section 12.3.
//! This module uses three messages for a range measurement, as described in
//! section 12.3.2.
//!
//! This module defines the messages required, and provides code for sending and
//! decoding them. It is left to the user to tie all that together, by sending
//! out the messages at the right time.
//!
//! There can be some variation in the use of this module, depending on the use
//! case. Here is one example of how this module can be used:
//! 1. Nodes are divided into anchors and tags. Tags are those nodes whose
//!    position interests us. Anchors are placed in known locations to enable
//!    range measurements.
//! 2. Anchors regularly send out pings ([`Ping`]).
//! 3. Tags listen for these pings, and reply with a ranging request
//!    ([`Request`]) for each ping they receive.
//! 4. When an anchor receives a ranging request, it replies with a ranging
//!    response ([`Response`]).
//! 5. Once the tag receives the ranging response, it has all the information it
//!    needs to compute the distance.
//!
//! Please refer to the [examples] in the DWM1001 Board Support Crate for an
//! implementation of this scheme.
//!
//! In this scheme, anchors initiate the exchange, which results in the tag
//! having the distance information. Possible variations include the tag
//! initiating the request and the anchor calculating the distance, or a
//! peer-to-peer scheme without dedicated tags and anchors.
//!
//! Please note that using the code in this module without further processing of
//! the result will yield imprecise measurements. To improve the precision of
//! those measurements, a range bias needs to be applied. Please refer to the
//! user manual, and [this DWM1001 issue] for more information.
//!
//! [`Ping`]: struct.Ping.html
//! [`Request`]: struct.Request.html
//! [`Response`]: struct.Response.html
//! [examples]: https://github.com/braun-robotics/rust-dwm1001/tree/master/examples
//! [this DWM1001 issue]: https://github.com/braun-robotics/rust-dwm1001/issues/55


use core::mem::size_of;

use embedded_hal::{
    blocking::spi,
    digital::v2::OutputPin,
};
use serde::{
    Deserialize,
    Serialize,
};
use ssmarshal;

use crate::{hl, mac, time::{
    Duration,
    Instant,
}, configs::{
    UwbChannel,
    PulseRepetitionFrequency,
    RxConfig
},
DW1000, Error, Ready, Sending, TxConfig
};


/// The transmission delay
///
/// This defines the transmission delay as 10 ms. This should be enough to
/// finish the rest of the preparation and send the message, even if we're
/// running with unoptimized code.
const TX_DELAY: u32 = 10_000_000;


/// Implemented by all ranging messages
pub trait Message: Sized + for<'de> Deserialize<'de> + Serialize {
    /// A prelude that identifies the message
    const PRELUDE: Prelude;

    /// The length of the message's prelude
    ///
    /// This is a bit of a hack that we need until `slice::<impl [T]>::len` is
    /// stable as a const fn.
    const PRELUDE_LEN: usize;

    /// The length of the whole message, including prelude and data
    const LEN: usize = Self::PRELUDE_LEN + size_of::<Self>();

    /// Decodes a received message of this type
    ///
    /// The user is responsible for receiving a message using
    /// [`DW1000::receive`]. Once a message has been received, this method can
    /// be used to check what type of message this is.
    ///
    /// Returns `Ok(None)`, if the message is not of the right type. Otherwise,
    /// returns `Ok(Some(RxMessage<Self>)), if the message is of the right type,
    /// and no error occured.
    fn decode<SPI, CS>(message: &hl::Message)
        -> Result<Option<RxMessage<Self>>, Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        if !message.frame.payload.starts_with(Self::PRELUDE.0) {
            // Not a message of this type
            return Ok(None);
        }

        if message.frame.payload.len() != Self::LEN {
            // Invalid message
            return Err(Error::BufferTooSmall {
                required_len: Self::LEN,
            });
        }

        // The message passes muster. Let's decode it.
        let (payload, _) = ssmarshal::deserialize::<Self>(
            &message.frame.payload[Self::PRELUDE.0.len()..
        ])?;

        Ok(Some(RxMessage {
            rx_time: message.rx_time,
            source:  message.frame.header.source,
            payload,
        }))
    }
}


/// An incoming ranging message
///
/// Contains the received payload, as well as some metadata that's required to
/// create a reply to the message.
#[derive(Debug)]
pub struct RxMessage<T: Message> {
    /// The time the message was received
    pub rx_time: Instant,

    /// The source of the message
    pub source: Option<mac::Address>,

    /// The message data
    pub payload: T,
}


/// An outgoing ranging message
///
/// Contains the payload to be sent, as well as some metadata.
#[derive(Debug)]
pub struct TxMessage<T: Message> {
    /// The recipient of the message
    ///
    /// This is an IEEE 802.15.4 MAC address. This could be a broadcast address,
    /// for messages that are sent to all other nodes in range.
    pub recipient: Option<mac::Address>,

    /// The time this message is going to be sent
    ///
    /// When creating this struct, this is going to be an instant in the near
    /// future. When sending the message, the sending is delayed to make sure it
    /// it sent at exactly this instant.
    pub tx_time: Instant,

    /// The actual message payload
    pub payload: T,
}

impl<T> TxMessage<T> where T: Message {
    /// Send this message via the DW1000
    ///
    /// Serializes the message payload and uses [`DW1000::send`] internally to
    /// send it.
    pub fn send<'r, SPI, CS>(&self, dw1000: DW1000<SPI, CS, Ready>)
        -> Result<DW1000<SPI, CS, Sending>, Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        // Create a buffer that fits the biggest message currently implemented.
        // This is a really ugly hack. The size of the buffer should just be
        // `T::LEN`. Unfortunately that's not possible. See:
        // https://github.com/rust-lang/rust/issues/42863
        const LEN: usize = 48;
        assert!(T::LEN <= LEN);
        let mut buf = [0; LEN];

        buf[..T::PRELUDE.0.len()].copy_from_slice(T::PRELUDE.0);
        ssmarshal::serialize(
            &mut buf[T::PRELUDE.0.len()..],
            &self.payload,
        )?;

        let future = dw1000.send(
            &buf[..T::LEN],
            self.recipient,
            Some(self.tx_time),
            TxConfig::default()
        )?;

        Ok(future)
    }
}


/// Sent before a message's data to identify the message
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct Prelude(pub &'static [u8]);


/// Ranging ping message
///
/// This message is typically sent to initiate a range measurement transaction.
/// See [module documentation] for more info.
///
/// [module documentation]: index.html
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct Ping {
    /// When the ping was sent, in local sender time
    pub ping_tx_time: Instant,
}

impl Ping {
    /// Creates a new ping message
    ///
    /// Only creates the message, but doesn't yet send it. Sets the transmission
    /// time to 10 milliseconds in the future. Make sure to send the message
    /// within that time frame, or the distance measurement will be negatively
    /// affected.
    pub fn new<SPI, CS>(dw1000: &mut DW1000<SPI, CS, Ready>)
        -> Result<TxMessage<Self>, Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        let tx_time = dw1000.sys_time()? + Duration::from_nanos(TX_DELAY);
        let ping_tx_time = tx_time + dw1000.get_tx_antenna_delay()?;

        let payload = Ping {
            ping_tx_time,
        };

        Ok(TxMessage {
            recipient: mac::Address::broadcast(&mac::AddressMode::Short),
            tx_time,
            payload,
        })
    }
}

impl Message for Ping {
    const PRELUDE:     Prelude = Prelude(b"RANGING PING");
    const PRELUDE_LEN: usize   = 12;
}


/// Ranging request message
///
/// This message is typically sent in response to a ranging ping, to request a
/// ranging response. See [module documentation] for more info.
///
/// [module documentation]: index.html
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct Request {
    /// When the original ping was sent, in local time on the anchor
    pub ping_tx_time: Instant,

    /// The time between the ping being received and the reply being sent
    pub ping_reply_time: Duration,

    /// When the ranging request was sent, in local sender time
    pub request_tx_time: Instant,
}

impl Request {
    /// Creates a new ranging request message
    ///
    /// Only creates the message, but doesn't yet send it. Sets the transmission
    /// time to 10 milliseconds in the future. Make sure to send the message
    /// within that time frame, or the distance measurement will be negatively
    /// affected.
    pub fn new<SPI, CS>(
        dw1000: &mut DW1000<SPI, CS, Ready>,
        ping:   &RxMessage<Ping>,
    )
        -> Result<TxMessage<Self>, Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        let tx_time = dw1000.sys_time()? + Duration::from_nanos(TX_DELAY);
        let request_tx_time = tx_time + dw1000.get_tx_antenna_delay()?;

        let ping_reply_time = request_tx_time.duration_since(ping.rx_time);

        let payload = Request {
            ping_tx_time: ping.payload.ping_tx_time,
            ping_reply_time,
            request_tx_time,
        };

        Ok(TxMessage {
            recipient: ping.source,
            tx_time,
            payload,
        })
    }
}

impl Message for Request {
    const PRELUDE:     Prelude = Prelude(b"RANGING REQUEST");
    const PRELUDE_LEN: usize   = 15;
}


/// Ranging response message
///
/// This message is typically sent in response to a ranging request, to wrap up
/// the range measurement transaction.. See [module documentation] for more
/// info.
///
/// [module documentation]: index.html
#[derive(Debug, Deserialize, Serialize)]
#[repr(C)]
pub struct Response {
    /// The time between the ping being received and the reply being sent
    pub ping_reply_time: Duration,

    /// The time between the ping being sent and the reply being received
    pub ping_round_trip_time: Duration,

    /// The time the ranging request was sent, in local sender time
    pub request_tx_time: Instant,

    /// The time between the request being received and a reply being sent
    pub request_reply_time: Duration,
}

impl Response {
    /// Creates a new ranging response message
    ///
    /// Only creates the message, but doesn't yet send it. Sets the transmission
    /// time to 10 milliseconds in the future. Make sure to send the message
    /// within that time frame, or the distance measurement will be negatively
    /// affected.
    pub fn new<SPI, CS>(
        dw1000:  &mut DW1000<SPI, CS, Ready>,
        request: &RxMessage<Request>,
    )
        -> Result<TxMessage<Self>, Error<SPI, CS>>
        where
            SPI: spi::Transfer<u8> + spi::Write<u8>,
            CS:  OutputPin,
    {
        let tx_time = dw1000.sys_time()? + Duration::from_nanos(TX_DELAY);
        let response_tx_time = tx_time + dw1000.get_tx_antenna_delay()?;

        let ping_round_trip_time =
            request.rx_time.duration_since(request.payload.ping_tx_time);
        let request_reply_time =
            response_tx_time.duration_since(request.rx_time);

        let payload = Response {
            ping_reply_time: request.payload.ping_reply_time,
            ping_round_trip_time,
            request_tx_time: request.payload.request_tx_time,
            request_reply_time,
        };

        Ok(TxMessage {
            recipient: request.source,
            tx_time,
            payload,
        })
    }
}

impl Message for Response {
    const PRELUDE:     Prelude = Prelude(b"RANGING RESPONSE");
    const PRELUDE_LEN: usize   = 16;
}


/// Computes the distance to another node from a ranging response
pub fn compute_distance_mm(response: &RxMessage<Response>, rx_config: RxConfig)
    -> Result<u64, ComputeDistanceError>
{
    // To keep variable names to a reasonable length, this function uses `rt` as
    // a short-hand for "reply time" and `rtt` and a short-hand for "round-trip
    // time".

    let ping_rt = response.payload.ping_reply_time.value();
    let ping_rtt = response.payload.ping_round_trip_time.value();
    let request_rt = response.payload.request_reply_time.value();
    let request_rtt = response.rx_time
        .duration_since(response.payload.request_tx_time)
        .value();

    // Compute time of flight according to the formula given in the DW1000 user
    // manual, section 12.3.2.
    let rtt_product = ping_rtt.checked_mul(request_rtt)
        .ok_or(ComputeDistanceError::RoundTripTimesTooLarge)?;
    let rt_product = ping_rt.checked_mul(request_rt)
        .ok_or(ComputeDistanceError::ReplyTimesTooLarge)?;
    let rt_sum = ping_rt.checked_add(request_rt)
        .ok_or(ComputeDistanceError::SumTooLarge)?;
    let rtt_sum = ping_rtt.checked_add(request_rtt)
        .ok_or(ComputeDistanceError::SumTooLarge)?;
    let sum = rt_sum.checked_add(rtt_sum)
        .ok_or(ComputeDistanceError::SumTooLarge)?;
    let time_of_flight = (rtt_product - rt_product) / sum;

    // Nominally, all time units are based on a 64 Ghz clock, meaning each time
    // unit is 1/64 ns.

    const SPEED_OF_LIGHT: u64 = 299_792_458; // m/s or nm/ns

    let distance_nm_times_64 = SPEED_OF_LIGHT.checked_mul(time_of_flight)
        .ok_or(ComputeDistanceError::TimeOfFlightTooLarge)?;
    let distance_mm          = (distance_nm_times_64 / 64) / 1_000_000;

    // Now we need to adjust the distance measurement depending on a couple of factors:
    let base_bias_mm: i64 = match (rx_config.pulse_repetition_frequency, rx_config.channel.is_narrow()) {
        (PulseRepetitionFrequency::Mhz16, true) => 230,
        (PulseRepetitionFrequency::Mhz16, false) => 280,
        (PulseRepetitionFrequency::Mhz64, true) => 170,
        (PulseRepetitionFrequency::Mhz64, false) => 300,
    };
    let distance_fudge_mm = calculate_distance_fudge(distance_mm as i64, rx_config.channel, rx_config.pulse_repetition_frequency);
    let range_bias_mm = distance_fudge_mm - base_bias_mm;

    let corrected_distance_mm = distance_mm as i64 + range_bias_mm;

    if corrected_distance_mm >= 0 {
        Ok(corrected_distance_mm as u64)
    } else {
        Ok(0)
    }
}

struct CalibrationPoint {
    /// This is how much to take off the range
    value_cm: u8,
    /// Lower bound for this point
    lower_bound_cm: u16,
    /// Upper bound for this point
    upper_bound_cm: Option<u16>,
}

impl CalibrationPoint {
    /// Used for binary searching. Tells if the given value is below, in or above the range.
    fn is_in_range(&self, distance_mm: i64) -> core::cmp::Ordering {
        // Test for below lower bound
        let lower = self.lower_bound_cm as i64 * 10;
        if distance_mm < lower {
            return core::cmp::Ordering::Less;
        }

        // Test for above upper bound
        if let Some(upper_bound_cm) = self.upper_bound_cm {
            let upper = upper_bound_cm as i64 * 10;
            if distance_mm > upper {
                return core::cmp::Ordering::Greater;
            }
        }

        // It's >= than the lower bound and (either <= than the upper bound, or there's no upper bound)
        core::cmp::Ordering::Equal
    }

    fn get_adjustment_mm(&self) -> i64 {
        self.value_cm as i64 * 10
    }
}

fn calculate_distance_fudge(distance_mm: i64, channel: UwbChannel, prf: PulseRepetitionFrequency) -> i64 {
    let table = match (channel, prf) {
        (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz16) => &CHANNEL1_PRF16_VALUES[..],
        (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz16) => &CHANNEL2_PRF16_VALUES[..],
        (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz16) => &CHANNEL3_PRF16_VALUES[..],
        (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz16) => &CHANNEL4_PRF16_VALUES[..],
        (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz16) => &CHANNEL5_PRF16_VALUES[..],
        (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz16) => &CHANNEL7_PRF16_VALUES[..],
        (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz64) => &CHANNEL1_PRF64_VALUES[..],
        (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz64) => &CHANNEL2_PRF64_VALUES[..],
        (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz64) => &CHANNEL3_PRF64_VALUES[..],
        (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz64) => &CHANNEL4_PRF64_VALUES[..],
        (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz64) => &CHANNEL5_PRF64_VALUES[..],
        (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz64) => &CHANNEL7_PRF64_VALUES[..],
    };
    match table.binary_search_by(|probe| probe.is_in_range(distance_mm).reverse()) {
        Ok(idx) => table[idx].get_adjustment_mm(),
        Err(e) => panic!("Table error {:?} {} mm {:?} {:?}. {:p}", e, distance_mm, channel, prf, table.as_ptr())
    }
}

static CHANNEL1_PRF16_VALUES: [CalibrationPoint; 36] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 25, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 125, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 175, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 225, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 325, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 375, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 450, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 500, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 575, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 625, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 700, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 750, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 825, upper_bound_cm: Some(900) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 900, upper_bound_cm: Some(1000) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 1000, upper_bound_cm: Some(1075) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 1075, upper_bound_cm: Some(1175) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 1175, upper_bound_cm: Some(1250) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1250, upper_bound_cm: Some(1350) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 1350, upper_bound_cm: Some(1450) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 1450, upper_bound_cm: Some(1575) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 1575, upper_bound_cm: Some(1650) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 1650, upper_bound_cm: Some(1775) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 1775, upper_bound_cm: Some(1900) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 1900, upper_bound_cm: Some(2050) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 2050, upper_bound_cm: Some(2225) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 2225, upper_bound_cm: Some(2450) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 2450, upper_bound_cm: Some(2725) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 2725, upper_bound_cm: Some(3175) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 3175, upper_bound_cm: Some(3875) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 3875, upper_bound_cm: Some(5550) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 5550, upper_bound_cm: None },
];

static CHANNEL2_PRF16_VALUES: [CalibrationPoint; 37] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 50, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 125, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 150, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 250, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 325, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 375, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 450, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 500, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 550, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 600, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 675, upper_bound_cm: Some(725) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 725, upper_bound_cm: Some(800) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 800, upper_bound_cm: Some(875) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 875, upper_bound_cm: Some(950) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 950, upper_bound_cm: Some(1025) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 1025, upper_bound_cm: Some(1100) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1100, upper_bound_cm: Some(1175) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 1175, upper_bound_cm: Some(1275) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 1275, upper_bound_cm: Some(1375) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 1375, upper_bound_cm: Some(1450) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 1450, upper_bound_cm: Some(1550) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 1550, upper_bound_cm: Some(1650) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 1650, upper_bound_cm: Some(1775) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 1775, upper_bound_cm: Some(1950) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 1950, upper_bound_cm: Some(2125) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 2125, upper_bound_cm: Some(2400) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 2400, upper_bound_cm: Some(2775) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 2775, upper_bound_cm: Some(3375) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 3375, upper_bound_cm: Some(4850) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 4850, upper_bound_cm: Some(6000) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 6000, upper_bound_cm: None },
];

static CHANNEL3_PRF16_VALUES: [CalibrationPoint; 37] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 125, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 250, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 300, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 350, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 400, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 450, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 500, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 550, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 600, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 650, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 700, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 775, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 825, upper_bound_cm: Some(900) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 900, upper_bound_cm: Some(975) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 975, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 1050, upper_bound_cm: Some(1125) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 1125, upper_bound_cm: Some(1225) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 1225, upper_bound_cm: Some(1300) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 1300, upper_bound_cm: Some(1375) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 1375, upper_bound_cm: Some(1475) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 1475, upper_bound_cm: Some(1575) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 1575, upper_bound_cm: Some(1725) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 1725, upper_bound_cm: Some(1900) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 1900, upper_bound_cm: Some(2125) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 2125, upper_bound_cm: Some(2450) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 2450, upper_bound_cm: Some(3000) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 3000, upper_bound_cm: Some(4325) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 4325, upper_bound_cm: Some(5325) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 5325, upper_bound_cm: None },
];

static CHANNEL4_PRF16_VALUES: [CalibrationPoint; 61] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 250, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 325, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 350, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 400, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 425, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 450, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 475, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 500, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 525, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 550, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 575, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 600, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 650, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 675, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 700, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 750, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 775, upper_bound_cm: Some(800) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 800, upper_bound_cm: Some(850) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 850, upper_bound_cm: Some(900) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 900, upper_bound_cm: Some(950) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 950, upper_bound_cm: Some(1000) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 1000, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 1050, upper_bound_cm: Some(1100) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 1100, upper_bound_cm: Some(1150) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 1150, upper_bound_cm: Some(1200) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 1200, upper_bound_cm: Some(1250) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 1250, upper_bound_cm: Some(1300) },
    CalibrationPoint { value_cm: 37, lower_bound_cm: 1300, upper_bound_cm: Some(1375) },
    CalibrationPoint { value_cm: 38, lower_bound_cm: 1375, upper_bound_cm: Some(1425) },
    CalibrationPoint { value_cm: 39, lower_bound_cm: 1425, upper_bound_cm: Some(1475) },
    CalibrationPoint { value_cm: 40, lower_bound_cm: 1475, upper_bound_cm: Some(1525) },
    CalibrationPoint { value_cm: 41, lower_bound_cm: 1525, upper_bound_cm: Some(1575) },
    CalibrationPoint { value_cm: 42, lower_bound_cm: 1575, upper_bound_cm: Some(1650) },
    CalibrationPoint { value_cm: 43, lower_bound_cm: 1650, upper_bound_cm: Some(1700) },
    CalibrationPoint { value_cm: 44, lower_bound_cm: 1700, upper_bound_cm: Some(1775) },
    CalibrationPoint { value_cm: 45, lower_bound_cm: 1775, upper_bound_cm: Some(1850) },
    CalibrationPoint { value_cm: 46, lower_bound_cm: 1850, upper_bound_cm: Some(1950) },
    CalibrationPoint { value_cm: 47, lower_bound_cm: 1950, upper_bound_cm: Some(2025) },
    CalibrationPoint { value_cm: 48, lower_bound_cm: 2025, upper_bound_cm: Some(2125) },
    CalibrationPoint { value_cm: 49, lower_bound_cm: 2125, upper_bound_cm: Some(2225) },
    CalibrationPoint { value_cm: 50, lower_bound_cm: 2225, upper_bound_cm: Some(2350) },
    CalibrationPoint { value_cm: 51, lower_bound_cm: 2350, upper_bound_cm: Some(2475) },
    CalibrationPoint { value_cm: 52, lower_bound_cm: 2475, upper_bound_cm: Some(2600) },
    CalibrationPoint { value_cm: 53, lower_bound_cm: 2600, upper_bound_cm: Some(2750) },
    CalibrationPoint { value_cm: 54, lower_bound_cm: 2750, upper_bound_cm: Some(2900) },
    CalibrationPoint { value_cm: 55, lower_bound_cm: 2900, upper_bound_cm: Some(3075) },
    CalibrationPoint { value_cm: 56, lower_bound_cm: 3075, upper_bound_cm: Some(3250) },
    CalibrationPoint { value_cm: 57, lower_bound_cm: 3250, upper_bound_cm: Some(3475) },
    CalibrationPoint { value_cm: 58, lower_bound_cm: 3475, upper_bound_cm: Some(3750) },
    CalibrationPoint { value_cm: 59, lower_bound_cm: 3750, upper_bound_cm: Some(4100) },
    CalibrationPoint { value_cm: 60, lower_bound_cm: 4100, upper_bound_cm: Some(4550) },
    CalibrationPoint { value_cm: 61, lower_bound_cm: 4550, upper_bound_cm: Some(5175) },
    CalibrationPoint { value_cm: 62, lower_bound_cm: 5175, upper_bound_cm: Some(5950) },
    CalibrationPoint { value_cm: 63, lower_bound_cm: 5950, upper_bound_cm: None },
];

static CHANNEL5_PRF16_VALUES: [CalibrationPoint; 35] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 125, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 150, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 225, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 300, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 350, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 400, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 450, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 500, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 525, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 575, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 625, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 675, upper_bound_cm: Some(725) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 725, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 775, upper_bound_cm: Some(850) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 850, upper_bound_cm: Some(900) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 900, upper_bound_cm: Some(950) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 950, upper_bound_cm: Some(1025) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 1025, upper_bound_cm: Some(1100) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 1100, upper_bound_cm: Some(1200) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 1200, upper_bound_cm: Some(1325) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 1325, upper_bound_cm: Some(1475) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 1475, upper_bound_cm: Some(1700) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 1700, upper_bound_cm: Some(2075) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 2075, upper_bound_cm: Some(3000) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 3000, upper_bound_cm: Some(3700) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 3700, upper_bound_cm: None },
];

static CHANNEL7_PRF16_VALUES: [CalibrationPoint; 58] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 125, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 150, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 250, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 325, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 350, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 400, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 425, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 450, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 475, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 500, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 525, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 550, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 575, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 625, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 650, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 675, upper_bound_cm: Some(725) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 725, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 750, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 775, upper_bound_cm: Some(800) },
    CalibrationPoint { value_cm: 37, lower_bound_cm: 800, upper_bound_cm: Some(850) },
    CalibrationPoint { value_cm: 38, lower_bound_cm: 850, upper_bound_cm: Some(875) },
    CalibrationPoint { value_cm: 39, lower_bound_cm: 875, upper_bound_cm: Some(900) },
    CalibrationPoint { value_cm: 40, lower_bound_cm: 900, upper_bound_cm: Some(950) },
    CalibrationPoint { value_cm: 41, lower_bound_cm: 950, upper_bound_cm: Some(975) },
    CalibrationPoint { value_cm: 42, lower_bound_cm: 975, upper_bound_cm: Some(1000) },
    CalibrationPoint { value_cm: 43, lower_bound_cm: 1000, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 44, lower_bound_cm: 1050, upper_bound_cm: Some(1100) },
    CalibrationPoint { value_cm: 45, lower_bound_cm: 1100, upper_bound_cm: Some(1150) },
    CalibrationPoint { value_cm: 46, lower_bound_cm: 1150, upper_bound_cm: Some(1200) },
    CalibrationPoint { value_cm: 47, lower_bound_cm: 1200, upper_bound_cm: Some(1250) },
    CalibrationPoint { value_cm: 48, lower_bound_cm: 1250, upper_bound_cm: Some(1300) },
    CalibrationPoint { value_cm: 49, lower_bound_cm: 1300, upper_bound_cm: Some(1375) },
    CalibrationPoint { value_cm: 50, lower_bound_cm: 1375, upper_bound_cm: Some(1450) },
    CalibrationPoint { value_cm: 51, lower_bound_cm: 1450, upper_bound_cm: Some(1525) },
    CalibrationPoint { value_cm: 52, lower_bound_cm: 1525, upper_bound_cm: Some(1600) },
    CalibrationPoint { value_cm: 53, lower_bound_cm: 1600, upper_bound_cm: Some(1700) },
    CalibrationPoint { value_cm: 54, lower_bound_cm: 1700, upper_bound_cm: Some(1800) },
    CalibrationPoint { value_cm: 55, lower_bound_cm: 1800, upper_bound_cm: Some(1875) },
    CalibrationPoint { value_cm: 56, lower_bound_cm: 1875, upper_bound_cm: Some(2000) },
    CalibrationPoint { value_cm: 57, lower_bound_cm: 2000, upper_bound_cm: Some(2125) },
    CalibrationPoint { value_cm: 58, lower_bound_cm: 2125, upper_bound_cm: Some(2300) },
    CalibrationPoint { value_cm: 59, lower_bound_cm: 2300, upper_bound_cm: Some(2525) },
    CalibrationPoint { value_cm: 60, lower_bound_cm: 2525, upper_bound_cm: Some(2800) },
    CalibrationPoint { value_cm: 61, lower_bound_cm: 2800, upper_bound_cm: Some(3175) },
    CalibrationPoint { value_cm: 62, lower_bound_cm: 3175, upper_bound_cm: Some(3675) },
    CalibrationPoint { value_cm: 63, lower_bound_cm: 3675, upper_bound_cm: Some(4200) },
    CalibrationPoint { value_cm: 64, lower_bound_cm: 4200, upper_bound_cm: Some(4550) },
    CalibrationPoint { value_cm: 65, lower_bound_cm: 4550, upper_bound_cm: Some(4850) },
    CalibrationPoint { value_cm: 66, lower_bound_cm: 4850, upper_bound_cm: Some(5125) },
    CalibrationPoint { value_cm: 67, lower_bound_cm: 5125, upper_bound_cm: None },
];

static CHANNEL1_PRF64_VALUES: [CalibrationPoint; 25] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 125, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 175, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 250, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 325, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 400, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 475, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 550, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 600, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 675, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 750, upper_bound_cm: Some(800) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 800, upper_bound_cm: Some(875) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 875, upper_bound_cm: Some(950) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 950, upper_bound_cm: Some(1075) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 1075, upper_bound_cm: Some(1200) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 1200, upper_bound_cm: Some(1400) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 1400, upper_bound_cm: Some(1950) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1950, upper_bound_cm: Some(2525) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 2525, upper_bound_cm: Some(3000) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 3000, upper_bound_cm: Some(3925) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 3925, upper_bound_cm: None },
];

static CHANNEL2_PRF64_VALUES: [CalibrationPoint; 24] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 100, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 150, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 225, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 300, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 350, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 425, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 475, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 525, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 600, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 650, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 700, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 775, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 825, upper_bound_cm: Some(925) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 925, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 1050, upper_bound_cm: Some(1225) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 1225, upper_bound_cm: Some(1700) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1700, upper_bound_cm: Some(2225) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 2225, upper_bound_cm: Some(2625) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 2625, upper_bound_cm: Some(3450) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 3450, upper_bound_cm: None },
];

static CHANNEL3_PRF64_VALUES: [CalibrationPoint; 24] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 2, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 125, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 200, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 250, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 325, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 375, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 425, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 475, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 525, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 575, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 625, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 675, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 750, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 825, upper_bound_cm: Some(925) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 925, upper_bound_cm: Some(1100) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 1100, upper_bound_cm: Some(1500) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1500, upper_bound_cm: Some(1975) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 1975, upper_bound_cm: Some(2325) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 2325, upper_bound_cm: Some(3050) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 3050, upper_bound_cm: None },
];

static CHANNEL4_PRF64_VALUES: [CalibrationPoint; 52] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 250, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 325, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 350, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 400, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 425, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 450, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 475, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 500, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 525, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 550, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 600, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 625, upper_bound_cm: Some(675) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 675, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 700, upper_bound_cm: Some(725) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 725, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 27, lower_bound_cm: 750, upper_bound_cm: Some(800) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 800, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 825, upper_bound_cm: Some(850) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 850, upper_bound_cm: Some(875) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 875, upper_bound_cm: Some(925) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 925, upper_bound_cm: Some(975) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 975, upper_bound_cm: Some(1025) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 1025, upper_bound_cm: Some(1075) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 1075, upper_bound_cm: Some(1125) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 1125, upper_bound_cm: Some(1200) },
    CalibrationPoint { value_cm: 37, lower_bound_cm: 1200, upper_bound_cm: Some(1250) },
    CalibrationPoint { value_cm: 38, lower_bound_cm: 1250, upper_bound_cm: Some(1325) },
    CalibrationPoint { value_cm: 39, lower_bound_cm: 1325, upper_bound_cm: Some(1400) },
    CalibrationPoint { value_cm: 40, lower_bound_cm: 1400, upper_bound_cm: Some(1500) },
    CalibrationPoint { value_cm: 41, lower_bound_cm: 1500, upper_bound_cm: Some(1600) },
    CalibrationPoint { value_cm: 42, lower_bound_cm: 1600, upper_bound_cm: Some(1700) },
    CalibrationPoint { value_cm: 43, lower_bound_cm: 1700, upper_bound_cm: Some(1850) },
    CalibrationPoint { value_cm: 44, lower_bound_cm: 1850, upper_bound_cm: Some(2025) },
    CalibrationPoint { value_cm: 45, lower_bound_cm: 2025, upper_bound_cm: Some(2225) },
    CalibrationPoint { value_cm: 46, lower_bound_cm: 2225, upper_bound_cm: Some(2450) },
    CalibrationPoint { value_cm: 47, lower_bound_cm: 2450, upper_bound_cm: Some(2725) },
    CalibrationPoint { value_cm: 48, lower_bound_cm: 2725, upper_bound_cm: Some(3050) },
    CalibrationPoint { value_cm: 49, lower_bound_cm: 3050, upper_bound_cm: Some(3400) },
    CalibrationPoint { value_cm: 50, lower_bound_cm: 3400, upper_bound_cm: Some(3650) },
    CalibrationPoint { value_cm: 51, lower_bound_cm: 3650, upper_bound_cm: Some(3850) },
    CalibrationPoint { value_cm: 52, lower_bound_cm: 3850, upper_bound_cm: Some(4050) },
    CalibrationPoint { value_cm: 53, lower_bound_cm: 4050, upper_bound_cm: Some(4450) },
    CalibrationPoint { value_cm: 54, lower_bound_cm: 4450, upper_bound_cm: Some(5500) },
    CalibrationPoint { value_cm: 55, lower_bound_cm: 5500, upper_bound_cm: Some(6225) },
    CalibrationPoint { value_cm: 56, lower_bound_cm: 6225, upper_bound_cm: None },
];

static CHANNEL5_PRF64_VALUES: [CalibrationPoint; 23] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(25) },
    CalibrationPoint { value_cm: 3, lower_bound_cm: 25, upper_bound_cm: Some(50) },
    CalibrationPoint { value_cm: 5, lower_bound_cm: 50, upper_bound_cm: Some(75) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 75, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 7, lower_bound_cm: 100, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 150, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 9, lower_bound_cm: 175, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 11, lower_bound_cm: 250, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 13, lower_bound_cm: 325, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 14, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 400, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 16, lower_bound_cm: 425, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 475, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 525, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 19, lower_bound_cm: 575, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 650, upper_bound_cm: Some(750) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 750, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 1050, upper_bound_cm: Some(1375) },
    CalibrationPoint { value_cm: 23, lower_bound_cm: 1375, upper_bound_cm: Some(1625) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 1625, upper_bound_cm: Some(2125) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 2125, upper_bound_cm: None },
];

static CHANNEL7_PRF64_VALUES: [CalibrationPoint; 47] = [
    CalibrationPoint { value_cm: 0, lower_bound_cm: 0, upper_bound_cm: Some(100) },
    CalibrationPoint { value_cm: 1, lower_bound_cm: 100, upper_bound_cm: Some(125) },
    CalibrationPoint { value_cm: 4, lower_bound_cm: 125, upper_bound_cm: Some(150) },
    CalibrationPoint { value_cm: 6, lower_bound_cm: 150, upper_bound_cm: Some(175) },
    CalibrationPoint { value_cm: 8, lower_bound_cm: 175, upper_bound_cm: Some(200) },
    CalibrationPoint { value_cm: 10, lower_bound_cm: 200, upper_bound_cm: Some(225) },
    CalibrationPoint { value_cm: 12, lower_bound_cm: 225, upper_bound_cm: Some(250) },
    CalibrationPoint { value_cm: 15, lower_bound_cm: 250, upper_bound_cm: Some(275) },
    CalibrationPoint { value_cm: 17, lower_bound_cm: 275, upper_bound_cm: Some(300) },
    CalibrationPoint { value_cm: 18, lower_bound_cm: 300, upper_bound_cm: Some(325) },
    CalibrationPoint { value_cm: 20, lower_bound_cm: 325, upper_bound_cm: Some(350) },
    CalibrationPoint { value_cm: 21, lower_bound_cm: 350, upper_bound_cm: Some(375) },
    CalibrationPoint { value_cm: 22, lower_bound_cm: 375, upper_bound_cm: Some(400) },
    CalibrationPoint { value_cm: 24, lower_bound_cm: 400, upper_bound_cm: Some(425) },
    CalibrationPoint { value_cm: 25, lower_bound_cm: 425, upper_bound_cm: Some(450) },
    CalibrationPoint { value_cm: 26, lower_bound_cm: 450, upper_bound_cm: Some(475) },
    CalibrationPoint { value_cm: 28, lower_bound_cm: 475, upper_bound_cm: Some(500) },
    CalibrationPoint { value_cm: 29, lower_bound_cm: 500, upper_bound_cm: Some(525) },
    CalibrationPoint { value_cm: 30, lower_bound_cm: 525, upper_bound_cm: Some(550) },
    CalibrationPoint { value_cm: 31, lower_bound_cm: 550, upper_bound_cm: Some(575) },
    CalibrationPoint { value_cm: 32, lower_bound_cm: 575, upper_bound_cm: Some(600) },
    CalibrationPoint { value_cm: 33, lower_bound_cm: 600, upper_bound_cm: Some(625) },
    CalibrationPoint { value_cm: 34, lower_bound_cm: 625, upper_bound_cm: Some(650) },
    CalibrationPoint { value_cm: 35, lower_bound_cm: 650, upper_bound_cm: Some(700) },
    CalibrationPoint { value_cm: 36, lower_bound_cm: 700, upper_bound_cm: Some(725) },
    CalibrationPoint { value_cm: 37, lower_bound_cm: 725, upper_bound_cm: Some(775) },
    CalibrationPoint { value_cm: 38, lower_bound_cm: 775, upper_bound_cm: Some(825) },
    CalibrationPoint { value_cm: 39, lower_bound_cm: 825, upper_bound_cm: Some(875) },
    CalibrationPoint { value_cm: 40, lower_bound_cm: 875, upper_bound_cm: Some(925) },
    CalibrationPoint { value_cm: 41, lower_bound_cm: 925, upper_bound_cm: Some(975) },
    CalibrationPoint { value_cm: 42, lower_bound_cm: 975, upper_bound_cm: Some(1050) },
    CalibrationPoint { value_cm: 43, lower_bound_cm: 1050, upper_bound_cm: Some(1150) },
    CalibrationPoint { value_cm: 44, lower_bound_cm: 1150, upper_bound_cm: Some(1250) },
    CalibrationPoint { value_cm: 45, lower_bound_cm: 1250, upper_bound_cm: Some(1350) },
    CalibrationPoint { value_cm: 46, lower_bound_cm: 1350, upper_bound_cm: Some(1500) },
    CalibrationPoint { value_cm: 47, lower_bound_cm: 1500, upper_bound_cm: Some(1675) },
    CalibrationPoint { value_cm: 48, lower_bound_cm: 1675, upper_bound_cm: Some(1875) },
    CalibrationPoint { value_cm: 49, lower_bound_cm: 1875, upper_bound_cm: Some(2075) },
    CalibrationPoint { value_cm: 50, lower_bound_cm: 2075, upper_bound_cm: Some(2250) },
    CalibrationPoint { value_cm: 51, lower_bound_cm: 2250, upper_bound_cm: Some(2375) },
    CalibrationPoint { value_cm: 52, lower_bound_cm: 2375, upper_bound_cm: Some(2500) },
    CalibrationPoint { value_cm: 53, lower_bound_cm: 2500, upper_bound_cm: Some(2750) },
    CalibrationPoint { value_cm: 54, lower_bound_cm: 2750, upper_bound_cm: Some(3375) },
    CalibrationPoint { value_cm: 55, lower_bound_cm: 3375, upper_bound_cm: Some(3825) },
    CalibrationPoint { value_cm: 56, lower_bound_cm: 3825, upper_bound_cm: Some(4300) },
    CalibrationPoint { value_cm: 57, lower_bound_cm: 4300, upper_bound_cm: Some(4800) },
    CalibrationPoint { value_cm: 58, lower_bound_cm: 4800, upper_bound_cm: None },
];

/// Returned from [`compute_distance_mm`] in case of an error
#[derive(Debug)]
pub enum ComputeDistanceError {
    /// Reply times are too large to be multiplied
    ReplyTimesTooLarge,

    /// Round-trip times are too large to be multiplied
    RoundTripTimesTooLarge,

    /// The sum computed as part of the algorithm is too large
    SumTooLarge,

    /// The time of flight is so large, the distance calculation would overflow
    TimeOfFlightTooLarge,
}
