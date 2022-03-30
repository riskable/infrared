//! # Samsung Blu-Ray Player Protocol
//!
//! Protocol used on some Samsung BluRay players and probably other devices from Samsung.
//!
//! Pulse distance coding is used with. After the Header the 16 bit address is sent.
//! Then a pause and then 4 bits of unknown function (could be repeat indicator?)
//! After this the 8 bit command is sent twice, second time inverted.
//!

use core::convert::TryInto;

use crate::receiver::time::InfraMonotonic;
#[cfg(feature = "remotes")]
use crate::receiver::{ProtocolDecoder, DecodingError, State};
use crate::{
    cmd::{AddressCommand, Command},
    protocol::Protocol,
    receiver::{time::PulseSpans, DecoderData},
};

/// Samsung BluRay player protocol
pub struct Sbp;

impl Protocol for Sbp {
    type Cmd = SbpCommand;
}

/// Samsung Blu-ray protocol
pub struct SbpData<Mono: InfraMonotonic> {
    state: SbpState,
    address: u16,
    command: u32,
    since_rising: Mono::Duration,
    spans: PulseSpans<Mono::Duration>,
}

impl<Mono: InfraMonotonic> DecoderData for SbpData<Mono> {
    fn reset(&mut self) {
        self.state = SbpState::Init;
        self.address = 0;
        self.command = 0;
        self.since_rising = Mono::ZERO_DURATION;
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SbpCommand {
    pub address: u16,
    pub command: u8,
    pub valid: bool,
}

impl SbpCommand {
    pub fn unpack(address: u16, mut command: u32) -> Self {
        // Discard the 4 unknown bits
        command >>= 4;

        // Check the checksum
        let valid = ((command ^ (command >> 8)) & 0xFF) == 0xFF;

        Self {
            address,
            command: (command) as u8,
            valid,
        }
    }
}

impl Command for SbpCommand {
    fn is_repeat(&self) -> bool {
        false
    }
}

impl AddressCommand for SbpCommand {
    fn address(&self) -> u32 {
        self.address.into()
    }

    fn command(&self) -> u32 {
        self.command.into()
    }

    fn create(address: u32, command: u32) -> Option<Self> {
        Some(SbpCommand {
            address: address.try_into().ok()?,
            command: command.try_into().ok()?,
            valid: true,
        })
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
// Internal receiver state
pub enum SbpState {
    // Waiting for first pulse
    Init,
    // Receiving address
    Address(u16),
    /// Pause
    Divider,
    // Receiving data
    Command(u16),
    // Command received
    Done,
    // In error state
    Err(DecodingError),
}

impl<Mono: InfraMonotonic> ProtocolDecoder<Mono> for Sbp {
    type Decoder = SbpData<Mono>;
    type InternalState = SbpState;
    const PULSE: [u32; 8] = [
        (4500 + 4500),
        (500 + 4500),
        (500 + 500),
        (500 + 1500),
        0,
        0,
        0,
        0,
    ];
    const TOL: [u32; 8] = [5, 5, 10, 10, 0, 0, 0, 0];

    fn decoder(freq: u32) -> Self::Decoder {
        SbpData {
            state: SbpState::Init,
            address: 0,
            command: 0,
            since_rising: Mono::ZERO_DURATION,
            spans: <Self as ProtocolDecoder<Mono>>::create_pulsespans(freq)
        }
    }

    #[rustfmt::skip]
    fn event(self_: &mut Self::Decoder, rising: bool, dt: Mono::Duration) -> SbpState {
        use SbpPulse::*;
        use SbpState::*;

        if rising {
            let dt = self_.since_rising + dt;
            let pulsewidth = self_.spans.get::<SbpPulse>(dt).unwrap_or(SbpPulse::NotAPulseWidth);

            self_.state = match (self_.state, pulsewidth) {
                (Init,          Sync)   => Address(0),
                (Init,          _)      => Init,

                (Address(15),   One)    => { self_.address |= 1 << 15; Divider }
                (Address(15),   Zero)   => Divider,
                (Address(bit),  One)    => { self_.address |= 1 << bit; Address(bit + 1) }
                (Address(bit),  Zero)   => Address(bit + 1),
                (Address(_),    _)      => Err(DecodingError::Address),

                (Divider,       Paus)   => Command(0),
                (Divider,       _)      => Err(DecodingError::Data),

                (Command(19),   One)    => { self_.command |= 1 << 19; Done }
                (Command(19),   Zero)   => Done,
                (Command(bit),  One)    => { self_.command |= 1 << bit; Command(bit + 1) }
                (Command(bit),  Zero)   => Command(bit + 1),
                (Command(_),    _)      => Err(DecodingError::Data),

                (Done,          _)      => Done,
                (Err(err),      _)      => Err(err),
            };
        } else {
            self_.since_rising = dt;
        }

        self_.state
    }

    fn command(this_: &Self::Decoder) -> Option<Self::Cmd> {
        Some(SbpCommand::unpack(this_.address, this_.command))
    }
}

impl From<SbpState> for State {
    fn from(state: SbpState) -> State {
        use SbpState::*;
        match state {
            Init => State::Idle,
            Done => State::Done,
            Err(e) => State::Error(e),
            _ => State::Receiving,
        }
    }
}

/*
struct SbpTiming {
    /// Header high
    hh: u32,
    /// Header low
    hl: u32,
    /// Repeat low
    pause: u32,
    /// Data high
    data: u32,
    /// Zero low
    zero: u32,
    /// One low
    one: u32,
}

const TIMING: SbpTiming = SbpTiming {
    hh: 4500,
    hl: 4500,
    pause: 4500,
    data: 500,
    zero: 500,
    one: 1500,
};
*/

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SbpPulse {
    Sync = 0,
    Paus = 1,
    Zero = 2,
    One = 3,
    NotAPulseWidth = 4,
}

impl From<usize> for SbpPulse {
    fn from(v: usize) -> Self {
        match v {
            0 => SbpPulse::Sync,
            1 => SbpPulse::Paus,
            2 => SbpPulse::Zero,
            3 => SbpPulse::One,
            _ => SbpPulse::NotAPulseWidth,
        }
    }
}
