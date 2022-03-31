use core::fmt::Debug;

use crate::{
    receiver::{time::InfraMonotonic, DecodingError, time::PulseSpans},
    Protocol,
};

pub trait ProtocolDecoderAdaptor<Mono: InfraMonotonic>: Protocol {
    /// Type of the decoder
    type Decoder: ProtocolDecoder<Mono, <Self as Protocol>::Cmd>;

    const PULSE: [u32; 8];
    const TOL: [u32; 8];

    /// Create the decoder
    fn decoder(freq: u32) -> Self::Decoder;

    fn create_pulsespans(freq: u32) -> PulseSpans<Mono::Duration> {
        PulseSpans {
            spans: [
                Mono::create_span(freq, Self::PULSE[0], Self::TOL[0]),
                Mono::create_span(freq, Self::PULSE[1], Self::TOL[1]),
                Mono::create_span(freq, Self::PULSE[2], Self::TOL[2]),
                Mono::create_span(freq, Self::PULSE[3], Self::TOL[3]),
                Mono::create_span(freq, Self::PULSE[4], Self::TOL[4]),
                Mono::create_span(freq, Self::PULSE[5], Self::TOL[5]),
                Mono::create_span(freq, Self::PULSE[6], Self::TOL[6]),
                Mono::create_span(freq, Self::PULSE[7], Self::TOL[7]),
            ],
        }
    }
}

/// Protocol decode state machine
pub trait ProtocolDecoder<Mono: InfraMonotonic, Cmd> {
    /// Notify the state machine of a new event
    /// * `edge`: true = positive edge, false = negative edge
    /// * `dt` : Duration since last event
    fn event(&mut self, edge: bool, dt: Mono::Duration) -> State;

    /// Get the command
    /// Returns the data if State == Done, otherwise None
    fn command(&self) -> Option<Cmd>;

    fn reset(&mut self);

    fn spans(&self) -> &PulseSpans<Mono::Duration>;
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Protocol decoder status
pub enum State {
    /// Idle
    Idle,
    /// Receiving data
    Receiving,
    /// Command successfully decoded
    Done,
    /// Error while decoding
    Error(DecodingError),
}
