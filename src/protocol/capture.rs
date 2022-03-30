use core::marker::PhantomData;

use crate::receiver::time::{InfraMonotonic};
use crate::{
    receiver::{DecoderData, Decoder, State},
    Protocol,
};

pub struct Capture<Dur> {
    dur: PhantomData<Dur>,
}

pub struct CaptureData<Dur> {
    pub ts: [Dur; 96],
    pub pos: usize,
}

impl<Dur> DecoderData for CaptureData<Dur> {
    fn reset(&mut self) {
        self.pos = 0;
    }
}

impl<Dur> Protocol for Capture<Dur> {
    type Cmd = [Dur; 96];
}

impl<Mono: InfraMonotonic> Decoder<Mono> for Capture<Mono::Duration> {
    type Data = CaptureData<Mono::Duration>;
    type InternalState = State;
    const PULSE: [u32; 8] = [0, 0, 0, 0, 0, 0, 0, 0];
    const TOL: [u32; 8] = [0, 0, 0, 0, 0, 0, 0, 0];

    fn decoder(_freq: u32) -> Self::Data {
        CaptureData {
            ts: [Mono::ZERO_DURATION; 96],
            pos: 0,
        }
    }

    fn event(
        state: &mut Self::Data,
        _edge: bool,
        dur: Mono::Duration,
    ) -> Self::InternalState {
        if state.pos >= state.ts.len() {
            return State::Done;
        }

        state.ts[state.pos] = dur;
        state.pos += 1;

        State::Receiving
    }

    fn command(state: &Self::Data) -> Option<Self::Cmd> {
        Some(state.ts)
    }
}
