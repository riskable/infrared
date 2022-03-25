use crate::receiver::time::InfraMonotonic;
use crate::receiver::NoPinInput;
use crate::{
    receiver::{DecoderData, DecoderStateMachine, Receiver, State},
    Protocol,
};

pub struct BufferIterator<'a, SM, Monotonic, C>
where
    SM: DecoderStateMachine<Monotonic>,
    Monotonic: InfraMonotonic,
    C: From<<SM as Protocol>::Cmd>,
{
    pub(crate) pos: usize,
    pub(crate) buf: &'a [Monotonic::Duration],
    pub(crate) receiver: Receiver<SM, NoPinInput, Monotonic, C>,
}

impl<'a, SM, Monotonic, C> Iterator for BufferIterator<'a, SM, Monotonic, C>
where
    SM: DecoderStateMachine<Monotonic>,
    Monotonic: InfraMonotonic,
    C: From<<SM as Protocol>::Cmd>,
{
    type Item = C;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.pos == self.buf.len() {
                break None;
            }

            let pos_edge = self.pos & 0x1 == 0;
            let dt_us = self.buf[self.pos];
            self.pos += 1;

            let state: State = SM::event(
                &mut self.receiver.state,
                &self.receiver.spans,
                pos_edge,
                dt_us,
            )
            .into();

            match state {
                State::Idle | State::Receiving => {
                    continue;
                }
                State::Done => {
                    let cmd = SM::command(&self.receiver.state);
                    self.receiver.state.reset();
                    break cmd.map(|r| r.into());
                }
                State::Error(_) => {
                    self.receiver.state.reset();
                    break None;
                }
            }
        }
    }
}
