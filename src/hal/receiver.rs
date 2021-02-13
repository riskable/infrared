//! Embedded-hal based Receivers

use embedded_hal::digital::v2::InputPin;

use crate::recv::{self, InfraredReceiver};

use crate::protocol::InfraredProtocol;
#[cfg(feature = "remotes")]
use crate::remotecontrol::{AsButton, Button, RemoteControl};

/// Event driven embedded-hal receiver
pub struct EventReceiver<PROTOCOL: InfraredReceiver, PIN> {
    recv: crate::recv::EventReceiver<PROTOCOL>,
    pub pin: PIN,
}

impl<PROTOCOL, PIN, PINERR> EventReceiver<PROTOCOL, PIN>
where
    PROTOCOL: InfraredReceiver,
    PIN: InputPin<Error = PINERR>,
{
    /// Create a new EventReceiver
    /// `pin`: The Inputpin connected to the receiver,
    /// `samplerate`: Sample rate of the receiver
    pub fn new(pin: PIN, samplerate: u32) -> Self {
        Self {
            recv: crate::recv::EventReceiver::new(samplerate),
            pin,
        }
    }

    /// Destroy Receiver and hand back pin
    pub fn destroy(self) -> PIN {
        self.pin
    }

    /// Tell the receiver to read the new pin value and update the receiver state machine
    ///
    /// Returns Ok(None) until a command is detected
    #[inline(always)]
    pub fn edge_event(&mut self, dt: u32) -> Result<Option<PROTOCOL::Cmd>, PINERR> {
        let pinval = self.pin.is_low()?;

        match self.recv.edge_event(pinval, dt) {
            Ok(cmd) => Ok(cmd),
            Err(_err) => Ok(None),
        }
    }
}

/// Periodic and polled Embedded hal Receiver
///
/// The poll methods should be called periodically for this receiver to work
pub struct PeriodicReceiver<PROTOCOL: InfraredReceiver, PIN> {
    /// The receiver state machine
    recv: recv::PeriodicReceiver<PROTOCOL>,
    /// Input pin
    pin: PIN,
    /// Internal sample counter
    counter: u32,
}

impl<PIN, PINERR, PROTOCOL> PeriodicReceiver<PROTOCOL, PIN>
where
    PROTOCOL: InfraredReceiver,
    PIN: InputPin<Error = PINERR>,
{
    /// Create a new PeriodicReceiver
    /// `pin` : The gpio pin the hw is connected to
    /// `samplerate` : Rate of which you intend to call poll.
    pub fn new(pin: PIN, samplerate: u32) -> Self {
        Self {
            recv: recv::PeriodicReceiver::new(samplerate),
            pin,
            counter: 0,
        }
    }

    pub fn destroy(self) -> PIN {
        self.pin
    }

    pub fn poll(&mut self) -> Result<Option<PROTOCOL::Cmd>, PINERR> {
        let pinval = self.pin.is_low()?;

        self.counter = self.counter.wrapping_add(1);

        match self.recv.poll(pinval, self.counter) {
            Ok(cmd) => Ok(cmd),
            Err(_err) => Ok(None),
        }
    }

    #[cfg(feature = "remotes")]
    pub fn poll_button<RC: RemoteControl<Cmd = PROTOCOL::Cmd>>(
        &mut self,
    ) -> Result<Option<Button>, PINERR>
    where
        <PROTOCOL as InfraredProtocol>::Cmd: AsButton,
    {
        self.poll().map(|cmd| cmd.and_then(RC::decode))
    }
}

macro_rules! multireceiver {
    (
        $(#[$outer:meta])*
        $name:ident, [ $( ($N:ident, $P:ident) ),* ]
    ) => {

    $(#[$outer])*
    pub struct $name<$( $P: InfraredReceiver ),* , PIN> {
        pin: PIN,
        counter: u32,
        $( $N : recv::PeriodicReceiver<$P> ),*
    }

    impl<PIN, PINERR, $( $P ),* > $name <$( $P ),* , PIN>
    where
        PIN: InputPin<Error = PINERR>,
        $( $P: InfraredReceiver),*,
    {
        pub fn new(pin: PIN, samplerate: u32) -> Self {
            Self {
                pin,
                counter: 0,
                $( $N: recv::PeriodicReceiver::new(samplerate)),*,
            }
        }

        pub fn destroy(self) -> PIN {
            self.pin
        }

        pub fn poll(&mut self) -> Result<( $( Option<$P::Cmd>),*), PINERR> {
            let pinval = self.pin.is_low()?;
            self.counter = self.counter.wrapping_add(1);

            Ok(($(
                match self.$N.poll(pinval, self.counter) {
                    Ok(cmd) => cmd,
                    Err(_err) => None,
                }
            ),* ))
        }
    }
};
}

multireceiver!(
    /// Receiver for two protocols
    PeriodicReceiver2,
    [(recv1, RECV1), (recv2, RECV2)]
);

multireceiver!(
    /// Receiver for three protocols
    PeriodicReceiver3,
    [(recv1, RECV1), (recv2, RECV2), (recv3, RECV3)]
);

multireceiver!(
    /// Receiver for four protocols
    PeriodicReceiver4,
    [
        (recv1, RECV1),
        (recv2, RECV2),
        (recv3, RECV3),
        (recv4, RECV4)
    ]
);

multireceiver!(
    /// Receiver for five protocols
    PeriodicReceiver5,
    [
        (recv1, RECV1),
        (recv2, RECV2),
        (recv3, RECV3),
        (recv4, RECV4),
        (recv5, RECV5)
    ]
);
