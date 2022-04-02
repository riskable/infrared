#![no_std]
#![no_main]

// The writeln! trait.
use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_halt as _;

use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

use solderparty_rp2040_stamp::{
    hal::{
        self,
        clocks::init_clocks_and_plls,
        gpio::{bank0::Gpio3, Floating, Input, Interrupt, Pin},
        pac,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
    },
    pac::interrupt,
    Pins, XOSC_CRYSTAL_FREQ,
};

use infrared::{
    protocol::NecApple,
    receiver::{Event, PinInput},
    Receiver,
};

type IrPin = Pin<Gpio3, Input<Floating>>;
type IrProto = NecApple;
type IrCommand = <IrProto as infrared::Protocol>::Cmd;

type IrReceiver = Receiver<IrProto, Event, PinInput<IrPin>>;

pub static IR_RECEIVER: Mutex<RefCell<Option<IrReceiver>>> = Mutex::new(RefCell::new(None));
pub static TIMER: Mutex<RefCell<Option<hal::Timer>>> = Mutex::new(RefCell::new(None));
pub static CMD: Mutex<RefCell<Option<IrCommand>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup the pin connected to the Receiver module
    let ir_pin = pins.gpio3.into_floating_input();
    ir_pin.set_interrupt_enabled(Interrupt::EdgeHigh, true);
    ir_pin.set_interrupt_enabled(Interrupt::EdgeLow, true);

    let receiver = Receiver::builder()
        .protocol::<IrProto>()
        .pin(ir_pin)
        .build();

    // The timer has a resolution of 1 us and is used for keeping track of time in the
    // pin interrupt
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    cortex_m::interrupt::free(|cs| {
        IR_RECEIVER.borrow(cs).replace(Some(receiver));
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        // Enable the GPIO pin interrupt
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        let cmd = cortex_m::interrupt::free(|cs| CMD.borrow(cs).take());

        if let Some(cmd) = cmd {
            let mut s = heapless::String::<128>::new();
            writeln!(&mut s, "cmd: {:?}\r\n", cmd).unwrap();
            let _ = serial.write(s.as_bytes());
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            let _ = serial.read(&mut buf);
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn IO_IRQ_BANK0() {
    static mut PREV_EDGE: u32 = 0;

    cortex_m::interrupt::free(|cs| {
        let timer = TIMER.borrow(cs).borrow_mut();
        let timer = timer.as_ref().unwrap();

        let mut receiver = IR_RECEIVER.borrow(cs).borrow_mut();
        let receiver = receiver.as_mut().unwrap();

        // Clear the interrupts
        let pin = receiver.pin();
        pin.clear_interrupt(Interrupt::EdgeHigh);
        pin.clear_interrupt(Interrupt::EdgeLow);

        // Get current time and calculate the time since the previous edge
        let now = timer.get_counter_low();
        let dt = now.wrapping_sub(*PREV_EDGE);
        *PREV_EDGE = now;

        if let Ok(Some(cmd)) = receiver.event(dt) {
            CMD.borrow(cs).borrow_mut().replace(cmd);
        }
    });
}