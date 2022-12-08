use ld06_embed::error::ParseError;
use ld06_embed::LD06;
use linux_embedded_hal::nb::Error;
use linux_embedded_hal::serial_core::{
    BaudOther, CharSize, FlowControl, ParityNone, PortSettings, SerialPort, StopBits,
};
use linux_embedded_hal::Serial;
use std::path::Path;

fn main() {
    let port = Path::new("/dev/ttyUSB0");
    let mut serial = Serial::open(port).unwrap();

    let conf = PortSettings {
        baud_rate: BaudOther(230_400),
        char_size: CharSize::Bits8,
        parity: ParityNone,
        stop_bits: StopBits::Stop1,
        flow_control: FlowControl::FlowNone,
    };
    serial.0.configure(&conf).unwrap();

    let mut ld06 = LD06::new(serial);

    loop {
        match ld06.read_next_byte() {
            Ok(None) => {}
            Err(err) => match err {
                Error::Other(parse_err) => match parse_err {
                    ParseError::SerialErr(_) => {
                        println!("Serial issue")
                    }
                    ParseError::CrcFail => {
                        println!("CRC failed")
                    }
                },
                Error::WouldBlock => {
                    println!("Would block")
                }
            },
            Ok(Some(scan)) => {
                println!("scan: {:?}", scan);
            }
        }
    }
}
