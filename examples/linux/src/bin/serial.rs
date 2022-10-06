use ld06_embed::error::ParseError;
use ld06_embed::LD06;
use linux_embedded_hal::nb::Error;
use linux_embedded_hal::Serial;
use std::path::Path;

fn main() {
    let port = Path::new("/dev/ttyUSB0");
    let serial = Serial::open(port).unwrap();

    let mut ld06 = LD06::new(serial);
    let mut byte_num = 0;

    loop {
        match ld06.read_next_byte() {
            Ok(None) => {
                println!("Read byte {}", byte_num);
                byte_num += 1;
            }
            Err(err) => match err {
                Error::Other(parse_err) => match parse_err {
                    ParseError::SerialErr => {
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
                byte_num = 0;
            }
        }
    }
}
