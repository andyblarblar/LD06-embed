pub use crate::scan_types::*;
use byteorder::ByteOrder;

use crate::crc::crc8;
use crate::error::ParseError;
use embedded_hal::serial::Read;
use nb::{Error, Result};
use pid::Pid;

/// An LD06 Peripheral.
///
/// This driver will not control the PWM of the LiDAR, if you are not using the included speed control board
/// please use [into_pid] to add motor speed control. Due to a lack of a proper embedded_hal PWM interface, you
/// will need to apply the PID value to the PWM line yourself.
pub struct LD06<R: Read<u8>> {
    reader: R,
    /// The current scan being processed.
    wip_scan: [u8; 47],
    /// Byte number we are currently on in the wip_scan. Range [0..47).
    packet_idx: u8,
}

impl<R: Read<u8>> LD06<R> {
    /// Creates a new interface to an LD06. The serial port must be configured as follows:
    /// - baud 230400
    /// - 8 Data bits
    /// - 1 Stop bit
    /// - No Parity
    /// - No Flow Control
    pub fn new(reader: R) -> Self {
        LD06 {
            reader,
            wip_scan: [0u8; 47],
            packet_idx: 0,
        }
    }

    /// Reads the next byte from the serial buffer, appending it to the wip scan. If a scan was completed,
    /// it will be returned. If an error occurs, the current packet will be corrupted, but the system will
    /// recover on the start of the next packet.
    ///
    /// This functions blocking behavior will be determined by the underlying serial read function.
    pub fn read_next_byte(&mut self) -> Result<Option<PartialScan>, ParseError<R::Error>> {
        let byte = self.reader.read();

        if let Err(err) = byte {
            return match err {
                // We cannot know what the errors the generic serial will have, so just glob them
                Error::Other(err) => Err(Error::Other(ParseError::SerialErr(err))),
                // Just skip read if would block
                Error::WouldBlock => Ok(None),
            };
        }

        let byte = unsafe { byte.unwrap_unchecked() };

        // The first packet may be misaligned, as the lidar writes at all times. Restart if that is the case.
        if byte == 0x54 && self.packet_idx != 0 {
            self.packet_idx = 0;
        }

        self.wip_scan[self.packet_idx as usize] = byte;
        self.packet_idx += 1;

        // If full scan has been read
        if self.packet_idx == 47 {
            self.packet_idx = 0;

            //Run CRC checksum
            if crc8(&self.wip_scan[0..=46]) != 0 {
                return Err(nb::Error::Other(ParseError::CrcFail));
            }

            let mut packet = PartialScan::default();
            let buf = &self.wip_scan;

            //See docs/refrence.pdf for packet format
            packet.radar_speed = byteorder::LE::read_u16(&buf[2..=3]);
            packet.start_angle = byteorder::LE::read_u16(&buf[4..=5]) as f32 / 100.0;

            // Range data
            for (i, range) in buf[6..12 * 3 + 6 /*6-41*/].chunks(3).enumerate() {
                packet.data[i].dist = byteorder::LE::read_u16(&range[0..=1]);
                packet.data[i].confidence = range[2];
            } //Read up to 41 here

            packet.end_angle = byteorder::LE::read_u16(&buf[42..=43]) as f32 / 100.0;
            packet.stamp = byteorder::LE::read_u16(&buf[44..=45]);
            packet.crc = buf[46];

            Ok(Some(packet))
        } else {
            Ok(None)
        }
    }

    /// Adds motor speed PID control output.
    pub fn into_pid(self) -> LD06Pid<R> {
        let pid = Pid::new(10.0f32, 2.0, 2.0, 500.0, 500.0, 500.0, 14_400.0, 3600.0);

        LD06Pid { inner: self, pid }
    }
}

/// LD06 peripheral driver that includes PID control for the motor.
pub struct LD06Pid<R: Read<u8>> {
    inner: LD06<R>,
    pid: Pid<f32>,
}

impl<R: Read<u8>> LD06Pid<R> {
    /// Reads the next byte from the serial buffer, appending it to the wip scan. If a scan was completed,
    /// it will be returned. If an error occurs, the current packet will be corrupted, but the system will
    /// recover on the start of the next packet.
    ///
    /// This variant will also output the next PID output, in degrees per second the LiDAR should run at.
    ///
    /// This functions blocking behavior will be determined by the underlying serial read function.
    pub fn read_next_byte(&mut self) -> Result<Option<(PartialScan, u16)>, ParseError<R::Error>> {
        let res = self.inner.read_next_byte()?;

        if let Some(scan) = res {
            let out = self.pid.next_control_output(scan.radar_speed as f32);
            Ok(Some((scan, out.output as u16)))
        } else {
            Ok(None)
        }
    }

    /// Returns the max lidar speed, in degrees per second. This can be used to find the duty % to send
    /// to the LiDAR motor given a PID control speed.
    pub fn get_max_lidar_speed(&self) -> u16 {
        14400 //In theory
    }
}

#[cfg(test)]
mod test {
    extern crate std;
    use crate::{Range, LD06};
    use embedded_hal_mock::serial::{Mock, Transaction};
    use std::prelude::rust_2021::*;

    // Example scan from the manual
    const REF_BYTES: &[u8] = &[
        0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00, 0xE5,
        0xD5, 0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2,
        0xC7, 0x00, 0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5, 0xBE, 0x82, 0x3A,
        0x1A, 0x50,
    ];

    #[test]
    fn test_lidar_read() {
        let expectations = &[Transaction::read_many(REF_BYTES)];
        let mock = Mock::new(expectations);

        let mut ld06 = LD06::new(mock);

        let scan = loop {
            if let Ok(Some(scan)) = ld06.read_next_byte() {
                break scan;
            }
        };

        assert_eq!(scan.radar_speed, 2152);
        assert_eq!(scan.start_angle, 324.27);
        assert_eq!(scan.end_angle, 334.7);
        assert_eq!(scan.crc, 0x50);
        assert_eq!(scan.stamp, 0x1a3a);
        assert_eq!(
            scan.data[0],
            Range {
                confidence: 228,
                dist: 224
            }
        );
        assert_eq!(
            scan.data[1],
            Range {
                confidence: 226,
                dist: 220
            }
        );
    }
}
