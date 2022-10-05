pub use crate::scan_types::*;
use byteorder::ByteOrder;

use crate::crc::crc8;
use crate::error::ParseError;
use embedded_hal::serial::Read;
use nb::Result;

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
    pub fn read_next_byte(&mut self) -> Result<Option<PartialScan>, ParseError> {
        let byte = self.reader.read().map_err(|_| ParseError::SerialErr)?;

        // The first packet may be misaligned, as the lidar writes at all times. Restart if that is the case.
        if byte == 0x54 && self.packet_idx != 0 {
            self.packet_idx = 0;
        }

        self.wip_scan[self.packet_idx as usize] = byte;
        self.packet_idx += 1;

        // If full scan has been read
        if self.packet_idx == 47 {
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

            self.packet_idx = 0;
            Ok(Some(packet))
        } else {
            Ok(None)
        }
    }
}