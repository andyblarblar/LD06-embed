/// A range reading from the sensor.
#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
pub struct Range {
    /// The distance from the unit, in mm.
    pub dist: u16,
    /// The intensity of the scan. 200 is 'average'.
    pub confidence: u8,
}

/// A single scan packet from the Lidar.
///
/// All angles are with clockwise respect to the arrow on the top of the unit.
#[derive(Copy, Clone, Debug, Default)]
pub struct PartialScan {
    /// The rotational speed of the unit, in degrees per second.
    pub radar_speed: u16,
    /// The starting angle of this scan, in degrees.
    pub start_angle: f32,
    /// The measured ranges.
    ///
    /// The first range angle is at [start_angle].
    pub data: [Range; 12],
    /// The ending angle of this scan, in degrees.
    pub end_angle: f32,
    /// The timestamp of this scan, in ms. This will roll over at 30000.
    pub stamp: u16,
    /// The CRC check from the lidar.
    pub crc: u8,
}

impl PartialScan {
    /// Gets the angular step per range reading.
    pub fn get_step(&self) -> f32 {
        (self.end_angle - self.start_angle + 360.0) / (self.data.len() - 1) as f32
    }

    /// Calculates the angle the nth reading was at in this packet.
    /// The reading number in this case is 0 indexed.
    pub fn get_angle_of_reading(&self, reading_num: u16) -> f32 {
        let angle = self.start_angle + self.get_step() * (reading_num) as f32;
        let spins = (angle / 360.0).floor();
        angle - 360.0 * spins
    }
}