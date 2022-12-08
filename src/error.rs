pub enum ParseError<E> {
    /// A serial error occurred, wraps the underlying error.
    SerialErr(E),
    /// Lidar CRC check failed.
    CrcFail,
}
