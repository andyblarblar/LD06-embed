# LD06-embed

This crate is an embedded_hal peripheral driver for the LD06/LD09 drivers sold under various brands.

## Setup
To use this crate, simply connect to the LD06 UART via an interface of your choice, then pass that interface to the 
LD06 struct found in this crate. Note that it seems the LiDAR needs a PWM signal for motor control as it has a PWM pin,
but in my experience this has not been the case. Nonetheless, I have provided a wrapper struct that also provides PID
control for this signal, should it be needed for your use case.

## Example
(see [here](./examples/linux) for runnable example on linux)
```rust
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
```
