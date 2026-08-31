# waveshare-scservo

`no_std` Rust driver for Waveshare SCServo motors over UART.

- Supports SCSCL and SMS/STS servo series
- Blocking and async APIs via `embedded-io`
- Position, wheel/speed, torque, telemetry, and synchronized commands

> [!IMPORTANT]
> Waveshare's documentation is very limited, so some register definitions are based on guesswork and should be considered best effort. If you encounter a mistake, feel free to open an issue or submit a PR yourself

## Usage

```toml
[dependencies]
waveshare-scservo = "0.1"
```

### Example

```rust
use waveshare_scservo::{ScsclBus, ServoMode};

let mut bus = ScsclBus::new(uart);

bus.blocking_ping(1)?;
bus.blocking_set_operating_mode(1, ServoMode::Position)?;
bus.blocking_write_position(1, 512)?;

bus.ping(1).await?;
bus.set_operating_mode(1, ServoMode::Position).await?;
bus.write_position(1, 512).await?;
```

Use `SmsStsBus` for SMS/STS servos. The UART must implement the corresponding
`embedded-io` traits.

## Cargo Features

- `scscl` (default): Enable the SCSCL series and `ScsclBus`
- `sms_sts` (default): Enable the SMS/STS series and `SmsStsBus`
- `defmt`: Enable `defmt` support for public types
- `std`: Opt out of `no_std`

## License

Licensed under either the [Apache License, Version 2.0](LICENSE-APACHE) or
the [MIT license](LICENSE-MIT) at your option.
