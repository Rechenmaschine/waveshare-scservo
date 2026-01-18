//! Example showing both SCSCL and SMS_STS servo buses.
//!
//! This example demonstrates the API for both servo series.
//! In practice, you'd use one or the other depending on your hardware.

#![allow(clippy::doc_markdown)]

use waveshare_scservo::{ScsPositionMove, ScsclBus, SmsStsBus};

/// Mock serial interface for demonstration.
/// Replace with your actual embedded-io serial implementation.
struct MockSerial;

impl embedded_io::ErrorType for MockSerial {
    type Error = core::convert::Infallible;
}

impl embedded_io::Read for MockSerial {
    fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(0)
    }
}

impl embedded_io::Write for MockSerial {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn main() {
    println!("=== SCSCL Series (Potentiometer, Big-Endian) ===\n");
    demo_scscl();

    println!("\n=== SMS_STS Series (Magnetic Encoder, Little-Endian) ===\n");
    demo_sms_sts();
}

fn demo_scscl() {
    let _bus = ScsclBus::new(MockSerial);

    println!("ScsclBus API:");
    println!("  bus.blocking_ping(1)              - Ping servo ID 1");
    println!("  bus.blocking_set_target_position(1, 512)");
    println!("  bus.blocking_current_position_steps(1)");
    println!("  bus.blocking_feedback(1)          - Read all telemetry");
    println!();

    println!("SCSCL-specific:");
    println!("  bus.blocking_pwm_mode(1)          - Enable PWM mode");
    println!("  bus.blocking_write_pwm(1, 500)    - Write PWM value");
    println!();

    println!("Configuration:");
    println!("  bus.blocking_set_torque_mode(1, TorqueMode::Enable)");
    println!("  bus.blocking_set_angle_limits(1, 0, 1023)");
    println!("  bus.blocking_set_pid_coefficients(1, 15, 0, 0)");
}

fn demo_sms_sts() {
    let _bus = SmsStsBus::new(MockSerial);

    println!("SmsStsBus API:");
    println!("  bus.blocking_ping(1)              - Ping servo ID 1");
    println!("  bus.blocking_set_target_position(1, 2048)");
    println!("  bus.blocking_current_position_steps(1)");
    println!("  bus.blocking_feedback(1)          - Read all telemetry (signed position)");
    println!();

    println!("SMS_STS-specific:");
    println!("  bus.blocking_wheel_mode(1)        - Enable wheel mode");
    println!("  bus.blocking_position_mode(1)     - Back to position mode");
    println!("  bus.blocking_write_speed(1, 500, 10) - Speed + acceleration");
    println!("  bus.blocking_calibration_ofs(1)   - Calibrate offset");
    println!();

    println!("Sync operations (same API on both buses):");
    println!("  bus.blocking_sync_write_position(&moves)");
    println!("  bus.blocking_sync_read_state(&[1, 2, 3])");
}

/// Example of sync write usage (works on both bus types).
#[allow(dead_code)]
fn sync_write_example<I>(bus: &mut ScsclBus<I>)
where
    I: embedded_io::Read + embedded_io::Write,
{
    let moves = [
        ScsPositionMove { id: 1, position: 100, time: 500, speed: 0 },
        ScsPositionMove { id: 2, position: 200, time: 500, speed: 0 },
        ScsPositionMove { id: 3, position: 300, time: 500, speed: 0 },
    ];

    let _ = bus.blocking_sync_write_position(&moves);
}

/// Example showing the telemetry structure.
#[allow(dead_code)]
fn telemetry_example<I>(bus: &mut SmsStsBus<I>)
where
    I: embedded_io::Read + embedded_io::Write,
{
    if let Ok(telemetry) = bus.blocking_feedback(1) {
        println!("Position: {} steps", telemetry.position);
        println!("Speed: {} steps/s", telemetry.speed);
        println!("Load: {}%", telemetry.load);
        println!("Voltage: {} V", telemetry.voltage);
        println!("Temperature: {} °C", telemetry.temperature);
        println!("Moving: {}", telemetry.moving);
        if let Some(current) = telemetry.current {
            println!("Current: {current} mA");
        }
    }
}
