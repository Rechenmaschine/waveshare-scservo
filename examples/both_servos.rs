//! Example showing both SCSCL and SMS_STS servo buses.
//!
//! This example demonstrates the API for both servo series.
//! In practice, you'd use one or the other depending on your hardware.
//!
//! # Synchronized Motion: SYNC_WRITE vs REG_WRITE+ACTION
//!
//! Both methods achieve synchronized movement, but work differently:
//!
//! **SYNC_WRITE (Recommended for position commands):**
//! - 1 packet total (broadcast to all servos)
//! - Immediate execution
//! - Most efficient
//! - Only works for commands with protocol-level SYNC_WRITE support
//!
//! **REG_WRITE + ACTION:**
//! - N+1 packets (one per servo + one ACTION broadcast)
//! - Deferred execution (queue then trigger)
//! - More flexible (works with any command)
//! - Use when SYNC_WRITE isn't available for your command

#![allow(clippy::doc_markdown)]

use waveshare_scservo::{
    BROADCAST_ID, ScsPositionMove, ScsclBus, ServoMode, SmsStsBus, TorqueMode,
};

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
    let mut bus = ScsclBus::new(MockSerial);

    // Basic operations
    let _ = bus.blocking_ping(1);
    let _ = bus.blocking_write_position(1, 512);
    let _ = bus.blocking_read_position(1);
    let _ = bus.blocking_read_state(1);

    // Operating mode (consistent API with SMS_STS, emulated via angle limits on SCSCL)
    let _ = bus.blocking_set_operating_mode(1, ServoMode::Wheel);
    let _ = bus.blocking_write_motor(1, 500); // Write motor output
    let _ = bus.blocking_set_operating_mode(1, ServoMode::Position);

    // Configuration
    let _ = bus.blocking_set_torque_mode(1, TorqueMode::Enable);
    let _ = bus.blocking_set_pid_coefficients(1, 15, 0, 0);

    println!("SCSCL demo completed (see code for examples)");
}

fn demo_sms_sts() {
    let mut bus = SmsStsBus::new(MockSerial);

    // Basic operations
    let _ = bus.blocking_ping(1);
    let _ = bus.blocking_write_position(1, 2048);
    let _ = bus.blocking_read_position(1);
    let _ = bus.blocking_read_state(1);

    // SMS_STS has native mode register
    let _ = bus.blocking_set_operating_mode(1, ServoMode::Wheel);
    let _ = bus.blocking_set_operating_mode(1, ServoMode::Position);
    let _ = bus.blocking_calibrate(1);

    // Sync operations
    let _ = bus.blocking_sync_write_position(&[ScsPositionMove {
        id: 1,
        position: 100,
        time: 500,
        speed: 0,
    }]);
    let _ = bus.blocking_sync_read_state(&[1, 2, 3]);

    println!("SMS_STS demo completed (see code for examples)");
}

/// Example: SYNC_WRITE for simultaneous movement.
///
/// **How it works:**
/// - Sends ONE packet to BROADCAST_ID containing all servo commands
/// - Packet format: [address, data_len, id1, data1, id2, data2, id3, data3]
/// - All servos receive and execute immediately
/// - Most efficient for moving multiple servos
///
/// **Limitation:** Only works for commands with SYNC_WRITE support
#[allow(dead_code)]
fn sync_write_example<I>(bus: &mut ScsclBus<I>)
where
    I: embedded_io::Read + embedded_io::Write,
{
    let moves = [
        ScsPositionMove {
            id: 1,
            position: 100,
            time: 500,
            speed: 0,
        },
        ScsPositionMove {
            id: 2,
            position: 200,
            time: 500,
            speed: 0,
        },
        ScsPositionMove {
            id: 3,
            position: 300,
            time: 500,
            speed: 0,
        },
    ];

    // ONE packet sent, all servos start moving simultaneously
    let _ = bus.blocking_sync_write_position(&moves);
}

/// Example: REG_WRITE + ACTION for synchronized movement.
///
/// **How it works:**
/// - REG_WRITE sends individual packets to each servo (id 1, id 2, id 3)
/// - Commands are queued but NOT executed yet
/// - ACTION sends ONE broadcast packet that triggers all queued commands
/// - Total packets: N (for N servos) + 1 (ACTION)
///
/// **When to use:** Works with any command, even those without SYNC_WRITE support.
/// For position commands specifically, prefer SYNC_WRITE (more efficient).
#[allow(dead_code)]
fn reg_write_action_example<I>(bus: &mut SmsStsBus<I>)
where
    I: embedded_io::Read + embedded_io::Write,
{
    // Send 3 individual REG_WRITE packets (one per servo)
    let _ = bus.blocking_reg_write_position(1, 100, 500, 1000); // Packet 1: queued
    let _ = bus.blocking_reg_write_position(2, 200, 500, 1000); // Packet 2: queued
    let _ = bus.blocking_reg_write_position(3, 300, 500, 1000); // Packet 3: queued

    // Send 1 ACTION broadcast packet - triggers all queued commands
    let _ = bus.blocking_action(BROADCAST_ID); // Packet 4: EXECUTE!
}

/// Example showing the telemetry structure.
#[allow(dead_code)]
fn telemetry_example<I>(bus: &mut SmsStsBus<I>)
where
    I: embedded_io::Read + embedded_io::Write,
{
    if let Ok(telemetry) = bus.blocking_read_state(1) {
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
