// Two-servo ordered flow on a UART (SCSCL series).
// For SMS_STS, swap ScsclBus + Scscl* types for SmsStsBus + Sms* types.

use std::time::Duration;

use waveshare_scservo::{
    BROADCAST_ID, ScsclBus, ScsclMotorCommand, ScsclPositionMove, ServoMode, TorqueMode,
};

struct StdUart(Box<dyn serialport::SerialPort>);

impl embedded_io::ErrorType for StdUart {
    type Error = embedded_io::ErrorKind;
}

impl embedded_io::Read for StdUart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        std::io::Read::read(&mut self.0, buf).map_err(|_| embedded_io::ErrorKind::Other)
    }
}

impl embedded_io::Write for StdUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        std::io::Write::write(&mut self.0, buf).map_err(|_| embedded_io::ErrorKind::Other)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        std::io::Write::flush(&mut self.0).map_err(|_| embedded_io::ErrorKind::Other)
    }
}

fn open_uart(path: &str, baud: u32) -> StdUart {
    let port = serialport::new(path, baud)
        .timeout(Duration::from_millis(50))
        .open()
        .expect("open uart");
    StdUart(port)
}

fn main() {
    let mut args = std::env::args().skip(1);
    let port = match args.next() {
        Some(value) => value,
        None => print_help(),
    };
    let baud = match args.next().and_then(|value| value.parse().ok()) {
        Some(value) => value,
        None => print_help(),
    };

    let mut bus = ScsclBus::new(open_uart(&port, baud));
    let ids = find_two_ids(&mut bus);

    for &id in &ids {
        bus.blocking_set_torque_mode(id, TorqueMode::Enable)
            .expect("torque on");
        bus.blocking_set_operating_mode(id, ServoMode::Position)
            .expect("position mode");
    }

    let tmp = temp_ids(ids);
    bus.blocking_set_id(ids[0], tmp[0]).expect("set id 1");
    bus.blocking_set_id(ids[1], tmp[1]).expect("set id 2");
    bus.blocking_set_id(tmp[0], ids[0]).expect("restore id 1");
    bus.blocking_set_id(tmp[1], ids[1]).expect("restore id 2");

    let moves = [
        ScsclPositionMove {
            id: ids[0],
            position: 800,
            time: 500,
            speed: 0,
        },
        ScsclPositionMove {
            id: ids[1],
            position: 200,
            time: 500,
            speed: 0,
        },
    ];
    bus.blocking_sync_write_position(&moves)
        .expect("sync move");

    let states = bus.blocking_sync_read_state(&ids).expect("sync read");
    println!("pos: {} {}", states[0].position(), states[1].position());

    bus.blocking_reg_write_position(ids[0], 300, 300, 0)
        .expect("reg write 1");
    bus.blocking_reg_write_position(ids[1], 700, 300, 0)
        .expect("reg write 2");
    bus.blocking_action(BROADCAST_ID).expect("action");

    for &id in &ids {
        bus.blocking_set_operating_mode(id, ServoMode::Wheel)
            .expect("wheel mode");
    }
    let motors = [
        ScsclMotorCommand {
            id: ids[0],
            output: 300,
        },
        ScsclMotorCommand {
            id: ids[1],
            output: -300,
        },
    ];
    bus.blocking_sync_write_motor(&motors)
        .expect("sync motor");

    for &id in &ids {
        bus.blocking_set_torque_mode(id, TorqueMode::Disable)
            .expect("torque off");
    }
}

fn print_help() -> ! {
    eprintln!("usage: cargo run --example both_servos -- <port> <baud>");
    std::process::exit(2)
}

fn find_two_ids(bus: &mut ScsclBus<StdUart>) -> [u8; 2] {
    let mut ids = [0u8; 2];
    let mut count = 0usize;
    for id in 1..=253 {
        if bus.blocking_ping(id).is_ok() {
            ids[count] = id;
            count += 1;
            if count == 2 {
                return ids;
            }
        }
    }
    panic!("need two servos");
}

fn temp_ids(ids: [u8; 2]) -> [u8; 2] {
    let (mut a, mut b) = (252u8, 253u8);
    if a == ids[0] || a == ids[1] || b == ids[0] || b == ids[1] {
        a = 250;
        b = 251;
    }
    if a == ids[0] || a == ids[1] || b == ids[0] || b == ids[1] {
        panic!("no temp ids");
    }
    [a, b]
}
