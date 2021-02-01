use crate::physics::PhysicsWorld;
use crate::robot::{get_joint, set_motor_speed, RobotBodyPartIndex};
use byteorder::{BigEndian, ByteOrder, ReadBytesExt, WriteBytesExt};
use nphysics3d::joint::RevoluteJoint;
use std::io::{Result, Write};

use std::iter::Iterator;
use std::net::{TcpListener, TcpStream};
use std::result::Result::{Err, Ok};
use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use std::thread;
use std::vec::Vec;

pub struct TcpController {
    tx: Sender<JointAngles>,
    rx: Receiver<JointVelocities>,
    frame_counter: u64,
}

struct JointAngles {
    frame: u64,
    angles: Vec<f32>,
}

struct JointVelocities {
    velocities: Vec<f32>,
}

impl TcpController {
    pub fn new_on_port(port: u16) -> Result<TcpController> {
        let (send_ja, rcv_ja) = mpsc::channel::<JointAngles>();
        let (send_jv, rcv_jv) = mpsc::channel::<JointVelocities>();

        thread::spawn(move || TcpController::run_tx_rx_thread(port, rcv_ja, send_jv));

        Ok(TcpController {
            tx: send_ja,
            rx: rcv_jv,
            frame_counter: 0,
        })
    }

    fn run_tx_rx_thread(
        port: u16,
        rcv_ja: Receiver<JointAngles>,
        send_jv: Sender<JointVelocities>,
    ) {
        let listener = TcpListener::bind(("0.0.0.0", port)).expect("Cannot bind to port.");

        println!("Listening on port {}", port);

        for inc in listener.incoming() {
            match inc {
                Ok(mut current_stream) => {
                    println!("Received remote control connection.");

                    loop {
                        // FIXME too much .unwrap() for my taste...
                        TcpController::write_joint_angles(
                            &mut current_stream,
                            rcv_ja.recv().unwrap(),
                        )
                        .unwrap();
                        send_jv
                            .send(TcpController::read_joint_angles(&mut current_stream).unwrap())
                            .unwrap();
                    }
                }
                Err(e) => {
                    println!("Unexpected connection error: {}", e);
                }
            }
        }
    }

    fn read_joint_angles(current_stream: &mut TcpStream) -> Result<JointVelocities> {
        let mut velocities = Vec::new();

        for _ in 0..10 {
            velocities.push(current_stream.read_f32::<BigEndian>()?);
        }

        Ok(JointVelocities { velocities })
    }

    fn write_joint_angles(current_stream: &mut TcpStream, ja: JointAngles) -> Result<()> {
        current_stream.write_u64::<BigEndian>(ja.frame as u64)?;

        for x in ja.angles.iter() {
            current_stream.write_f32::<BigEndian>(*x)?;
        }

        current_stream.flush()
    }

    pub fn control_cycle_synchronous(
        &mut self,
        physics: &mut PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) {
        self.send_joint_angles(physics, robot);
        self.receive_joint_velocities(physics, robot);
    }

    pub fn receive_joint_velocities(
        &mut self,
        physics: &mut PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) {
        let ja = self.rx.recv().unwrap();

        for (x, v) in robot.motor_parts().iter().zip(ja.velocities.into_iter()) {
            set_motor_speed(physics, *x, v);
        }
    }

    pub fn send_joint_angles(&mut self, physics: &PhysicsWorld, robot: &RobotBodyPartIndex) {
        let angles = robot
            .motor_parts()
            .iter()
            .map(|bph| {
                let joint = get_joint::<RevoluteJoint<f32>>(physics, *bph).unwrap();
                joint.angle()
            })
            .collect();

        self.tx
            .send(JointAngles {
                frame: self.frame_counter,
                angles,
            })
            .expect("Sending joint angles to network thread failed.");

        self.frame_counter += 1;
    }
}
