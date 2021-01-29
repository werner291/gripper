use crate::physics::PhysicsWorld;
use crate::robot::{get_joint, set_motor_speed, RobotBodyPartIndex};
use byteorder::{BigEndian, ByteOrder, ReadBytesExt, WriteBytesExt};
use nphysics3d::joint::RevoluteJoint;
use std::io::{Error, Result, Write};

use std::net::{TcpListener, TcpStream};
use std::result::Result::Ok;

pub struct TcpController {
    listener: TcpListener,
    current_stream: TcpStream,
    frame_counter: usize,
}

impl TcpController {
    pub fn new_wait_until_connected(port: u16) -> Result<TcpController> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        // let listener = TcpListener::bind(("localhost", port))?;

        println!("Listening on port {}", port);

        let (current_stream, remote_addr) = listener.accept()?;

        println!("Received connection from {}", remote_addr);

        Ok(TcpController {
            current_stream,
            listener,
            frame_counter: 0,
        })
    }

    pub fn control_cycle_synchronous(
        &mut self,
        physics: &mut PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> std::result::Result<(), Error> {
        self.send_joint_angles(physics, robot)
            .and(self.receive_joint_angles(physics, robot))
    }

    pub fn receive_joint_angles(
        &mut self,
        physics: &mut PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> Result<()> {
        for x in robot.motor_parts().iter() {
            let v = self.current_stream.read_f32::<BigEndian>()?;

            set_motor_speed(physics, *x, v);
        }
        Ok(())
    }

    pub fn send_joint_angles(
        &mut self,
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> Result<()> {
        self.current_stream
            .write_u64::<BigEndian>(self.frame_counter as u64)?;
        self.frame_counter += 1;

        for x in robot.motor_parts().iter() {
            let joint = get_joint::<RevoluteJoint<f32>>(physics, *x).unwrap();
            let angle = joint.angle();

            self.current_stream.write_f32::<BigEndian>(angle)?;
        }
        self.current_stream.flush()
    }
}
