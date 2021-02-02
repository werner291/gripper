use std::net::{TcpListener, SocketAddr, Ipv6Addr, TcpStream};
use std::sync::mpsc;
use crate::physics::{PhysicsWorld, ControllerStrategy};
use crate::robot::{RobotBodyPartIndex, get_joint_mut, get_joint, set_motor_speed};
use std::result::Result::{Ok, Err};
use std::io::{Error, Write, Result};
use nphysics3d::joint::RevoluteJoint;
use byteorder::{ReadBytesExt, WriteBytesExt, ByteOrder, BigEndian};
use std::option::Option::{None, Some};
use std::option::Option;

pub struct TcpController {
    listener: TcpListener,
    current_stream: Option<TcpStream>,
    frame_counter: usize
}

impl TcpController {

    pub fn new_on_port(port: u16) -> Result<TcpController> {

        let listener = TcpListener::bind(("0.0.0.0", port))?;

        println!("Listening on port {}", port);

        Ok(TcpController {
            current_stream: None,
            listener,
            frame_counter: 0
        })

    }

    fn current_stream(&mut self) -> Result<&mut TcpStream> {
        if self.current_stream.is_none() {
            let (current_stream, remote_addr) = self.listener.accept()?;

            println!("Received connection from {}", remote_addr);

            self.current_stream = Some(current_stream);
        }
        Ok(self.current_stream.as_mut().unwrap())
    }

    pub fn control_cycle_synchronous(
        &mut self,
        mut physics: &mut PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> std::result::Result<(), Error> {
        self.send_joint_angles(physics, robot)
            .and(self.receive_joint_angles(physics, robot))

    }

    pub fn receive_joint_angles(&mut self, mut physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex) -> Result<()> {

        let current_stream = self.current_stream()?;

        for x in robot.motor_parts().iter() {

            let v = current_stream.read_f32::<BigEndian>()?;

            set_motor_speed(physics, *x, v);
        }
        Ok(())
    }

    pub fn send_joint_angles(&mut self, physics: &PhysicsWorld, robot: &RobotBodyPartIndex) -> Result<()> {

        let frm = self.frame_counter;
        self.frame_counter += 1;

        let current_stream = self.current_stream()?;

        current_stream.write_u64::<BigEndian>(frm as u64)?;

        for x in robot.motor_parts().iter() {
            let joint = get_joint::<RevoluteJoint<f32>>(physics, *x).unwrap();
            let angle = joint.angle();

            current_stream.write_f32::<BigEndian>(angle)?;
        }
        current_stream.flush()
    }
}

impl ControllerStrategy for TcpController {
    fn apply_controller(&mut self, physics_world: &mut PhysicsWorld, robot: &RobotBodyPartIndex) {
        self.control_cycle_synchronous(physics_world, robot)
            // FIXME errors shouldn't get up to this point...
            .expect("Network error while attempting to run robot controller.");
    }
}