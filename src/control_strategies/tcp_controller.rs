use std::io::{Error, Result, Write};
use std::net::{TcpListener, TcpStream};
use std::option::Option;
use std::option::Option::{None, Some};
use std::result::Result::Ok;

use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use nphysics3d::joint::RevoluteJoint;

use crate::control_strategies::ControllerStrategy;
use crate::multibody_util::get_joint;
use crate::physics::PhysicsWorld;
use crate::robot::joint_map::JointVelocities;
use crate::robot::RobotBodyPartIndex;

pub struct TcpController {
    listener: TcpListener,
    current_stream: Option<TcpStream>,
    frame_counter: usize,
}

impl TcpController {
    pub fn new_on_port(port: u16) -> Result<TcpController> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;

        println!("Listening on port {}", port);

        Ok(TcpController {
            current_stream: None,
            listener,
            frame_counter: 0,
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
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> std::result::Result<JointVelocities, Error> {
        self.send_joint_angles(physics, robot)
            .and(self.receive_joint_angles())
    }

    pub fn receive_joint_angles(&mut self) -> Result<JointVelocities> {
        let current_stream = self.current_stream()?;

        Ok(JointVelocities {
            swivel: current_stream.read_f32::<BigEndian>()?,
            link1: current_stream.read_f32::<BigEndian>()?,
            link2: current_stream.read_f32::<BigEndian>()?,
            gripper: current_stream.read_f32::<BigEndian>()?,
            finger_0: current_stream.read_f32::<BigEndian>()?,
            finger_1: current_stream.read_f32::<BigEndian>()?,
            finger_2: current_stream.read_f32::<BigEndian>()?,
            finger_0_2: current_stream.read_f32::<BigEndian>()?,
            finger_1_2: current_stream.read_f32::<BigEndian>()?,
            finger_2_2: current_stream.read_f32::<BigEndian>()?,
        })
    }

    pub fn send_joint_angles(
        &mut self,
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> Result<()> {
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
    fn apply_controller(
        &mut self,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities {
        self.control_cycle_synchronous(physics_world, robot)
            // FIXME errors shouldn't get up to this point...
            .expect("Network error while attempting to run robot controller.")
    }
}
