use core::marker::Send;
use core::ops::FnMut;
use std::boxed::Box;
use std::collections::HashMap;
use std::iter::Iterator;
use std::sync::mpsc::{channel, Receiver};
use std::thread;
use std::thread::JoinHandle;

use nalgebra::geometry::Isometry3;
use nphysics3d::object::Body;
use nphysics3d::object::{BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle};

use crate::multibody_util::set_motor_speed;
use crate::physics::PhysicsWorld;
use crate::robot::{JointVelocities, RobotBodyPartIndex};
use crate::sync_strategies::WaitStrategy;

/// A callback that implements logic to control the robot's motors.
/// TODO Too much mutability for my liking, better make a function returning motor speeds.
pub trait ControllerStrategy: Send + 'static {
    fn apply_controller(
        &mut self,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities;
}

impl<F> ControllerStrategy for F
where
    F: FnMut(&PhysicsWorld, &RobotBodyPartIndex) -> JointVelocities + Send + 'static,
{
    fn apply_controller(&mut self, pw: &PhysicsWorld, rob: &RobotBodyPartIndex) -> JointVelocities {
        self(pw, rob)
    }
}

/// Message sent from physics thread about the state of the world.
pub type PhysicsUpdate = HashMap<DefaultBodyPartHandle, Isometry3<f32>>;

/// Run the "simulation" part of the simulator app, independently of the graphics thread.
/// Returns the JoinHandle of the thread, as well a Receiver, which provides the position
/// of every body part in the simulation at every frame.
///
/// That way, the simulation can run at whatever pace makes sense, depending on the scenario.
///
/// For instance, the provided WaitStrategy can simply hold the simulation until the graphics
/// thread has completed drawing a frame, keeping both roughly in sync, but allowing the simulation
/// to take longer if it needs to.
///
/// The provided controller can also take however much time it needs, making sure that it is run
/// exactly once every time step of the simulation.
///
/// # Arguments
///
/// * `robot` - A RobotBodyPartIndex corresponding to a robot somewhere in the simulation.
/// * `controller` - A callback meant to hold the logic that controls the target motor speeds of the robot.
/// * `wait_strategy` - A method that should hold the calling thread until the next frame of the simulation should be computed.
/// * `physics` - The PhysicsWorld, initialized with whatever needs to be present in the simulation.
///
/// TODO: Maybe pass the Sender in as a parameter instead?
pub fn start_physics_thread<W>(
    robot: RobotBodyPartIndex,
    mut controller: Box<dyn ControllerStrategy>, // I hate that this is stateful...
    mut wait_strategy: W,
    mut physics: PhysicsWorld,
) -> (JoinHandle<()>, Receiver<PhysicsUpdate>)
where
    W: WaitStrategy,
{
    // Create achannel for updates about positions.
    let (snd, rcv) = channel();

    // Spawn the simulation main loop thread and move necessary valies into it.
    let join = thread::spawn(move || {
        loop {
            // Apply the waiting strategy, e.g. to synchronize with the graphics thread without blocking it.
            wait_strategy();

            // Apply a timestep in the physics engine.
            physics.step();

            let speeds = controller.apply_controller(&mut physics, &robot);

            apply_motor_speeds(&robot, &mut physics, speeds);

            // Update any interested parties in the positions of the various body parts.
            snd.send(snapshot_physics(&physics)).unwrap()
        }
    });

    (join, rcv)
}

pub fn apply_motor_speeds(
    robot: &RobotBodyPartIndex,
    mut physics: &mut PhysicsWorld,
    speeds: JointVelocities,
) {
    set_motor_speed(&mut physics, robot.swivel, speeds.swivel);
    set_motor_speed(&mut physics, robot.link1, speeds.link1);
    set_motor_speed(&mut physics, robot.link2, speeds.link2);
    set_motor_speed(&mut physics, robot.gripper, speeds.gripper);
    set_motor_speed(&mut physics, robot.finger_0, speeds.finger_0);
    set_motor_speed(&mut physics, robot.finger_1, speeds.finger_1);
    set_motor_speed(&mut physics, robot.finger_2, speeds.finger_2);
    set_motor_speed(&mut physics, robot.finger_0_2, speeds.finger_0_2);
    set_motor_speed(&mut physics, robot.finger_1_2, speeds.finger_1_2);
    set_motor_speed(&mut physics, robot.finger_2_2, speeds.finger_2_2);
}

/// Take a snapshot of the Physics engine that can be safely sent to the graphics thread.
fn snapshot_physics(physics: &PhysicsWorld) -> PhysicsUpdate {
    // For every body,
    physics
        .bodies
        .iter()
        .flat_map(|(bh, body): (DefaultBodyHandle, &dyn Body<f32>)| {
            // for every body part,
            (0..body.num_parts()).map(move |i| {
                // register the BodyPartHandle and world position,
                let bph = BodyPartHandle(bh, i);
                let pos = body.part(i).unwrap().position().clone();
                (bph, pos)
            })
        })
        .collect() // then put them all in a HashMap for easy lookup.
}
