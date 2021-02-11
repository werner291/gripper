use std::boxed::Box;
use std::collections::HashMap;
use std::iter::Iterator;
use std::result::Result::{Err, Ok};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError};
use std::thread;
use std::thread::JoinHandle;
use std::time::Duration;

use nalgebra::geometry::Isometry3;
use nalgebra::Point3;
use nphysics3d::object::{BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle, FEMVolume};
use nphysics3d::object::Body;

use crate::control_strategies::ControllerStrategy;
use crate::graphics::Graphics;
use crate::multibody_util::set_motor_speed;
use crate::physics::PhysicsWorld;
use crate::robot::{JointVelocities, RobotBodyPartIndex};
use crate::sync_strategies;
use crate::sync_strategies::WaitStrategy;
use std::prelude::v1::Vec;
use std::option::Option::Some;

/// Message sent from physics thread about the state of the world.
pub struct PhysicsUpdate {
    pub positions: HashMap<DefaultBodyPartHandle, Isometry3<f32>>,
    pub fvm_points: HashMap<DefaultBodyHandle, Vec<Point3<f32>>>,
}

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
pub fn snapshot_physics(physics: &PhysicsWorld) -> PhysicsUpdate {

    let mut pu = PhysicsUpdate {
        positions: HashMap::new(),
        fvm_points: HashMap::new(),
    };

    for (bh,body) in physics.bodies.iter() {

        if let Some(fem_body) = body.downcast_ref::<FEMVolume<f32>>() {
            let bm = fem_body.boundary_mesh().0;
            let points = bm.points().into_iter().cloned().collect::<Vec<_>>();
            pu.fvm_points.insert(bh, points);
        }

        for i in 0..body.num_parts() {
            // register the BodyPartHandle and world position,
            let bph = BodyPartHandle(bh, i);
            let pos = body.part(i).unwrap().position();
            pu.positions.insert(bph, pos);
        }


    }

    pu

}

pub fn run_synced_to_graphics(graphics: &mut Graphics, physics: PhysicsWorld, robot: RobotBodyPartIndex, tctrl: Box<dyn ControllerStrategy>) {
    let (mut notifier, ws) = sync_strategies::continue_once_of_allowed();

    let (_jh, pos_updates) = start_physics_thread(robot, tctrl, ws, physics);

    let mut should_close = false;

    while !should_close {
        notifier();
        should_close |= !graphics.draw_frame();

        match pos_updates.recv_timeout(Duration::from_millis(100)) {
            Ok(positions) => {
                graphics.synchronize_physics_to_graphics(&positions);
            }
            Err(RecvTimeoutError::Timeout) => {
                println!("Simulation thread taking more than 100ms last timestep.")
            }
            Err(RecvTimeoutError::Disconnected) => panic!("Physics thread possibly crashed."),
        }
    }
}
