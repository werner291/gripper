use std::collections::HashMap;
use std::iter::Iterator;
use std::marker::Send;
use std::ops::FnMut;
use std::option::Option::Some;
use std::prelude::v1::Vec;
use std::result::Result::{Err, Ok};
use std::sync::mpsc::{channel, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use nalgebra::geometry::Isometry3;
use nalgebra::Point3;
use nphysics3d::object::{BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle, FEMVolume};

use crate::graphics::Graphics;
use crate::multibody_util::set_motor_speed;
use crate::physics::PhysicsWorld;
use crate::robot::joint_map::JointVelocities;
use crate::robot::RobotBodyPartIndex;
use crate::sync_strategies;

/// Message sent from physics thread about the state of the world.
pub struct PhysicsUpdate {
    pub positions: HashMap<DefaultBodyPartHandle, Isometry3<f32>>,
    pub fvm_points: HashMap<DefaultBodyHandle, Vec<Point3<f32>>>,
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

    for (bh, body) in physics.bodies.iter() {
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

pub fn run_synced_to_graphics<Cb>(
    mut graphics: Graphics,
    mut physics_world: PhysicsWorld,
    mut callback: Cb,
) where
    Cb: FnMut(&mut PhysicsWorld) + Send + 'static,
{
    let (mut notifier, ws) = sync_strategies::continue_once_of_allowed();

    let should_stop = Arc::new(Mutex::new(false));
    let should_stop_clone: Arc<Mutex<bool>> = should_stop.clone();

    let mut wait_strategy = ws;
    let (snd, pos_updates) = channel();
    let join = thread::spawn(move || {
        while !(*(*should_stop_clone)
            .lock()
            .expect("Should-stop mutex poisoned."))
        {
            // Apply the waiting strategy, e.g. to synchronize with the graphics thread without blocking it.
            wait_strategy();

            callback(&mut physics_world);

            // Apply a timestep in the physics engine.
            physics_world.step();

            // Update any interested parties in the positions of the various body parts.
            snd.send(snapshot_physics(&physics_world)).unwrap()
        }
    });

    while !(*(*should_stop).lock().expect("Should-stop mutex poisoned.")) {
        notifier();
        (*(*should_stop).lock().expect("Should-stop mutex poisoned.")) |= !graphics.draw_frame();

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

    notifier(); // Wake up the physics thread so that it can be terminated.
    join.join().expect("Physics thread join failed.")
}
