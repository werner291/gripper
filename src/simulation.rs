use std::boxed::Box;
use std::result::Result::{Err, Ok};
use std::sync::mpsc::RecvTimeoutError;
use std::time::Duration;

use crate::graphics::Graphics;
use crate::physics::PhysicsWorld;
use crate::robot::RobotBodyPartIndex;
use crate::simulator_thread::{ControllerStrategy, start_physics_thread};
use crate::sync_strategies;

pub fn run_synced_to_graphics(graphics: &mut Graphics, mut physics: PhysicsWorld, robot: RobotBodyPartIndex, tctrl: Box<dyn ControllerStrategy>) {
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
