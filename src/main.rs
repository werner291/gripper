#![allow(dead_code)]

use std::boxed::Box;
use std::option::Option;
use std::result::Result::{Err, Ok};
use std::sync::mpsc::RecvTimeoutError;
use std::time::Duration;

use clap::Clap;
use na::{Isometry3, Unit, Vector3};

use control_strategies::tcp_controller::TcpController;
use graphics::Graphics;

use crate::control_strategies::gradient_descent_control::GradientDescentController;
use crate::simulator_thread::{start_physics_thread, ControllerStrategy};
use crate::spawn_utilities::{make_ground, make_pinned_ball};

mod control_strategies;
mod graphics;
mod load_mesh;
mod physics;
mod robot;
mod simulator_thread;
mod spawn_utilities;
mod sync_strategies;
mod multibody_util;

extern crate array_init;
extern crate kiss3d;
extern crate nalgebra as na;

#[derive(Clap, Debug)]
#[clap(author = "Werner Kroneman <w.kroneman@ucr.nl>")]
struct Opts {
    #[clap(
        short,
        long,
        about = "Draw a red tracing line from the tip of the end effector."
    )]
    trace: bool,

    #[clap(
        short,
        long,
        about = "If present, accepts remote control signal on specified port."
    )]
    remote_control_port: Option<u16>,
}

fn main() {
    let opts: Opts = Opts::parse();

    let mut graphics = Graphics::init();

    let mut physics = physics::PhysicsWorld::new();

    let robot = robot::make_robot(&mut physics, &mut graphics);

    make_ground(&mut physics, &mut graphics);
    let (_, ball_bh) = make_pinned_ball(&mut physics, &mut graphics);

    if opts.trace {
        println!("Tracing enabled.");
        graphics.enable_trace(robot.gripper, Isometry3::translation(0.0, 0.5, 0.0));
    }

    let tctrl: Box<dyn ControllerStrategy> = match opts.remote_control_port {
        Option::None => Box::new(GradientDescentController::new(
            ball_bh,
            Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0)),
        )),
        Option::Some(port) => {
            Box::new(TcpController::new_on_port(port).expect("Connection failed."))
        }
    };

    let (mut notifier, ws) = sync_strategies::continue_once_of_allowed();

    let (_jh, pos_updates) = start_physics_thread(robot, tctrl, ws, physics);

    let mut should_close = false;

    // control_demos::control_gripper_demo(&mut physics, &robot, t);

    // gradient_descent_control::gradient_descent_control(
    //     &mut physics, &robot, &(target), &Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0)),
    // );
    // control_flailing_demo(&mut physics, &robot, t)
    // control_robot_by_keys(&window,&mut physics,&robot);

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
