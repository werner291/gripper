#![allow(dead_code)]

use std::boxed::Box;
use std::option::Option;

use clap::Clap;
use na::Isometry3;

use control_strategies::ControllerStrategy;
use control_strategies::tcp_controller::TcpController;
use graphics::Graphics;

use crate::control_strategies::gradient_descent_control::GradientDescentController;
use crate::spawn_utilities::{make_ground, make_pinned_ball};

mod control_strategies;
mod graphics;
mod load_mesh;
mod multibody_util;
mod physics;
mod robot;
mod simulator_thread;
mod spawn_utilities;
mod sync_strategies;
mod kinematics;
mod simulation;

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
        )),
        Option::Some(port) => {
            Box::new(TcpController::new_on_port(port).expect("Connection failed."))
        }
    };


    simulation::run_synced_to_graphics(&mut graphics, physics, robot, tctrl)
}
