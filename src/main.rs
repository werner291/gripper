#![allow(dead_code)]

use std::convert::From;
use std::option::Option;
use std::option::Option::Some;
use std::prelude::v1::Vec;

use clap::Clap;
use kiss3d::{light::Light, scene::SceneNode, window::Window};
use na::{Isometry3, Point3, Translation3, Vector3};
use nphysics3d::object::{BodyPartHandle, DefaultBodyPartHandle};

use graphics::Graphics;

use crate::physics::PhysicsWorld;
use crate::spawn_utilities::{make_ground, make_pinned_ball};
use crate::tcp_controller::TcpController;

mod control_demos;
mod gradient_descent_control;
mod keyboard_control;
mod load_mesh;
mod physics;
mod robot;
mod spawn_utilities;
mod tcp_controller;
mod graphics;

extern crate kiss3d;
extern crate nalgebra as na;

#[derive(Clap, Debug)]
#[clap(author = "Werner Kroneman <w.kroneman@ucr.nl>")]
struct Opts {
    #[clap(short, long, about="Draw a red tracing line from the tip of the end effector.")]
    trace: bool,

    #[clap(short, long, about="If present, accepts remote control signal on specified port.")]
    remote_control_port: Option<u16>
}

fn main() {

    let opts: Opts = Opts::parse();

    let mut graphics = Graphics::init();

    let mut physics = physics::PhysicsWorld::new();

    let robot = robot::make_robot(&mut physics, &mut graphics);

    make_ground(&mut physics, &mut graphics);
    make_pinned_ball(&mut physics, &mut graphics);

    if opts.trace {
        println!("Tracing enabled.");
        graphics.enable_trace(robot.gripper, Isometry3::translation(0.0,0.5,0.0));
    }

    let mut tctrl = opts.remote_control_port.map(|port| {
        TcpController::new_on_port(11235).expect("Cannot establish TCP socket.")
    });

    let mut should_close = false;

    while ! should_close {

        should_close |= !graphics.draw_frame(&physics);

        physics.step();

        if let Some(rc) = &mut tctrl {
            rc.control_cycle_synchronous(&mut physics, &robot);
        }

        // control_demos::control_gripper_demo(&mut physics, &robot, t);

        // gradient_descent_control::gradient_descent_control(
        //     &mut physics, &robot, &(target), &Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0)),
        // );
        // control_flailing_demo(&mut physics, &robot, t)
        // control_robot_by_keys(&window,&mut physics,&robot);
    }
}
