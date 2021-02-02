#![allow(dead_code)]

use std::option::Option;
use std::option::Option::Some;

use clap::Clap;

use na::Isometry3;

use graphics::Graphics;

use crate::spawn_utilities::{make_ground, make_pinned_ball};
use crate::tcp_controller::TcpController;
use std::thread;
use std::sync::{Mutex, Arc, Condvar};
use crate::physics::PhysicsWorld;
use crate::robot::RobotBodyPartIndex;
use std::thread::JoinHandle;
use std::ops::{FnMut, Fn};
use std::marker::Send;
use std::clone::Clone;
use std::sync::mpsc::{Receiver, channel, RecvTimeoutError};
use std::collections::HashMap;
use nphysics3d::object::{DefaultBodyPartHandle, DefaultBodyHandle, Body, BodyPartHandle};
use std::iter::Iterator;
use std::time::{Duration, Instant};
use std::result::Result::{Ok, Err};

mod control_demos;
mod gradient_descent_control;
mod graphics;
mod keyboard_control;
mod load_mesh;
mod physics;
mod robot;
mod spawn_utilities;
mod tcp_controller;
mod sync_strategies;

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
        graphics.enable_trace(robot.gripper, Isometry3::translation(0.0, 0.5, 0.0));
    }

    let mut tctrl = opts
        .remote_control_port
        .map(|port| TcpController::new_on_port(port).expect("Connection failed."));

    let controller = move |pw : &mut PhysicsWorld, rb : &RobotBodyPartIndex| {
        if let Some(rc) = &mut tctrl {
            rc.control_cycle_synchronous(pw, rb).unwrap();
        }
    };

    let (mut notifier, ws) = sync_strategies::continue_once_of_allowed();

    // let physics_mtx = Arc::new(Mutex::new(physics));

    let (jh, pos_updates) = start_physics_thread(robot, controller, ws, physics);

    let mut should_close = false;

    // control_demos::control_gripper_demo(&mut physics, &robot, t);

    // gradient_descent_control::gradient_descent_control(
    //     &mut physics, &robot, &(target), &Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0)),
    // );
    // control_flailing_demo(&mut physics, &robot, t)
    // control_robot_by_keys(&window,&mut physics,&robot);

    let mut last_physics_update = Instant::now();

    while !should_close {

        notifier();

        match pos_updates.recv_timeout(Duration::from_millis(30)) {
            Ok(positions) => {
                last_physics_update = Instant::now();
                graphics.synchronize_physics_to_graphics(&positions);
            },
            Err(RecvTimeoutError::Timeout) => println!("Timeout {}", graphics.frames_drawn),
            Err(RecvTimeoutError::Disconnected) => panic!("Physics thread possibly crashed.")
        }

        should_close |= !graphics.draw_frame();



    }
}

fn start_physics_thread<C, W>(robot: RobotBodyPartIndex,
                        mut controller: C,
                        mut wait_strategy: W,
                        mut physics: PhysicsWorld) -> (JoinHandle<()>, Receiver<HashMap<DefaultBodyPartHandle, Isometry3<f32>>>)
    where C : FnMut(&mut PhysicsWorld, &RobotBodyPartIndex) + Send + 'static,
          W : FnMut() + Send + 'static {

    let (snd, rcv) = channel();

    let join = thread::spawn(move || {
        loop {
            wait_strategy();

            physics.step();

            controller(&mut physics, &robot);

            snd.send(
                physics.bodies.iter().flat_map(|(bh, body) : (DefaultBodyHandle, &dyn Body<f32>)| {
                    (0..body.num_parts()).map(move |i| {
                        let bph = BodyPartHandle(bh, i);
                        let pos = body.part(i).unwrap().position().clone();
                        (bph, pos)
                    })
                }).collect()
            ).unwrap()
        }
    });

    (join, rcv)
}