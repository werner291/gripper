#![feature(array_map)]
#![allow(dead_code)]

use std::convert::From;
use std::iter::Iterator;
use std::option::Option::Some;
use std::prelude::v1::Vec;

use kiss3d::light::Light;
use kiss3d::resource::{MaterialManager, Mesh, TextureManager};
use kiss3d::scene::{Object, SceneNode};
use kiss3d::window::Window;
use na::{Isometry3, Point3, Translation3, Unit, Vector3};
use nphysics3d::object::{
    BodyPart, BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodyHandle, DefaultBodyPartHandle,
    Multibody, MultibodyDesc, MultibodyLink, RigidBodyDesc,
};
use rand::Rng;
use stl_io::{IndexedTriangle, Triangle, Vertex};

use crate::keyboard_control::control_robot_by_keys;
use crate::physics::PhysicsWorld;
use crate::spawn_utilities::{make_ground, make_pinned_ball};

mod control_demos;
mod gradient_descent_control;
mod keyboard_control;
mod load_mesh;
mod physics;
mod robot;
mod spawn_utilities;

extern crate kiss3d;
extern crate nalgebra as na;

fn main() {
    let mut window = Window::new("Kiss3d: cube");

    window.set_framerate_limit(Some(60));
    let mut physics = physics::PhysicsWorld::new();

    let mut bp_to_sn = Vec::new();

    let robot = robot::make_robot(&mut physics, window.scene_mut(), &mut bp_to_sn);

    window.set_light(Light::StickToCamera);

    make_ground(&mut window, &mut physics, &mut bp_to_sn);
    let (_, BodyPartHandle(ball, _)) = make_pinned_ball(&mut window, &mut physics, &mut bp_to_sn);

    let mut t = 0.0;

    let mut marker = window.add_sphere(0.1);

    let mut trace = Vec::new();

    let mut frame = 0;

    while window.render() {
        let ball_pos = Point3::from(
            physics
                .bodies
                .rigid_body(ball)
                .unwrap()
                .position()
                .translation
                .vector,
        );
        let target = &ball_pos + Vector3::new(0.0, 0.0, 0.0);
        marker.set_local_translation(Translation3::from(target - Point3::new(0.0, 0.0, 0.0)));
        marker.set_color(1.0, 0.0, 0.0);

        t += physics.mechanical_world.timestep();

        physics.step();

        frame += 1;
        if frame % 10 == 0 {
            trace.push(gradient_descent_control::point_inside_gripper(
                &physics, &robot,
            ));
        }

        if trace.len() >= 2 {
            for i in 0..trace.len() - 1 {
                window.draw_line(&trace[i], &trace[i + 1], &Point3::new(1.0, 0.0, 0.0));
            }
        }

        synchronize_physics_to_graphics(&mut physics, &mut bp_to_sn);

        control_demos::control_gripper_demo(&mut physics, &robot, t);

        gradient_descent_control::gradient_descent_control(
            &mut physics,
            &robot,
            &(target),
            &Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0)),
        );
        // control_flailing_demo(&mut physics, &robot, t)
        // control_robot_by_keys(&window,&mut physics,&robot);
    }
}

fn synchronize_physics_to_graphics(
    physics: &mut PhysicsWorld,
    bp_to_sn: &mut Vec<(SceneNode, DefaultBodyPartHandle)>,
) {
    for (sn, BodyPartHandle(bh, ph)) in bp_to_sn.iter_mut() {
        let pos: Isometry3<f32> = physics
            .bodies
            .get(*bh)
            .unwrap()
            .part(*ph)
            .unwrap()
            .position();

        sn.set_local_transformation(pos);
    }
}
