
extern crate gripper_experiment;

use gripper_experiment::physics::PhysicsWorld;
use gripper_experiment::graphics::Graphics;
use gripper_experiment::{spawn_utilities, robot};
use gripper_experiment::simulator_thread::{snapshot_physics, run_synced_to_graphics};
use nphysics3d::object::{FEMVolumeDesc, FEMVolume, RigidBodyDesc, ColliderDesc, BodyPartHandle, RigidBody, Body};
use nalgebra::{Point3, Vector3, Isometry3};
use std::vec::Vec;
use std::f32::consts::PI;
use std::option::Option::{None, Some};
use kiss3d::resource::Mesh;
use std::rc::Rc;
use std::cell::RefCell;
use std::borrow::BorrowMut;
use std::option::Option;
use nphysics3d::ncollide3d::shape::{ShapeHandle, Ball};
use nphysics3d::algebra::Velocity3;
use nphysics3d::joint::BallConstraint;
use std::iter::IntoIterator;


fn make_branch_mesh() {
    let branch_origin = Point3::new(-2.0, 2.0, 2.0);

    // let branch_direction = Vector3::new(1.0, 0.0, 0.0);
    //
    // let branch_points = (0..10).flat_map(|i| {
    //
    //     let ring_center = branch_origin + branch_direction * i as f32 * 0.1;
    //
    //     (0..10).map(|j| {
    //         let theta = 2.0 * PI * j as f32 / 10.0;
    //         ring_center + Vector3::new(theta.cos(), 0.0, theta.sin()) * 0.5
    //     }).collect::<Vec<_>>()
    //
    // }).collect();
}

fn main() {
    let mut p = PhysicsWorld::new();
    let mut g = Graphics::init();

    let mut fem_body = FEMVolumeDesc::cube(4, 1, 1)
        .scale(Vector3::new(10.0, 0.5, 0.5))
        .young_modulus(1.0e4)
        .poisson_ratio(0.1)
        .mass_damping(0.5)
        .build();

    // Make sure to do this *BEFORE* setting the kinematic indices.
    // See: https://github.com/dimforge/nphysics/issues/283
    let boundary_desc = fem_body.boundary_collider_desc();

    for i in 0..(fem_body.positions().len()/3) {
        if fem_body.positions()[i*3] == -5.0 {
            fem_body.set_node_kinematic(i, true);
        }
    }

    let num_tets = fem_body.num_parts();

    let fem_body_handle = p.bodies.insert(fem_body);


    let co = boundary_desc.build(fem_body_handle);
    p.colliders.insert(co);

    let bm = p.bodies.get(fem_body_handle).unwrap().downcast_ref::<FEMVolume<f32>>().unwrap().boundary_mesh().0;
    let mut mesh = Rc::new(RefCell::new(
        Mesh::new(
            bm.points().into_iter().cloned().collect(),
            bm.faces().into_iter().map(|f| Point3::new(f.indices.x as u16, f.indices.y as u16, f.indices.z as u16)).collect(),
            None,
            None,
            true)
    ));

    let apple = {
        const RADIUS: f32 = 0.35;

        let rb = RigidBodyDesc::new()
            .translation(Vector3::new(5.0, -2.0, 0.0))
            .velocity(Velocity3::linear(0.0, 0.0, 0.0))
            .build();

        let ball = p.bodies.insert(rb);
        p.colliders.insert(
            ColliderDesc::new(ShapeHandle::new(Ball::new(RADIUS)))
                .density(0.5)
                .build(BodyPartHandle(ball, 0)),
        );
        let ball_sn = g.window.add_sphere(RADIUS);
        g
            .bp_to_sn
            .push((ball_sn, BodyPartHandle(ball, 0)));

        ball
    };

    let stem = BallConstraint::new(BodyPartHandle(apple, 0),
                                         BodyPartHandle(fem_body_handle, num_tets-1),
                                         Point3::new(0.0, 1.0, 0.0),
                                         Point3::new(0.0, 0.0, 0.0));

    p.joint_constraints.insert(stem);

    let sn = g.window.add_mesh(
        mesh.clone(),
        Vector3::new(1.0, 1.0, 1.0)
    );

    g.fem_bodies.push((sn, fem_body_handle, mesh));

    let robot = robot::make_robot(&mut p, &mut g);

    run_synced_to_graphics(g, p, move |p| {

    });

    // while g.draw_frame() {
    //     g.synchronize_physics_to_graphics(&snapshot_physics(&p));
    //
    //     p.step();
    //
    //     let fem_body: &FEMVolume<f32> = p.bodies.get(fem_body_handle).unwrap().downcast_ref::<FEMVolume<f32>>().unwrap();
    //     let bm = fem_body.boundary_mesh().0;
    //     (*mesh).borrow_mut().coords().write().expect("Mesh was poisoned.").data_mut().as_mut().unwrap().splice(.., bm.points().into_iter().cloned().collect::<Vec<_>>());
    // }

}