use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nalgebra::{Vector3, Unit, Point3};
use nphysics3d::ncollide3d::shape::{
        Ball,
        Plane,
        ShapeHandle
    };
use nphysics3d::{
    object::BodyStatus,
    object::{BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle},
    object::ColliderDesc,
    object::RigidBodyDesc
};
use crate::physics::PhysicsWorld;
use std::prelude::v1::Vec;
use std::clone::Clone;

/// Spawns a new ball with a radius of 0.3 at (4.0, 1.0, 0.0).
///
/// The ball is stuck to a vertical axis that it cannot leave:
/// it can only be moved along the Y-axis.
pub fn make_pinned_ball(window: &mut Window, physics: &mut PhysicsWorld, bp_to_sn: &mut Vec<(SceneNode, DefaultBodyPartHandle)>) -> (SceneNode, DefaultBodyPartHandle) {
    const RADIUS: f32 = 0.3;

    let rb = RigidBodyDesc::new()
        .translation(Vector3::new(4.0, 1.0, 0.0))
        .kinematic_translations(Vector3::new(true, false, true))
        .build();

    let ball = physics.bodies.insert(rb);
    physics.colliders.insert(ColliderDesc::new(ShapeHandle::new(Ball::new(RADIUS))).density(0.5).build(BodyPartHandle(ball, 0)));
    let ball_sn = window.add_sphere(RADIUS);
    bp_to_sn.push((ball_sn.clone(), BodyPartHandle(ball, 0)));

    (ball_sn, BodyPartHandle(ball, 0))
}

/// Spawns a ground plane.
///
/// Note that while the visible geometry is finite, the physical actual plane extends to infinity.
pub fn make_ground(window: &mut Window, physics: &mut PhysicsWorld, bp_to_sn: &mut Vec<(SceneNode, DefaultBodyPartHandle)>) {
    let ground = physics.bodies.insert(RigidBodyDesc::new().status(BodyStatus::Static).build());
    let plane_shape = Plane::new(Unit::new_unchecked(Vector3::y()));
    physics.colliders.insert(ColliderDesc::new(ShapeHandle::new(plane_shape)).build(BodyPartHandle(ground, 0)));
    let ground_quad = window.add_quad_with_vertices(
        &[
            Point3::new(-50.0, 0.0, -50.0),
            Point3::new(50.0, 0.0, -50.0),
            Point3::new(-50.0, 0.0, 50.0),
            Point3::new(50.0, 0.0, 50.0),
        ],
        2, 2
    );
    bp_to_sn.push((ground_quad, BodyPartHandle(ground, 0)));
}
