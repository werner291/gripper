use crate::physics::PhysicsWorld;
use kiss3d::scene::SceneNode;

use nalgebra::{Point3, Unit, Vector3};
use nphysics3d::ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::{
    object::BodyPartHandle, object::BodyStatus, object::ColliderDesc, object::RigidBodyDesc,
};
use std::clone::Clone;

use crate::graphics::Graphics;
use nphysics3d::object::DefaultBodyHandle;

/// Spawns a new ball with a radius of 0.3 at (4.0, 1.0, 0.0).
///
/// The ball is stuck to a vertical axis that it cannot leave:
/// it can only be moved along the Y-axis.
pub fn make_pinned_ball(
    physics: &mut PhysicsWorld,
    graphics: &mut Graphics,
) -> (SceneNode, DefaultBodyHandle) {
    const RADIUS: f32 = 0.35;

    let rb = RigidBodyDesc::new()
        .translation(Vector3::new(3.0, RADIUS, 0.0))
        .kinematic_translations(Vector3::new(true, false, true))
        .build();

    let ball = physics.bodies.insert(rb);
    physics.colliders.insert(
        ColliderDesc::new(ShapeHandle::new(Ball::new(RADIUS)))
            .density(0.5)
            .build(BodyPartHandle(ball, 0)),
    );
    let ball_sn = graphics.window.add_sphere(RADIUS);
    graphics
        .bp_to_sn
        .push((ball_sn.clone(), BodyPartHandle(ball, 0)));

    (ball_sn, ball)
}

/// Spawns a ground plane.
///
/// Note that while the visible geometry is finite, the physical actual plane extends to infinity.
pub fn make_ground(physics: &mut PhysicsWorld, graphics: &mut Graphics) {
    let ground = physics
        .bodies
        .insert(RigidBodyDesc::new().status(BodyStatus::Static).build());
    let plane_shape = Plane::new(Unit::new_unchecked(Vector3::y()));
    physics
        .colliders
        .insert(ColliderDesc::new(ShapeHandle::new(plane_shape)).build(BodyPartHandle(ground, 0)));
    let ground_quad = graphics.window.add_quad_with_vertices(
        &[
            Point3::new(-50.0, 0.0, -50.0),
            Point3::new(50.0, 0.0, -50.0),
            Point3::new(-50.0, 0.0, 50.0),
            Point3::new(50.0, 0.0, 50.0),
        ],
        2,
        2,
    );
    graphics
        .bp_to_sn
        .push((ground_quad, BodyPartHandle(ground, 0)));
}
