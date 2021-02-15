use std::cell::RefCell;
use std::clone::Clone;
use std::option::Option::None;
use std::rc::Rc;

use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use nalgebra::{Point3, Translation3, Unit, Vector3};
use nphysics3d::ncollide3d::shape::{Ball, Plane, ShapeHandle};
use nphysics3d::object::{Body, DefaultBodyHandle, FEMVolume, FEMVolumeDesc};
use nphysics3d::{
    object::BodyPartHandle, object::BodyStatus, object::ColliderDesc, object::RigidBodyDesc,
};

use crate::graphics::Graphics;
use crate::physics::PhysicsWorld;

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

/// Spawns a new ball with a radius of 0.3 at (4.0, 1.0, 0.0).
///
/// The ball is stuck to a vertical axis that it cannot leave:
/// it can only be moved along the Y-axis.
pub fn make_ball(
    physics: &mut PhysicsWorld,
    graphics: &mut Graphics,
) -> (SceneNode, DefaultBodyHandle) {
    const RADIUS: f32 = 0.35;

    let rb = RigidBodyDesc::new()
        .translation(Vector3::new(3.0, RADIUS, 0.0))
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

pub fn spawn_ball(
    p: &mut PhysicsWorld,
    g: &mut Graphics,
    radius: f32,
    middle_point: Point3<f32>,
) -> (SceneNode, DefaultBodyHandle) {
    let rb = RigidBodyDesc::new()
        .translation(middle_point.coords)
        .build();

    let ball = p.bodies.insert(rb);
    p.colliders.insert(
        ColliderDesc::new(ShapeHandle::new(Ball::new(radius)))
            .density(0.5)
            .build(BodyPartHandle(ball, 0)),
    );
    let ball_sn = g.window.add_sphere(radius);
    g.bp_to_sn.push((ball_sn.clone(), BodyPartHandle(ball, 0)));

    (ball_sn, ball)
}

pub fn spawn_flexible_rod(
    p: &mut PhysicsWorld,
    g: &mut Graphics,
    origin: Point3<f32>,
) -> (SceneNode, DefaultBodyHandle) {
    let mut fem_body = FEMVolumeDesc::cube(4, 1, 1)
        .scale(Vector3::new(10.0, 0.5, 0.5))
        .translation(origin.coords + Vector3::new(5.0, 0.0, 0.0))
        .young_modulus(1.0e5)
        .poisson_ratio(0.1)
        .mass_damping(0.5)
        .build();

    // Make sure to do this *BEFORE* setting the kinematic indices.
    // See: https://github.com/dimforge/nphysics/issues/283
    let boundary_desc = fem_body.boundary_collider_desc();

    for i in 0..(fem_body.positions().len() / 3) {
        if (fem_body.positions()[i * 3] - origin.x).abs() < 1.0e-5 {
            fem_body.set_node_kinematic(i, true);
        }
    }

    let fem_body_handle = p.bodies.insert(fem_body);

    let co = boundary_desc.build(fem_body_handle);
    p.colliders.insert(co);

    let bm = p
        .bodies
        .get(fem_body_handle)
        .unwrap()
        .downcast_ref::<FEMVolume<f32>>()
        .unwrap()
        .boundary_mesh()
        .0;
    let mesh = Rc::new(RefCell::new(Mesh::new(
        bm.points().into_iter().cloned().collect(),
        bm.faces()
            .into_iter()
            .map(|f| Point3::new(f.indices.x as u16, f.indices.y as u16, f.indices.z as u16))
            .collect(),
        None,
        None,
        true,
    )));

    let sn = g.window.add_mesh(mesh.clone(), Vector3::new(1.0, 1.0, 1.0));

    g.fem_bodies.push((sn.clone(), fem_body_handle, mesh));

    (sn, fem_body_handle)
}
