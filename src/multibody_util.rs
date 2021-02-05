//! Module containing a few shorthand functions to do common things related to Multibodies.

use nphysics3d::joint::{Joint, RevoluteJoint};
use nphysics3d::ncollide3d::na::Isometry3;
use nphysics3d::object::{BodyPart, DefaultBodyPartHandle, MultibodyLink};
use std::option::Option;

use crate::physics::PhysicsWorld;

pub fn get_joint<J: Joint<f32>>(
    physics: &PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&J> {
    get_multibody_link(physics, part_handle)?
        .joint()
        .downcast_ref()
}

pub fn get_joint_mut<J: Joint<f32>>(
    physics: &mut PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&mut J> {
    get_multibody_link_mut(physics, part_handle)?
        .joint_mut()
        .downcast_mut()
}

pub fn get_multibody_link(
    physics: &PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&MultibodyLink<f32>> {
    physics.bodies.multibody(part_handle.0)?.link(part_handle.1)
}

pub fn get_multibody_link_mut(
    physics: &mut PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&mut MultibodyLink<f32>> {
    physics
        .bodies
        .multibody_mut(part_handle.0)?
        .link_mut(part_handle.1)
}

pub fn set_motor_speed(physics: &mut PhysicsWorld, part_handle: DefaultBodyPartHandle, speed: f32) {
    get_joint_mut::<RevoluteJoint<f32>>(physics, part_handle)
        .unwrap()
        .set_desired_angular_motor_velocity(speed);
}

/// Extract the global position of the specified multibody link.
pub fn multibody_link_position(
    physics: &PhysicsWorld,
    bph: DefaultBodyPartHandle,
) -> Option<Isometry3<f32>> {
    get_multibody_link(physics, bph).map(MultibodyLink::position)
}
