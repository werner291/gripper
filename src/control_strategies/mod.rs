//! A collection of various ControllerStrategy implementations.

use std::marker::Send;
use std::prelude::v1::FnMut;

use crate::physics::PhysicsWorld;
use crate::robot::{JointVelocities, RobotBodyPartIndex};

pub mod demo_flailing;
pub mod gradient_descent_control;
pub mod keyboard_control;
pub mod tcp_controller;
pub mod to_angles;
pub mod path_controller;

/// A trait implemented by types that express a method for controlling the default robot arm.
pub trait ControllerStrategy: Send + 'static {
    fn apply_controller(
        &mut self,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities;
}

impl<F> ControllerStrategy for F
where
    F: FnMut(&PhysicsWorld, &RobotBodyPartIndex) -> JointVelocities + Send + 'static,
{
    fn apply_controller(&mut self, pw: &PhysicsWorld, rob: &RobotBodyPartIndex) -> JointVelocities {
        self(pw, rob)
    }
}
