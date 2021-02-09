use std::convert::Into;
use std::iter::{IntoIterator, Iterator};
use std::vec::Vec;

use nalgebra::Vector4;

use crate::control_strategies::ControllerStrategy;
use crate::physics::PhysicsWorld;
use crate::robot::{arm_joint_angles, ArmJointMap, FINGERS_OPEN, JointVelocities, RobotBodyPartIndex};
use splines::{Spline, Interpolation, Key};

/// A controller that attempts to follow a pre-defined motion plan.
pub struct PathController {
    time: f32,
    motion_plan: Spline<f32, Vector4<f32>>
}

impl PathController {
    pub fn new(motion_plan: Vec<(f32, ArmJointMap<f32>)>) -> PathController {
        PathController{
            time: 0.0,
            motion_plan: Spline::from_iter(motion_plan.into_iter()
                .map(|(t,jm)| Key::new(t,jm.into(), Interpolation::Cosine)))

        }
    }
}

impl ControllerStrategy for PathController {

    fn apply_controller(
        &mut self,
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities {

        self.time += physics.mechanical_world.timestep();


        let joint_angles: Vector4<f32> = arm_joint_angles(physics, robot).into();

        let target_angles: Vector4<f32> = self.motion_plan.clamped_sample(self.time).expect("Empty path.");

        let delta = joint_angles - target_angles;

        JointVelocities::from_arm_and_finger(delta.into(), FINGERS_OPEN)
    }
}


