//! This mod contains the "standard robot arm": a simulated 4-DoF
//! robotic arm with a three-fingered gripper end-effector.
//!
//! TODO: At some point, we may want to make this more generic, maybe based on URDF?

use std::clone::Clone;
use std::iter::Iterator;
use std::option::Option;
use std::result::Result::{Err, Ok};

use kiss3d::ncollide3d::procedural::TriMesh;
use kiss3d::ncollide3d::shape::{ConvexHull, ShapeHandle};
use kiss3d::resource::{MaterialManager, Mesh, TextureManager};
use kiss3d::scene::{Object, SceneNode};
use na::{Isometry3, Rotation3, Unit, Vector3};
use nphysics3d::joint::{FixedJoint, RevoluteJoint};
use nphysics3d::nalgebra::{RealField, Vector4};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodyPartHandle, DefaultColliderHandle,
    MultibodyDesc,
};

use joint_map::{ArmJointMap, FingerJointMap, JointMap};

use crate::{load_mesh, multibody_util};
use crate::graphics::Graphics;
use crate::kinematics::KinematicModel;
use crate::multibody_util::get_joint;
use crate::physics::PhysicsWorld;

pub mod joint_map;
pub mod spawn;

pub fn arm_joint_angles(physics: &PhysicsWorld, robot: &RobotBodyPartIndex) -> ArmJointMap<f32> {
    ArmJointMap {
        swivel: get_joint::<RevoluteJoint<f32>>(physics, robot.swivel)
            .unwrap()
            .angle(),
        link1: get_joint::<RevoluteJoint<f32>>(physics, robot.link1)
            .unwrap()
            .angle(),
        link2: get_joint::<RevoluteJoint<f32>>(physics, robot.link2)
            .unwrap()
            .angle(),
        gripper: get_joint::<RevoluteJoint<f32>>(physics, robot.gripper)
            .unwrap()
            .angle(),
    }
}

/// A struct that contains the body handle and body part handle of the various parts of a robot.
/// Note that each body part also has a name, should that be more convenient.
#[derive(Debug)]
pub struct RobotBodyPartIndex {
    pub body: DefaultBodyHandle,
    pub base: DefaultBodyPartHandle,
    pub swivel: DefaultBodyPartHandle,
    pub link1: DefaultBodyPartHandle,
    pub link2: DefaultBodyPartHandle,

    pub gripper: DefaultBodyPartHandle,
    pub gripper_collider: DefaultColliderHandle,

    pub finger_0: DefaultBodyPartHandle,
    pub finger_0_collider: DefaultColliderHandle,
    pub finger_1: DefaultBodyPartHandle,
    pub finger_1_collider: DefaultColliderHandle,
    pub finger_2: DefaultBodyPartHandle,

    pub finger_2_collider: DefaultColliderHandle,
    pub finger_0_2: DefaultBodyPartHandle,
    pub finger_0_2_collider: DefaultColliderHandle,
    pub finger_1_2: DefaultBodyPartHandle,
    pub finger_1_2_collider: DefaultColliderHandle,
    pub finger_2_2: DefaultBodyPartHandle,
    pub finger_2_2_collider: DefaultColliderHandle,
}

impl RobotBodyPartIndex {
    /// Provides an array of body part handles that correspond to parts
    /// that have a motorized revolute joint.
    ///
    /// Also provides a canonical mapping for motors to numerical channels.
    pub fn motor_parts(&self) -> [DefaultBodyPartHandle; 10] {
        [
            self.swivel,
            self.link1,
            self.link2,
            self.gripper,
            self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_0_2,
        ]
    }

    /// Array of motors that control the robot's fingers.
    ///
    /// Setting a positive speed on these opens the gripper, a negative speed closes them.
    pub fn finger_parts(&self) -> [DefaultBodyPartHandle; 6] {
        [
            self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_2_2,
        ]
    }

    pub fn gripper_colliders(&self) -> [DefaultColliderHandle; 7] {
        [
            self.gripper_collider,
            self.finger_0_collider,
            self.finger_1_collider,
            self.finger_2_collider,
            self.finger_0_2_collider,
            self.finger_1_2_collider,
            self.finger_2_2_collider,
        ]
    }
}

#[derive(Clone, Copy)]
pub enum GripperDirection {
    Open,
    Closed,
}

pub fn set_gripper_direction(
    physics: &mut PhysicsWorld,
    bdi: &RobotBodyPartIndex,
    dir: GripperDirection,
) {
    for bp in bdi.finger_parts().iter() {
        multibody_util::set_motor_speed(
            physics,
            *bp,
            match dir {
                GripperDirection::Open => 1.0,
                GripperDirection::Closed => -1.0,
            },
        );
    }
}

/// Extract the joint angles of every finger part of the robot.
pub fn gripper_finger_angles(
    physics: &PhysicsWorld,
    robot: &RobotBodyPartIndex,
) -> FingerJointMap<f32> {
    FingerJointMap {
        finger_0: revolute_joint_angle(physics, robot.finger_0).unwrap(),
        finger_1: revolute_joint_angle(physics, robot.finger_1).unwrap(),
        finger_2: revolute_joint_angle(physics, robot.finger_2).unwrap(),
        finger_0_2: revolute_joint_angle(physics, robot.finger_0_2).unwrap(),
        finger_1_2: revolute_joint_angle(physics, robot.finger_1_2).unwrap(),
        finger_2_2: revolute_joint_angle(physics, robot.finger_2_2).unwrap(),
    }
}

pub fn revolute_joint_angle(
    physics: &PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<f32> {
    multibody_util::get_joint::<RevoluteJoint<f32>>(physics, part_handle).map(RevoluteJoint::angle)
}

pub fn kinematic_model_from_robot(p: &PhysicsWorld, robot: &RobotBodyPartIndex) -> KinematicModel {
    KinematicModel::from_multibody(
        p,
        robot.base,
        &[robot.swivel, robot.link1, robot.link2, robot.gripper],
        Vector3::new(0.0, 1.0, 0.0),
    )
}
