//! A module containing some structs that make interactions with the standard robot arm a bit easier.
//!
//! Primarily, it improves upon the previous approach that used numerical indices.

use na::{Scalar, Vector4};
use std::clone::Clone;
use std::convert::{From, Into, TryFrom};
use std::result::Result;
use std::result::Result::{Err, Ok};

#[derive(Debug)]
pub struct WrongLengthError;

//region ArmJointMap

/// A struct with one entry for every joint that affects the position of the end effector
#[derive(Debug, Clone, Copy)]
pub struct ArmJointMap<T> {
    pub swivel: T,
    pub link1: T,
    pub link2: T,
    pub gripper: T,
}

impl<N: Scalar> From<Vector4<N>> for ArmJointMap<N> {
    fn from(v: Vector4<N>) -> Self {
        ArmJointMap {
            swivel: v[0].clone(),
            link1: v[1].clone(),
            link2: v[2].clone(),
            gripper: v[3].clone(),
        }
    }
}

impl<N: Clone + Scalar> From<&Vector4<N>> for ArmJointMap<N> {
    fn from(v: &Vector4<N>) -> Self {
        ArmJointMap {
            swivel: v[0].clone(),
            link1: v[1].clone(),
            link2: v[2].clone(),
            gripper: v[3].clone(),
        }
    }
}

impl<N: Scalar> Into<Vector4<N>> for ArmJointMap<N> {
    fn into(self) -> Vector4<N> {
        Vector4::new(self.swivel, self.link1, self.link2, self.gripper)
    }
}

impl<N> Into<[N; 4]> for ArmJointMap<N> {
    fn into(self) -> [N; 4] {
        [self.swivel, self.link1, self.link2, self.gripper]
    }
}

impl<T: Clone> TryFrom<&[T]> for ArmJointMap<T> {
    type Error = WrongLengthError;

    fn try_from(value: &[T]) -> Result<Self, WrongLengthError> {
        if value.len() == 4 {
            Ok(Self {
                swivel: value[0].clone(),
                link1: value[1].clone(),
                link2: value[2].clone(),
                gripper: value[3].clone(),
            })
        } else {
            Err(WrongLengthError)
        }
    }
}

//endregion

//region ArmJointVelocities

pub type ArmJointVelocities = ArmJointMap<f32>;

impl ArmJointVelocities {
    /// If the largest absolute value is larger than the given limit all values
    /// are uniformly scaled down such that the largest is exactly the limit.
    pub fn limit_to_safe(self, lim: f32) -> ArmJointVelocities {
        let max = self
            .swivel
            .abs()
            .max(self.link1.abs())
            .max(self.link2.abs())
            .max(self.gripper.abs());
        if max > lim {
            Self {
                swivel: lim * self.swivel / max,
                link1: lim * self.link1 / max,
                link2: lim * self.link2 / max,
                gripper: lim * self.gripper / max,
            }
        } else {
            self
        }
    }
}

//endregion

//region FingerJointMap

#[derive(Debug, Clone)]
pub struct FingerJointMap<T> {
    pub finger_0: T,
    pub finger_1: T,
    pub finger_2: T,
    pub finger_0_2: T,
    pub finger_1_2: T,
    pub finger_2_2: T,
}

pub const FINGERS_OPEN: FingerJointMap<f32> = FingerJointMap {
    finger_0: 0.5,
    finger_1: 0.5,
    finger_2: 0.5,
    finger_0_2: 0.5,
    finger_1_2: 0.5,
    finger_2_2: 0.5,
};

pub const FINGERS_CLOSE: FingerJointMap<f32> = FingerJointMap {
    finger_0: -0.5,
    finger_1: -0.5,
    finger_2: -0.5,
    finger_0_2: -0.5,
    finger_1_2: -0.5,
    finger_2_2: -0.5,
};

pub const ZERO_JOINT_VELOCITIES: JointVelocities = JointVelocities {
    swivel: 0.0,
    link1: 0.0,
    link2: 0.0,
    gripper: 0.0,
    finger_0: 0.0,
    finger_1: 0.0,
    finger_2: 0.0,
    finger_0_2: 0.0,
    finger_1_2: 0.0,
    finger_2_2: 0.0,
};

//endregion

//region JointMap

/// A struct with one entry for each motorized joint in the robot.
#[derive(Debug, Clone)]
pub struct JointMap<T> {
    pub swivel: T,
    pub link1: T,
    pub link2: T,
    pub gripper: T,
    pub finger_0: T,
    pub finger_1: T,
    pub finger_2: T,
    pub finger_0_2: T,
    pub finger_1_2: T,
    pub finger_2_2: T,
}

impl<T> JointMap<T> {
    /// Merges an ArmJointMap and a FingerJointMap into a JointMap.
    pub fn from_arm_and_finger(arm: ArmJointMap<T>, finger: FingerJointMap<T>) -> JointMap<T> {
        JointMap {
            swivel: arm.swivel,
            link1: arm.link1,
            link2: arm.link2,
            gripper: arm.gripper,
            finger_0: finger.finger_0,
            finger_1: finger.finger_1,
            finger_2: finger.finger_2,
            finger_0_2: finger.finger_0_2,
            finger_1_2: finger.finger_1_2,
            finger_2_2: finger.finger_2_2,
        }
    }
}

pub type JointVelocities = JointMap<f32>;

//endregion
