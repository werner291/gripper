//! A module containing some utilities with respect to kinematics,
//! as well as a simplified datatype that represents kinematic models,
//! a simplified representation of a robot.

use std::prelude::v1::{Iterator, Vec};

use na::{Isometry3, Matrix3xX, Matrix6xX, Point3, Translation3, Unit, Vector3, Vector6};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyPart, DefaultBodyPartHandle};
use rand::prelude::ThreadRng;
use rand::Rng;

use crate::multibody_util::{get_joint, get_multibody_link};
use crate::physics::PhysicsWorld;
use nphysics3d::algebra::Velocity3;
use std::convert::{From, Into};
use std::iter::IntoIterator;
use std::option::Option;
use std::option::Option::Some;

/// A link in a KinematicModel, assumed to consist of an axis of rotation,
/// as well as a translation, and max/min angles (which may be infinite).
#[derive(Debug)]
pub struct KinematicLink {
    pub offset: Vector3<f32>,
    pub axis: Unit<Vector3<f32>>,
    pub min: f32,
    pub max: f32,
}

/// A simple kinematic chain model, consisting of a single origin and a set of revolute links.
///
/// The `origin` field is effectively an "anchor" where the base of the chain is attached in space.
///
/// Then, each link contains an 'offset' and an 'axis' field. The axis is the direction of the axis
/// around which the link rotates, and the offset is the vector from the base to the tip of the link
/// when at 0 angle.
///
#[derive(Debug)]
pub struct KinematicModel {
    // The position of the first link with 0 angle.
    pub origin: Isometry3<f32>,
    pub links: Vec<KinematicLink>,
}

pub struct PredictedPositions {
    pub link_base_positions: Vec<Isometry3<f32>>,
    pub tip_position: Isometry3<f32>,
}

impl KinematicModel {
    /// Predict global the tip position of every kinematic link.
    pub fn predict(&self, angles: &[f32]) -> PredictedPositions {
        let mut last_tip_position = self.origin.clone();

        let mut base_positions = Vec::new();

        for (a, l) in angles.iter().zip(self.links.iter()) {
            let base_pos = last_tip_position
                * Isometry3::new(Vector3::new(0.0, 0.0, 0.0), l.axis.into_inner() * *a);
            last_tip_position = base_pos * Translation3::from(l.offset);
            base_positions.push(base_pos);
        }

        PredictedPositions {
            link_base_positions: base_positions,
            tip_position: last_tip_position,
        }
    }

    /// Small convenience function to randomly mutate the given slice of angles by at most `range`,
    /// while capping them to the legal angles of the kinematic model.
    pub fn mutate_angles(&self, angles: &mut [f32], rng: &mut ThreadRng, range: f32) {
        for angle in angles.iter_mut() {
            *angle += rng.gen_range(-range..range);
        }
        self.cap_angles_to_legal(angles);
    }

    /// Mutate the given angles such that they are within the legal range of every kinematic link.
    pub fn cap_angles_to_legal(&self, angles: &mut [f32]) {
        for (angle, link) in angles.iter_mut().zip(self.links.iter()) {
            *angle = angle.max(link.min).min(link.max);
        }
    }

    /// Computes the gradient of the velocity of the tip of the kinematic chain,
    /// with respect to the given joint angles.
    ///
    /// Effectively, this is equivalent to the velocity of the tip if
    /// each of the joints was rotating at exactly unit speed.
    ///
    pub fn velocity_gradients(&self, angles: &[f32]) -> Vec<Velocity3<f32>> {
        let PredictedPositions {
            link_base_positions,
            tip_position,
        } = self.predict(angles);

        let end_position = tip_position.translation.vector.into();

        angles
            .iter()
            .enumerate()
            .map(|(i, _a)| {
                // Global point at the center of the joint.
                let link_position = link_base_positions[i] * Point3::new(0.0, 0.0, 0.0);

                // A vector from the center of the joint to the point inside the gripper.
                let toward_end = &end_position - &link_position;

                // Extract the global rotation axis of this joint.
                let rot_axis: Unit<Vector3<f32>> = link_base_positions[i] * self.links[i].axis;

                // Linear velocity of the gripper if the current joint was rotating at unit velocity, all others immobile.
                let lv_g = -toward_end.cross(&rot_axis);

                Velocity3 {
                    linear: lv_g,
                    angular: -rot_axis.into_inner(),
                }
            })
            .collect()
    }

    pub fn predict_tip_linear_velocity(
        &self,
        angles: &[f32],
        motor_speeds: &[f32],
    ) -> Vector3<f32> {
        self.velocity_gradients(angles)
            .into_iter()
            .zip(motor_speeds.iter())
            .map(|(v, ms)| v.linear * *ms)
            .sum()
    }

    pub fn inverse_solve_velocity(
        &self,
        angles: &[f32],
        tip_velocity: &Velocity3<f32>,
    ) -> Option<Vec<f32>> {
        let vg: Vec<Vector6<f32>> = self
            .velocity_gradients(angles)
            .into_iter()
            .map(|v| *v.as_vector())
            .collect();

        let mtx = Matrix6xX::from_columns(vg.as_slice());

        (mtx.transpose() * &mtx)
            .lu()
            .solve(&(mtx.transpose() * tip_velocity.as_vector()))
            .map(|v| v.into_iter().cloned().collect())
    }

    pub fn inverse_solve_linear_velocity(
        &self,
        angles: &[f32],
        tip_linear_velocity: &Vector3<f32>,
    ) -> Option<Vec<f32>> {
        let vg: Vec<Vector3<f32>> = self
            .velocity_gradients(angles)
            .into_iter()
            .map(|v| v.linear)
            .collect();

        let mtx = Matrix3xX::from_columns(vg.as_slice());

        (mtx.transpose() * &mtx)
            .lu()
            .solve(&(mtx.transpose() * tip_linear_velocity))
            .map(|v| v.into_iter().cloned().collect())
    }

    //noinspection RsTypeCheck (Typechecking is apparently broken for cloning the parent shift...)
    /// Derive a KinematicModel from a Multibody, given a set of links that are assumed
    /// to form a kinematic chain
    pub fn from_multibody(
        physics: &PhysicsWorld,
        base: DefaultBodyPartHandle,
        links: &[DefaultBodyPartHandle],
        last_tip: Vector3<f32>,
    ) -> Self {
        Self {
            origin: get_multibody_link(physics, base).unwrap().position()
                * Translation3::from(
                    *get_multibody_link(physics, *links.first().unwrap())
                        .unwrap()
                        .parent_shift(),
                ),
            links: links
                .iter()
                .enumerate()
                .map(|(i, bph)| {
                    let offset: Vector3<f32> = if let Some(next_bph) = links.get(i + 1) {
                        *get_multibody_link(physics, *next_bph)
                            .unwrap()
                            .parent_shift()
                    } else {
                        last_tip
                    };

                    let link_joint = get_joint::<RevoluteJoint<f32>>(physics, *bph).unwrap();
                    KinematicLink {
                        offset,
                        axis: link_joint.axis(),
                        min: link_joint.min_angle().unwrap_or(std::f32::NEG_INFINITY),
                        max: link_joint.max_angle().unwrap_or(std::f32::INFINITY),
                    }
                })
                .collect(),
        }
    }

    /// Get the sum of the lengths of all links in the kinematic chain,
    /// to get a rough estimate of maximum reach.
    pub fn chain_length(&self) -> f32 {
        self.links.iter().map(|link| link.offset.norm()).sum()
    }
}
