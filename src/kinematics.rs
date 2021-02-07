//! A module containing some utilities with respect to kinematics.

use std::clone::Clone;
use std::prelude::v1::{Iterator, Vec};

use na::{Isometry3, Unit, Vector3, Point3, distance, Matrix3xX};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyPart, DefaultBodyPartHandle};
use rand::prelude::ThreadRng;
use rand::Rng;

use crate::multibody_util::get_multibody_link;
use crate::physics::PhysicsWorld;
use nphysics3d::algebra::Velocity3;
use std::iter::IntoIterator;
use std::option::Option::Some;
use std::option::Option;
use std::convert::From;

/// A link in a KinematicModel, assumed to consist of an axis of rotation,
/// as well as a translation, and max/min angles (which may be infinite).
struct KinematicLink {
    offset: Vector3<f32>,
    axis: Unit<Vector3<f32>>,
    min: f32,
    max: f32,
}

/// A simple kinematic chain model, consisting of a single origin and a set of revolute links.
pub struct KinematicModel {
    origin: Isometry3<f32>,
    links: Vec<KinematicLink>
}

pub struct PredictedPositions(pub Vec<Isometry3<f32>>);

impl KinematicModel {

    /// Predict global the tip position of every kinematic link.
    pub fn predict(&self, angles : &[f32]) -> PredictedPositions {
        PredictedPositions(
            angles.iter().zip( self.links.iter()).scan( self.origin.clone(),
            | pos, (angle, link)| {
                let local_pos = Isometry3::new(
                    link.offset.clone(),
                    link.axis.into_inner() * * angle
                );

                *pos = *pos * local_pos;

                Some(pos.clone())
            }).collect()
        )
    }

    /// Small convenience function to randomly mutate the given slice of angles by at most `range`,
    /// while capping them to the legal angles of the kinematic model.
    pub fn mutate_angles(&self, angles : &mut [f32], rng: &mut ThreadRng, range: f32) {
        for angle in angles.iter_mut() {
            *angle = *angle + rng.gen_range(-range..range);
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

        let PredictedPositions(positions) = self.predict(angles);

        let end_position = positions.last().unwrap() * Point3::new(0.0, 0.0, 0.0);

        angles.iter().enumerate().map(|(i,a)| {
            // Global point at the center of the joint.
            let link_position = positions[i+1] * Point3::new(0.0, 0.0, 0.0);

            // A vector from the center of the joint to the point inside the gripper.
            let toward_end = &end_position - &link_position;

            // Extract the global rotation axis of this joint.
            let rot_axis: Unit<Vector3<f32>> = positions[i+1] * self.links[i].axis;

            // Linear velocity of the gripper if the current joint was rotating at unit velocity, all others immobile.
            let lv_g = toward_end.cross(&rot_axis);

            Velocity3 {
                linear: lv_g,
                angular: rot_axis.into_inner()
            }
        }).collect()
    }

    pub fn inverse_solve_linear_velocity(&self, angles: &[f32], tip_linear_velocity: &Vector3<f32>) -> Option<Vec<f32>> {

        let vg : Vec<Vector3<f32>> = self.velocity_gradients(angles).into_iter().map(|v| v.linear ).collect();

        let mtx = Matrix3xX::from_columns(vg.as_slice());

        (mtx.transpose() * &mtx).lu().solve(&(mtx.transpose() * tip_linear_velocity)).map(|v| v.into_iter().cloned().collect())

    }

    // /// Derive a KinematicModel from a Multibody, given a set of links that are assumed
    // /// to form a kinematic chain
    // pub fn from_multibody(physics: &PhysicsWorld, robot: &[DefaultBodyPartHandle]) -> Self {
    //     Self {
    //         origin: get_multibody_link(physics, robot[0]).unwrap().position(),
    //         links: robot.iter().skip(1).map(|bph| {
    //             let link = get_multibody_link(physics, *bph).unwrap();
    //             KinematicLink {
    //                 offset: link.parent_shift().clone(),
    //                 axis: link.joint().downcast_ref::<RevoluteJoint<f32>>().unwrap().axis(),
    //                 min: link.joint().downcast_ref::<RevoluteJoint<f32>>().unwrap().min_angle().unwrap_or(std::f32::NEG_INFINITY),
    //                 max: link.joint().downcast_ref::<RevoluteJoint<f32>>().unwrap().max_angle().unwrap_or(std::f32::INFINITY)
    //             }
    //         }).collect()
    //     }
    // }
}
