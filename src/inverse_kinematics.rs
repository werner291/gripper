use crate::kinematics::KinematicModel;
use float_ord::FloatOrd;
use kiss3d::ncollide3d::na::distance;
use na::{distance_squared, Point3, Vector3, Vector6};
use nphysics3d::algebra::Velocity3;
use rand::{thread_rng, Rng};
use std::borrow::ToOwned;
use std::convert::Into;
use std::f32::consts::PI;
use std::f32::INFINITY;
use std::iter::{IntoIterator, Iterator};
use std::option::Option;
use std::option::Option::{None, Some};
use std::prelude::v1::Vec;
use std::result::Result;
use std::result::Result::{Err, Ok};

/// A rather stupid approach that simply picks 10000 sets of joint angles at random,
/// and returns whichever set gets the closest to the target position and heading
/// for the tip of the kinematic chain.
pub fn best_of_random_final_angles(
    kinematic_model: &KinematicModel,
    starting_angles: &[f32],
    target_position: &Point3<f32>,
    target_heading: &Vector3<f32>,
) -> Vec<f32> {
    let mut rng = thread_rng();

    let angles = (0..10000)
        .map(|_| {
            let mut candidate = starting_angles.to_owned();
            kinematic_model.mutate_angles(&mut candidate, &mut rng, 2.1);
            candidate
        })
        .min_by_key(|c| {
            FloatOrd(solution_distance(
                &kinematic_model,
                target_position,
                target_heading,
                c,
            ))
        })
        .unwrap();

    angles
}

fn solution_distance(
    kinematic_model: &KinematicModel,
    target_position: &Point3<f32>,
    target_heading: &Vector3<f32>,
    c: &[f32],
) -> f32 {
    let tip_pos = kinematic_model.predict(c).tip_position;

    let tip_position: Point3<f32> = tip_pos.translation.vector.into();
    let tip_heading: Vector3<f32> =
        tip_pos.rotation * &kinematic_model.links.last().unwrap().offset;

    distance_squared(target_position, &tip_position) + (target_heading - tip_heading).norm_squared()
}

#[derive(Debug)]
pub struct BestAttempt {
    pub angles: Vec<f32>,
    pub remaining_distance: f32,
}

pub fn gradient_guided_planner(
    kinematic_model: &KinematicModel,
    target_position: &Point3<f32>,
    target_heading: &Vector3<f32>,
    attempts: usize,
    iterations_per_attempt: usize,
    suggestion: Option<&[f32]>,
) -> Result<Vec<f32>, BestAttempt> {
    let mut rng = rand::thread_rng();

    const ABANDON_THRESHOLD: f32 = 0.0001f32;
    const INITIAL_STEP_SIZE: f32 = 0.1;
    const ACCEPT_THRESHOLD: f32 = 0.001f32;

    let mut best_attempt: Option<Vec<f32>> = None;
    let mut best_score = INFINITY;

    for attempt in 0..attempts {
        let mut step_size = INITIAL_STEP_SIZE;

        let mut angles: Vec<f32> = if let Some(suggestion) = suggestion {
            kinematic_model
                .links
                .iter()
                .zip(suggestion.iter())
                .map(|(ln, sa)| {
                    (sa + rng.gen_range(-PI..PI) * (attempt as f32 / attempts as f32))
                        .min(ln.max)
                        .max(ln.min)
                })
                .collect()
        } else {
            kinematic_model
                .links
                .iter()
                .map(|ln| rng.gen_range(ln.min.max(-PI)..ln.max.min(PI)))
                .collect()
        };

        for _itr in 0..iterations_per_attempt {
            let tip_pos = kinematic_model.predict(angles.as_slice()).tip_position;

            let tip_position: Point3<f32> = tip_pos.translation.vector.into();
            let tip_heading: Vector3<f32> =
                tip_pos.rotation * &kinematic_model.links.last().unwrap().offset;

            let tip_gradient = Vector6::new(
                tip_position.x - target_position.x,
                tip_position.y - target_position.y,
                tip_position.z - target_position.z,
                tip_heading.x - target_heading.x,
                tip_heading.y - target_heading.y,
                tip_heading.z - target_heading.z,
            );

            let distance_left = tip_gradient.norm_squared();

            if best_score > distance_left {
                best_score = distance_left;
                best_attempt = Some(angles.clone());
            }

            if distance_left < ACCEPT_THRESHOLD {
                return Ok(angles);
            } else {
                let gradients: Vec<f32> = kinematic_model
                    .velocity_gradients(angles.as_slice())
                    .into_iter()
                    .map(|v: Velocity3<f32>| v.as_vector().dot(&tip_gradient))
                    .collect();

                if gradients.iter().map(|x| x.abs()).sum::<f32>() < ABANDON_THRESHOLD {
                    break; // Abandon attempt, gradient is gone.
                }

                for (a, g) in angles.iter_mut().zip(gradients) {
                    *a -= g * step_size;
                }
                step_size *= 0.99;

                kinematic_model.cap_angles_to_legal(angles.as_mut_slice());
            }
        }
    }

    if distance(
        &(&kinematic_model.origin * Point3::new(0.0, 0.0, 0.0)),
        target_position,
    ) > kinematic_model.chain_length()
    {
        eprintln!("Inverse kinematic solver likely failed because target position is further away than the length of the kinematic chain.")
    }

    Err(BestAttempt {
        angles: best_attempt.unwrap(),
        remaining_distance: best_score,
    })
}
