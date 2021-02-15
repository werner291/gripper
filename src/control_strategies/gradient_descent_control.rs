use std::iter::{IntoIterator, Iterator};
use std::option::Option;
use std::vec::Vec;


use nalgebra::{Point3, Unit, Vector3};
use nphysics3d::algebra::Velocity3;
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{DefaultBodyHandle};
use rand::Rng;

use crate::multibody_util::{get_joint};
use crate::physics::PhysicsWorld;

use crate::robot::{ArmJointVelocities, JointVelocities, RobotBodyPartIndex, FINGERS_OPEN, FINGERS_CLOSE};
use crate::control_strategies::ControllerStrategy;
use kiss3d::ncollide3d::na::Isometry3;
use kiss3d::ncollide3d::shape::ConvexHull;
use nphysics3d::ncollide3d::query::PointQuery;
use std::convert::From;
use crate::kinematics::KinematicModel;
use std::clone::Clone;

enum SphereGrabState {
    Approaching,
    Grabbing,
    Lifting,
}

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub struct GradientDescentController {
    target: DefaultBodyHandle,
    state: SphereGrabState,
}

impl ControllerStrategy for GradientDescentController {
    fn apply_controller(
        &mut self,
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities {
        let target_position = physics
            .bodies
            .rigid_body(self.target)
            .expect("Target is not a rigid body!")
            .position()
            * Point3::new(0.0, 0.0, 0.0);

        match self.state {
            SphereGrabState::Approaching => {

                // The ball sits on the ground, so we want the gripper to be pointing down.
                let target_heading = Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0));

                // Build the kinematic model and compute necessary kinematics-related information.
                // Model is configured such that the tip of the kinematic chain lies inside the gripper.
                let kinematic = KinematicModel::from_multibody(physics, robot.base, &[robot.swivel, robot.link1, robot.link2, robot.gripper], Vector3::new(0.0, 1.0, 0.0));
                let joint_angles: Vec<f32> = [robot.swivel, robot.link1, robot.link2, robot.gripper].iter().map(|bph| {
                    get_joint::<RevoluteJoint<f32>>(physics, *bph).unwrap().angle()
                }).collect();

                let current_state = kinematic.predict(joint_angles.as_slice());

                let gripper_pos = Point3::from(current_state.tip_position.translation.vector.clone());
                let gripper_heading = current_state.tip_position * Vector3::new(0.0, 1.0, 0.0);

                // Vector by which to translate the reference point to the center of the target.
                let translation_delta = target_position - gripper_pos;
                // Angle-axis rotation necessary to make the gripper point the right way.
                let rotation_delta = -gripper_heading.cross(&target_heading.into_inner());

                // Compute the remaining distance and angle
                let distance_remaining = translation_delta.norm();

                let angle_remaining = rotation_delta.norm(); // FIXME sign?
                
                let target_velocity = Velocity3 {
                    linear: translation_delta * 0.1,
                    angular: rotation_delta // FIXME Magnitude should be angle, not sine of angle.
                };

                let jv = kinematic.inverse_solve_velocity(joint_angles.as_slice(), &target_velocity)
                    .map(|speeds|
                        ArmJointVelocities {
                            swivel: speeds[0],
                            link1: speeds[1],
                            link2: speeds[2],
                            gripper: speeds[3],
                        }
                    )
                    .unwrap_or_else(GradientDescentController::random_arm_velocities)
                    //The solver can sometimes return solutions that are a teensy bit excessive.
                    .limit_to_safe(0.5);

                // If the remaining distance is smaller than a threshold, go to state Grabbing.
                // The robot will act upon this state next turn.
                if distance_remaining + angle_remaining < 0.05 {
                    self.state = SphereGrabState::Grabbing;
                }

                // Combine the requested arm joint velocities with finger joint velocities
                // to open the gripper, then return the velocities for execution.
                JointVelocities::from_arm_and_finger(jv, FINGERS_OPEN)
            }
            SphereGrabState::Grabbing => {
                // let angles = gripper_finger_angles(physics, robot);
                //
                // ;

                if grabbed(physics, robot, self.target) {
                    self.state = SphereGrabState::Lifting
                }

                JointVelocities::from_arm_and_finger(
                    ArmJointVelocities {
                        swivel: 0.0,
                        link1: 0.0,
                        link2: 0.0,
                        gripper: 0.0,
                    },
                    FINGERS_CLOSE)
            }
            SphereGrabState::Lifting => {
                println!("Lifting!");

                let _at_point = &target_position;
                let velocity = &Velocity3::linear(0.0, 1.0, 0.0);
                let kinematic = KinematicModel::from_multibody(physics, robot.base, &[robot.swivel, robot.link1, robot.link2, robot.gripper], Vector3::new(0.0, 1.0, 0.0));
                let joint_angles: Vec<f32> = [robot.swivel, robot.link1, robot.link2, robot.gripper].iter().map(|bph| {
                    get_joint::<RevoluteJoint<f32>>(physics, *bph).unwrap().angle()
                }).collect();
                let jv = kinematic.inverse_solve_velocity(joint_angles.as_slice(), &velocity)
                    .map(|speeds|
                        ArmJointVelocities {
                            swivel: speeds[0],
                            link1: speeds[1],
                            link2: speeds[2],
                            gripper: speeds[3],
                        }
                    )
                    .unwrap_or_else(GradientDescentController::random_arm_velocities)
                    //The solver can sometimes return solutions that are a teensy bit excessive.
                    .limit_to_safe(10.0);

                JointVelocities::from_arm_and_finger(jv, FINGERS_CLOSE)
            }
        }
    }
}

impl GradientDescentController {
    pub fn new(target: DefaultBodyHandle) -> Self {
        GradientDescentController {
            target,
            state: SphereGrabState::Approaching,
        }
    }

    /// Generate some random arm arm joint velocities,
    /// useful to break gimbal lock conditions.
    fn random_arm_velocities() -> ArmJointVelocities {
        let mut rng = rand::thread_rng();
        ArmJointVelocities {
            swivel: rng.gen_range(-0.1..0.1),
            link1: rng.gen_range(-0.1..0.1),
            link2: rng.gen_range(-0.1..0.1),
            gripper: rng.gen_range(-0.1..0.1),
        }
    }
}

pub fn grabbed(physics: &PhysicsWorld, robot: &RobotBodyPartIndex, target: DefaultBodyHandle) -> bool {
    let mut normals = Vec::new();

    for fc in robot.gripper_colliders().iter() {
        for (_, ca, _, cb, _, cman) in physics
            .geometrical_world
            .contacts_with(&physics.colliders, *fc, true)
            .expect("Finger collider does not exist in the world?")
        {
            if ca.body() == target {
                for contact in cman.contacts() {
                    normals.push(contact.contact.normal)
                }
            }

            if cb.body() == target {
                for contact in cman.contacts() {
                    normals.push(-contact.contact.normal);
                }
            }
        }
    }

    if normals.len() <= 3 {
        false
    } else {
        let ch: Option<ConvexHull<f32>> = ConvexHull::try_from_points(
            normals
                .into_iter()
                .map(|n| Point3::from(n.into_inner()))
                .collect::<Vec<_>>()
                .as_slice(),
        );

        match ch {
            Option::None => false,
            Option::Some(c) => {
                c.contains_point(&Isometry3::identity(), &Point3::new(0.0, 0.0, 0.0))
            }
        }
    }
}
