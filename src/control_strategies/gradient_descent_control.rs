use std::iter::Iterator;

use kiss3d::ncollide3d::na::{distance_squared, Point3, Unit, Vector3};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::BodyPart;
use rand::Rng;

use crate::physics::{ControllerStrategy, PhysicsWorld};
use crate::robot;
use crate::robot::{get_multibody_link, GripperDirection, RobotBodyPartIndex};
use nalgebra::Matrix3;
use std::convert::From;
use std::option::Option::Some;
use std::process::exit;
use std::vec::Vec;

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub(crate) struct GradientDescentController {
    target_position: Point3<f32>,
}

impl GradientDescentController {
    pub fn new(target_position: Point3<f32>) -> Self {
        GradientDescentController {target_position}
    }
}

impl ControllerStrategy for GradientDescentController {
    fn apply_controller(&mut self, physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex) {
        robot::set_gripper_direction(physics, &robot, GripperDirection::Open);

        let gripper_pos = point_inside_gripper(&physics, robot);

        // Will be used as our energy function to be minimized.
        let _remaining_distance = distance_squared(&self.target_position, &gripper_pos);

        // Gradient of square distance is simply the difference in positions.
        let distance_gradient = &self.target_position - gripper_pos;

        // let gripper_forward : Vector3<f32> = gripper_position * Vector3::new(0.0, 1.0, 0.0);

        // let gripper_preferred_rotation_axis: Vector3<f32> = target_heading.cross(&gripper_forward);

        let motors = [robot.swivel, robot.link1, robot.link2];

        let motor_gradients = motors
            .iter()
            .map(|bph| {
                // Retrieve the position and rotation of the body link being considered.
                let link = physics
                    .bodies
                    .multibody(robot.body)
                    .unwrap()
                    .link(bph.1)
                    .unwrap();

                let link_position = Point3::from(link.position().translation.vector);

                // A vector from the center of the joint to the point inside the gripper.
                let toward_gripper = &gripper_pos - &link_position;

                // Extract the rotation axis of this joint.
                let rot_axis_local: &Unit<Vector3<f32>> = &link
                    .joint()
                    .downcast_ref::<RevoluteJoint<f32>>()
                    .unwrap()
                    .axis();

                let rot_axis_global: Unit<Vector3<f32>> = link.position().rotation * rot_axis_local;

                // Velocity of the gripper if the current joint was rotating at unit velocity,
                // all others immobile.
                -toward_gripper.cross(&rot_axis_global)

                // let gradient_at_joint = induced_velocity.dot(&distance_gradient);

                // let ideal_rotation = toward_gripper.cross(&toward_ball) / toward_ball.norm();
                //
                //
                // let translational_gradient = ideal_rotation.dot(&rot_axis_global) / 1.5;
                //
                // let rotational_gradient = -rot_axis_global.dot(&gripper_preferred_rotation_axis);

                // rotational_gradient +
                // -gradient_at_joint
            })
            .collect::<Vec<_>>();

        let mut rng = rand::thread_rng();

        // let overall_size = motor_gradients.iter().map(|x| x.abs()).sum::<f32>();

        // dbg!(&motor_gradients);

        let motor_speeds = Matrix3::from_columns(&motor_gradients)
            .lu()
            .solve(&distance_gradient);

        // dbg!(&motor_speeds);

        if let Some(speeds) = motor_speeds {
            let safe_speeds = if speeds.norm() > 1.0 {
                speeds.normalize()
            } else {
                speeds
            };

            for (bph, g) in motors.iter().zip(safe_speeds.iter()) {
                if g.is_nan() {
                    exit(0);
                }

                robot::set_motor_speed(physics, *bph, *g);
            }
        } else {
            for (bph, _g) in motors.iter().zip(motor_speeds.iter()) {
                robot::set_motor_speed(physics, *bph, rng.gen_range(-0.1..0.1));
            }
        }
    }
}

pub fn point_inside_gripper(physics: &PhysicsWorld, robot: &RobotBodyPartIndex) -> Point3<f32> {
    // Get the position and orientation of the "gripper", i.e. the base of the end-effector.
    let gripper = get_multibody_link(&physics, robot.gripper).unwrap();

    // Extract spatial position without orientation, including a bit of offset to get a position
    // inside the gripper, instead of at the wrist.
    let gripper_pos: Point3<f32> = gripper.position() * Point3::new(0.0, 1.0, 0.0); // + &target_heading.into_inner() * 0.9;
    gripper_pos
}
