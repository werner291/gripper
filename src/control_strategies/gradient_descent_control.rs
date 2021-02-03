use std::iter::Iterator;

use kiss3d::ncollide3d::na::{distance_squared, Point3, Unit, Vector3};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyPart, DefaultBodyHandle};
use rand::Rng;

use crate::physics::PhysicsWorld;

use crate::robot::{
    get_multibody_link, GripperDirection, JointVelocities, RobotBodyPartIndex, CHANNEL_FINGER_0,
    CHANNEL_FINGER_0_2, CHANNEL_FINGER_1, CHANNEL_FINGER_1_2, CHANNEL_FINGER_2, CHANNEL_FINGER_2_2,
    CHANNEL_GRIPPER, CHANNEL_LINK1, CHANNEL_LINK2, CHANNEL_SWIVEL, NUM_CHANNELS,
};
use nalgebra::Matrix3;
use std::convert::From;
use std::option::Option::Some;

use std::vec::Vec;

use crate::simulator_thread::ControllerStrategy;

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub(crate) struct GradientDescentController {
    target: DefaultBodyHandle,
}

impl GradientDescentController {
    pub fn new(target: DefaultBodyHandle) -> Self {
        GradientDescentController { target }
    }
}

impl ControllerStrategy for GradientDescentController {
    fn apply_controller(
        &mut self,
        physics: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities {
        // FIXME robot::set_gripper_direction(physics, &robot, GripperDirection::Open);

        let target_position = physics
            .bodies
            .rigid_body(self.target)
            .expect("Target is not a rigid body!")
            .position()
            * Point3::new(0.0, 0.0, 0.0);

        let gripper_pos = point_inside_gripper(&physics, robot);

        // Will be used as our energy function to be minimized.
        let _remaining_distance = distance_squared(&target_position, &gripper_pos);

        // Gradient of square distance is simply the difference in positions.
        let distance_gradient = &target_position - gripper_pos;

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

        let final_target_speeds = if let Some(speeds) = motor_speeds {
            if speeds.norm() > 1.0 {
                speeds.normalize()
            } else {
                speeds
            }
        } else {
            Vector3::new(
                rng.gen_range(-0.1..0.1),
                rng.gen_range(-0.1..0.1),
                rng.gen_range(-0.1..0.1),
            )
        };

        JointVelocities {
            swivel: final_target_speeds[0],
            link1: final_target_speeds[1],
            link2: final_target_speeds[2],
            gripper: 0.0,
            finger_0: 0.1,
            finger_1: 0.1,
            finger_2: 0.1,
            finger_0_2: 0.1,
            finger_1_2: 0.1,
            finger_2_2: 0.1,
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
