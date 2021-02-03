use std::iter::Iterator;

use nalgebra::{distance_squared, Point3, Unit, Vector3, Vector4, Vector6};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyPart, DefaultBodyHandle};
use rand::Rng;

use crate::physics::PhysicsWorld;

use crate::robot::{
    get_multibody_link, JointVelocities, RobotBodyPartIndex,
};
use nalgebra::{Matrix3x4, Matrix4x3, Matrix4, Matrix6x4, Matrix4x6};
use std::convert::From;
use std::option::Option::Some;

use std::vec::Vec;

use crate::simulator_thread::ControllerStrategy;

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub(crate) struct GradientDescentController {
    target: DefaultBodyHandle,
    heading_vector: Unit<Vector3<f32>>
}

impl GradientDescentController {
    pub fn new(target: DefaultBodyHandle, heading_vector: Unit<Vector3<f32>>) -> Self {
        GradientDescentController { target, heading_vector }
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

        let physics_argument = &physics;
        let gripper = get_multibody_link(&physics_argument, robot.gripper).unwrap();

        let gripper_pos: Point3<f32> = gripper.position() * Point3::new(0.0, 1.0, 0.0);
        let gripper_forward: Vector3<f32> = gripper.position() * Vector3::new(0.0, 1.0, 0.0);

        // Will be used as our energy function to be minimized.
        // let _remaining_distance = distance_squared(&target_position, &gripper_pos);
        // let _remaining_heading_distance = (&self.heading_vector - &gripper_forward).norm_squared();

        // Gradient of square distance is simply the difference in positions.
        let distance_gradient = &target_position - gripper_pos;
        let preferred_rotation = gripper_forward.cross(&self.heading_vector.into_inner());
        let heading_gradient = preferred_rotation.norm_squared();

        let combined_gradient = Vector4::new(distance_gradient.x, distance_gradient.y, distance_gradient.z, heading_gradient);

        let motors = [robot.swivel, robot.link1, robot.link2, robot.gripper];

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
                let joint_trans_gradient = -toward_gripper.cross(&rot_axis_global);
                let joint_heading = preferred_rotation.dot(&rot_axis_global);

                Vector4::new(joint_trans_gradient.x, joint_trans_gradient.y, joint_trans_gradient.z, joint_heading)
            })
            .collect::<Vec<_>>();

        let motor_speeds = Matrix4::from_columns(motor_gradients.as_slice())
            .lu()
            .solve(&combined_gradient);

        // let motor_speeds = Matrix3x4::from_columns(motor_gradients.as_slice())
        //     .svd()
        //     .solve(&distance_gradient);
        //
        // let motor_speeds = Some(Vector4::new(0.0,0.0,0.0,0.0));

        let final_target_speeds = if let Some(speeds) = motor_speeds {
            if speeds.norm() > 1.0 {
                speeds.normalize()
            } else {
                speeds
            }
        } else {
            println!("Haha, motor go brrr!");
            let mut rng = rand::thread_rng();
            Vector4::new(
                rng.gen_range(-0.1..0.1),
                rng.gen_range(-0.1..0.1),
                rng.gen_range(-0.1..0.1),
                rng.gen_range(-0.1..0.1),
            )
        };

        JointVelocities {
            swivel: final_target_speeds[0],
            link1: final_target_speeds[1],
            link2: final_target_speeds[2],
            gripper: final_target_speeds[3],
            finger_0: 0.5,
            finger_1: 0.5,
            finger_2: 0.5,
            finger_0_2: 0.5,
            finger_1_2: 0.5,
            finger_2_2: 0.5,
        }
    }
}

