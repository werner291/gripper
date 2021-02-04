use std::iter::Iterator;

use nalgebra::{distance_squared, Point3, Unit, Vector3, Vector4, Vector6};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::{BodyPart, DefaultBodyHandle, DefaultBodyPartHandle};
use rand::Rng;

use crate::physics::PhysicsWorld;

use crate::robot::{get_multibody_link, JointVelocities, RobotBodyPartIndex, ArmJointVelocities, ArmJointMap, GripperDirection, multibody_link_position};
use nalgebra::{Matrix3x4, Matrix4x3, Matrix4, Matrix6x4, Matrix4x6};
use std::convert::From;
use std::option::Option::Some;

use std::vec::Vec;

use crate::simulator_thread::ControllerStrategy;
use crate::control_strategies::gradient_descent_control::SphereGrabState::{Approaching, Grabbing};
use crate::robot::GripperDirection::{Open, Closed};
use k::joint::Velocity;
use nphysics3d::algebra::Velocity3;
use std::option::Option;

enum SphereGrabState {
    Approaching,
    Grabbing,
}

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub(crate) struct GradientDescentController {
    target: DefaultBodyHandle,
    heading_vector: Unit<Vector3<f32>>,
    state: SphereGrabState
}

impl GradientDescentController {
    pub fn new(target: DefaultBodyHandle, heading_vector: Unit<Vector3<f32>>) -> Self {
        GradientDescentController { target, heading_vector, state: Approaching }
    }
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
            Approaching => {

                // The ball sits on the ground, so we want the gripper to be pointing down.
                let target_heading = Unit::new_unchecked(Vector3::new(0.0, -1.0, 0.0));

                let gripper = get_multibody_link(&physics, robot.gripper).unwrap();

                // Get the current forward vector of the gripper, which we'd like to eventually match target_heading.
                let gripper_forward: Vector3<f32> = gripper.position() * Vector3::new(0.0, 1.0, 0.0);

                // Get a point, in global coordinates, inside the gripper.
                // This roughly corresponds to where the center of the sphere will be once grasped.
                let point_inside_gripper = gripper.position() * Point3::new(0.0, 1.0, 0.0);

                // Vector by which to translate the reference point to the center of the target.
                let translation_delta = target_position - point_inside_gripper;
                // Angle-axis rotation necessary to make the gripper point the right way.
                let rotation_delta = -gripper_forward.cross(&target_heading.into_inner());

                // Compute the remaining distance and angle
                let distance_remaining = translation_delta.norm();
                let angle_remaining = rotation_delta.norm(); // FIXME sign?

                // Compute the desired velocity to bring the gripper into position.
                let target_velocity_at_point = Velocity3::new(
                    if distance_remaining > 1.0 { translation_delta / distance_remaining } else { translation_delta },
                    2.0 * rotation_delta
                );

                // Compute the necessary joint velocities.
                // If no solution, apply random velocities to try to break gimbal lock.
                let jv =
                    joint_velocities_for_velocity_at_point_and_angular_velocity(physics, robot, &point_inside_gripper, &target_velocity_at_point)
                        .unwrap_or_else(|| GradientDescentController::random_arm_velocities())
                        //The solver can sometimes return solutions that are a teensy bit excessive.
                        .limit_to_safe(10.0);

                // If the remaining distance is smaller than a threshold, go to state Grabbing.
                // The robot will act upon this state next turn.
                if distance_remaining + angle_remaining < 0.05 {
                    self.state = Grabbing;
                }

                // Combine the requested arm joint velocities with finger joint velocities
                // to open the gripper, then return the velocities for execution.
                GradientDescentController::joint_velocities_with_gripper(jv, Open)
            }
            Grabbing => {
                GradientDescentController::joint_velocities_with_gripper(ArmJointVelocities {
                    swivel: 0.0,
                    link1: 0.0,
                    link2: 0.0,
                    gripper: 0.0
                }, Closed)
            }
        }

    }
}


impl GradientDescentController {
    fn joint_velocities_with_gripper(jv: ArmJointVelocities, gripper_direction: GripperDirection) -> JointVelocities {
        let finger_v = match gripper_direction {
            GripperDirection::Open => 0.5,
            GripperDirection::Closed => -0.5
        };

        JointVelocities {
            swivel: jv.swivel,
            link1: jv.link1,
            link2: jv.link2,
            gripper: jv.gripper,
            finger_0: finger_v,
            finger_1: finger_v,
            finger_2: finger_v,
            finger_0_2: finger_v,
            finger_1_2: finger_v,
            finger_2_2: finger_v,
        }
    }
}


///
/// Indeed, the method name is a bit of a mouthful. Basically, this method is the core of
/// the inverse kinematics solver.
///
/// Basically, given a point in global space and a velocity, this method computes
/// the joint velocities of the robotic arm such that that point, at constant position
/// relative to the end effector, moves at the specified velocity.
///
/// Returns None if the requested velocity cannot be achieved by the joints.
///
fn joint_velocities_for_velocity_at_point_and_angular_velocity(
    physics: &PhysicsWorld, robot: &RobotBodyPartIndex,
    at_point: &Point3<f32>,
    velocity: &Velocity3<f32>) -> Option<ArmJointVelocities> {

    let motor_gradients =
        // List of all body parts corresponding to all MultibodyLinks that have a joint that affects the gripper position.
        [robot.swivel, robot.link1, robot.link2, robot.gripper]
        .iter()
        .map(|bph| {

            // Retrieve the position and rotation of the body link being considered.
            let link = physics
                .bodies
                .multibody(robot.body)
                .unwrap()
                .link(bph.1)
                .unwrap();

            // Global point at the center of the joint.
            let link_position = link.position() * Point3::new(0.0, 0.0, 0.0);

            // A vector from the center of the joint to the point inside the gripper.
            let toward_gripper = at_point - &link_position;

            // Extract the global rotation axis of this joint.
            let rot_axis: Unit<Vector3<f32>> = link.position() * link
                .joint()
                .downcast_ref::<RevoluteJoint<f32>>()
                .unwrap()
                .axis();

            // Linear velocity of the gripper if the current joint was rotating at unit velocity, all others immobile.
            let joint_trans_gradient = -toward_gripper.cross(&rot_axis);

            // Angular velocity of the gripper, around the axis of the target velocity, if the current joint was rotating
            // at unit velocity, all others immobile. Unfortunately, we need to do this since the robot arm has only 4 DoF.
            let joint_rot_gradient = -velocity.angular.normalize().dot(&rot_axis);

            // Assemble in a Vector4.
            Vector4::new(joint_trans_gradient.x, joint_trans_gradient.y, joint_trans_gradient.z, joint_rot_gradient)
        })
        .collect::<Vec<_>>();

    let target_vector = Vector4::new(
        velocity.linear.x,
        velocity.linear.y,
        velocity.linear.z,
        // Squashes the angular velocity down to a magnitude, since the robotic arm only has 4 DoF.
        // TODO: Find out if I can just have the solver return None for impossible rotations.
        velocity.angular.norm()
    );

    // Combine the vectors into a matrix, then solve.
    Matrix4::from_columns(motor_gradients.as_slice())
        .lu()
        .solve(&target_vector)
        .map(|motor_speeds| ArmJointVelocities {
            swivel: motor_speeds[0],
            link1: motor_speeds[1],
            link2: motor_speeds[2],
            gripper: motor_speeds[3]
        })
}

fn global_joint_axis(physics: &PhysicsWorld, bph : DefaultBodyPartHandle) -> Unit<Vector3<f32>> {
    // Retrieve the position and rotation of the body link being considered.
    let link = physics
        .bodies
        .multibody(bph.0)
        .unwrap()
        .link(bph.1)
        .unwrap();

    // Extract the rotation axis of this joint.
    let rot_axis_local: &Unit<Vector3<f32>> = &link
        .joint()
        .downcast_ref::<RevoluteJoint<f32>>()
        .unwrap()
        .axis();

    link.position().rotation * rot_axis_local
}

impl GradientDescentController {
    /// Generate some random arm arm joint velocities,
    /// useful to break gimbal lock conditions.
    fn random_arm_velocities() -> ArmJointVelocities {
        let mut rng = rand::thread_rng();
        ArmJointVelocities {
            swivel: rng.gen_range(-0.1..0.1),
            link1: rng.gen_range(-0.1..0.1),
            link2: rng.gen_range(-0.1..0.1),
            gripper: rng.gen_range(-0.1..0.1)
        }
    }
}
