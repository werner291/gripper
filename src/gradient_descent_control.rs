use std::iter::Iterator;

use kiss3d::ncollide3d::na::{distance_squared, Point3, Unit, Vector3};
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::BodyPart;
use rand::Rng;

use crate::physics::PhysicsWorld;
use crate::robot::{get_multibody_link, GripperDirection, RobotBodypartIndex};
use crate::robot;
use std::convert::From;

/// Controls the robot using a gradient-descent-based inverse kinematic controller.
/// Sets target motor speeds based on gradients of the distance of a point in front of the gripper.
pub fn gradient_descent_control(mut physics: &mut PhysicsWorld, robot: &RobotBodypartIndex, target: &Point3<f32>) -> f32 {
    let gripper_position = get_multibody_link(&physics, robot.gripper).unwrap().position();

    let gripper_pos : Point3<f32> = gripper_position * Point3::new(0.0, 1.5, 0.0);

    robot::set_gripper_direction(&mut physics, &robot, GripperDirection::Open);

    let motors = [robot.swivel, robot.link1, robot.link2, robot.gripper];

    let motor_gradients = motors.map(|bph| {
        let link = physics.bodies.multibody(robot.body).unwrap().link(bph.1).unwrap();
        let link_position = Point3::from(link.position().translation.vector);

        let toward_gripper = &gripper_pos - &link_position;
        let toward_ball = target - link_position;

        let ideal_rotation = toward_gripper.cross(&toward_ball);

        let rot_axis_local: &Unit<Vector3<f32>> = &link.joint().downcast_ref::<RevoluteJoint<f32>>().unwrap().axis();
        let rot_axis_global: Unit<Vector3<f32>> = link.position().rotation * rot_axis_local;

        let d = ideal_rotation.dot(&rot_axis_global) / 1.5;

        d * d * d
    });


    let mut rng = rand::thread_rng();

    let overall_size = motor_gradients.iter().map(|x| x.abs()).sum::<f32>();

    // dbg!(&motor_gradients);

    let remaining_distance = distance_squared(&target, &gripper_pos);

    if overall_size == 0.0 && remaining_distance > 0.1 {
        for (bph, g) in motors.iter().zip(motor_gradients.iter()) {
            robot::set_motor_speed(&mut physics, *bph, rng.gen_range(-0.01..0.01));
        }
    } else if overall_size > 0.01 {
        for (bph, g) in motors.iter().zip(motor_gradients.iter()) {
            robot::set_motor_speed(&mut physics, *bph, g / overall_size);
        }
    } else {
        for (bph, g) in motors.iter().zip(motor_gradients.iter()) {
            robot::set_motor_speed(&mut physics, *bph, *g);
        }
    }

    remaining_distance
}
