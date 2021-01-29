use std::iter::Iterator;

use nphysics3d::joint::RevoluteJoint;

use crate::physics::PhysicsWorld;
use crate::robot;
use crate::robot::{set_gripper_direction, GripperDirection, RobotBodyPartIndex};

pub fn control_gripper_demo(mut physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex, t: f32) {
    set_gripper_direction(
        &mut physics,
        &robot,
        if (t / 5.0) as i64 % 2 == 0 {
            GripperDirection::Open
        } else {
            GripperDirection::Closed
        },
    );
}

fn control_flailing_demo(mut physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex, t: f32) {
    for (i, bph) in robot.motor_parts().iter().enumerate() {
        let v = (t + i as f32).sin();
        let revjoint = robot::get_joint_mut::<RevoluteJoint<f32>>(&mut physics, *bph).unwrap();
        revjoint.set_desired_angular_motor_velocity(v);
    }
}
