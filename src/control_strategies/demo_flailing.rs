use std::iter::Iterator;

use nphysics3d::joint::RevoluteJoint;

use crate::physics::{PhysicsWorld, ControllerStrategy};
use crate::robot;
use crate::robot::{set_gripper_direction, GripperDirection, RobotBodyPartIndex};

/// A dummy controller that simply sends a sine wave into the motor speed controllers.
struct FlailController {
    t: f32
}

impl ControllerStrategy for FlailController {
    fn apply_controller(&mut self, physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex) {
        self.t += 0.05;
        for (i, bph) in robot.motor_parts().iter().enumerate() {
            let v = (self.t + i as f32).sin();
            let revjoint = robot::get_joint_mut::<RevoluteJoint<f32>>(physics, *bph).unwrap();
            revjoint.set_desired_angular_motor_velocity(v);
        }
    }
}

/// A dummy controller that opens and closes the gripper repeatedly.
struct GripperOpenCloseDemo {
    t: f32
}

impl ControllerStrategy for GripperOpenCloseDemo {
    fn apply_controller(&mut self, physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex) {
        self.t += 0.05;
        set_gripper_direction(
            physics,
            &robot,
            if (self.t / 5.0) as i64 % 2 == 0 {
                GripperDirection::Open
            } else {
                GripperDirection::Closed
            },
        );
    }
}