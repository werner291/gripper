use crate::physics::PhysicsWorld;
use crate::simulator_thread::ControllerStrategy;

use crate::robot::{
    set_gripper_direction, GripperDirection, JointVelocities, RobotBodyPartIndex, NUM_CHANNELS,
};

/// A dummy controller that simply sends a sine wave into the motor speed controllers.
pub(crate) struct FlailController {
    t: f32,
}

impl FlailController {
    pub fn new() -> Self {
        FlailController { t: 0.0 }
    }
}

impl ControllerStrategy for FlailController {
    fn apply_controller(
        &mut self,
        _physics: &PhysicsWorld,
        _robot: &RobotBodyPartIndex,
    ) -> JointVelocities {
        self.t += 0.05;
        JointVelocities {
            swivel: (self.t + 0.0).sin(),
            link1: (self.t + 1.0).sin(),
            link2: (self.t + 2.0).sin(),
            gripper: (self.t + 3.0).sin(),
            finger_0: (self.t + 4.0).sin(),
            finger_1: (self.t + 5.0).sin(),
            finger_2: (self.t + 6.0).sin(),
            finger_0_2: (self.t + 7.0).sin(),
            finger_1_2: (self.t + 8.0).sin(),
            finger_2_2: (self.t + 9.0).sin(),
        }
    }
}
