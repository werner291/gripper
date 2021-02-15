use nphysics3d::joint::RevoluteJoint;

use crate::control_strategies::ControllerStrategy;
use crate::multibody_util::get_joint;
use crate::physics::PhysicsWorld;
use crate::robot::{ArmJointMap, ArmJointVelocities, FINGERS_OPEN, JointVelocities, RobotBodyPartIndex};

/// A ControllerStrategy implementation that calls [`joint_velocities_towards_angles`] with a constant set of angles.
pub struct ApproachAngles {
    pub angles: ArmJointMap<f32>
}

impl ControllerStrategy for ApproachAngles {
    fn apply_controller(&mut self, physics_world: &PhysicsWorld, robot: &RobotBodyPartIndex) -> JointVelocities {
        let jv = joint_velocities_towards_angles(&self.angles, physics_world, robot);
        dbg!(&jv);
        JointVelocities::from_arm_and_finger(jv, FINGERS_OPEN)
    }
}

/// A controller that sets target velocities such that the joints approach the given target angles.
///
/// The target velocity is the cube of the leftover signed angle, with motor speeds limited to a
/// rotation speed of 1 (other joint speeds scaled down proportionally).
pub fn joint_velocities_towards_angles(target_angles : &ArmJointMap<f32>, physics_world: &PhysicsWorld, robot: &RobotBodyPartIndex) -> ArmJointVelocities {

    let arm_joint_angles = ArmJointMap {
        swivel: get_joint::<RevoluteJoint<f32>>(physics_world, robot.swivel).unwrap().angle(),
        link1: get_joint::<RevoluteJoint<f32>>(physics_world, robot.link1).unwrap().angle(),
        link2: get_joint::<RevoluteJoint<f32>>(physics_world, robot.link2).unwrap().angle(),
        gripper: get_joint::<RevoluteJoint<f32>>(physics_world, robot.gripper).unwrap().angle()
    };

    ArmJointVelocities {
        swivel: (target_angles.swivel - arm_joint_angles.swivel) * 10.0,//.powi(3),
        link1: (target_angles.link1 - arm_joint_angles.link1) * 10.0,//.powi(3),
        link2: (target_angles.link2 - arm_joint_angles.link2) * 10.0,//.powi(3),
        gripper: (target_angles.gripper - arm_joint_angles.gripper) * 10.0,//.powi(3),
    }.limit_to_safe(0.5)
}

