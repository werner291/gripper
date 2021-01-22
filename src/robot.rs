use std::string::ToString;

use kiss3d::ncollide3d::na::{Isometry3, Unit, Vector3, Rotation3};
use nphysics3d::joint::{FixedJoint, RevoluteJoint};
use nphysics3d::object::{DefaultBodyHandle, MultibodyDesc, DefaultBodyPartHandle, BodyPartHandle, Body};

use crate::physics::PhysicsWorld;
use std::option::Option::None;

pub struct RobotBodypartIndex {
    pub body: DefaultBodyHandle,
    pub base: DefaultBodyPartHandle,
    pub swivel: DefaultBodyPartHandle,
    pub link1: DefaultBodyPartHandle,
    pub link2: DefaultBodyPartHandle,
    pub gripper: DefaultBodyPartHandle,
    pub finger_0: DefaultBodyPartHandle,
    pub finger_1: DefaultBodyPartHandle,
    pub finger_2: DefaultBodyPartHandle,
    pub finger_0_2: DefaultBodyPartHandle,
    pub finger_1_2: DefaultBodyPartHandle,
    pub finger_2_2: DefaultBodyPartHandle
}

pub fn make_robot(physics: &mut PhysicsWorld) -> RobotBodypartIndex {
// Carriage body part
    let mut joint = FixedJoint::new(Isometry3::identity());

    let mut base = MultibodyDesc::new(joint).name("base".to_string()).mass(1.0);

    let mut swivel = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::y()), 0.0);
        joint.enable_angular_motor();
        joint.set_desired_angular_motor_velocity(1.0);
        joint.set_max_angular_motor_torque(1.0);
        base.add_child(joint).set_name("swivel".to_string()).set_parent_shift(Vector3::new(0.0, 0.25, 0.0))
    };

    let mut link1 = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::x()), 0.0);
        joint.enable_angular_motor();
        joint.set_max_angular_motor_torque(1.0);
        joint.enable_min_angle(- std::f32::consts::FRAC_PI_2);
        joint.enable_max_angle(std::f32::consts::FRAC_PI_2);
        swivel.add_child(joint).set_name("link1".to_string()).set_parent_shift(Vector3::new(0.0, 1.25, 0.0))
    };

    let mut link2 = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::x()), 0.0);
        joint.enable_angular_motor();
        joint.set_max_angular_motor_torque(1.0);
        joint.enable_min_angle(-std::f32::consts::FRAC_PI_2);
        joint.enable_max_angle(std::f32::consts::FRAC_PI_2);
        link1.add_child(joint).set_name("link2".to_string()).set_parent_shift(Vector3::new(0.0, 2.5, 0.0))
    };

    let mut gripper = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::x()), 0.0);
        joint.enable_angular_motor();
        joint.set_max_angular_motor_torque(1.0);
        joint.enable_min_angle(-std::f32::consts::FRAC_PI_2);
        joint.enable_max_angle(std::f32::consts::FRAC_PI_2);
        link2.add_child(joint).set_name("gripper".to_string()).set_parent_shift(Vector3::new(0.0, 2.5, 0.0))
    };

    for i in 0..3 {
        let theta = 2.0 * std::f32::consts::PI * (i as f32) / 3.0;
        dbg!(theta);
        let rot = Rotation3::new(Vector3::new(0.0, theta, 0.0));

        let mut phalanx = {
            let mut joint = RevoluteJoint::new(Unit::new_normalize(&rot * Vector3::x() * 0.3), 0.0);
            joint.enable_angular_motor();
            joint.set_max_angular_motor_torque(1.0);
            joint.enable_min_angle(0.0);
            joint.enable_max_angle(std::f32::consts::FRAC_PI_2);

            gripper.add_child(joint).set_name(format!("finger_{}", i)).set_parent_shift(
                &rot * Vector3::new(0.0, 0.5, 0.3)
            )
        };

        {
            let mut joint = RevoluteJoint::new(Unit::new_normalize(&rot * Vector3::x() * 0.3), 0.0);
            joint.enable_angular_motor();
            joint.set_max_angular_motor_torque(0.5);
            joint.enable_min_angle(-0.4);
            joint.enable_max_angle(std::f32::consts::FRAC_PI_2);

            phalanx.add_child(joint).set_name(format!("finger_{}_2", i)).set_parent_shift(
                Vector3::new(0.0, 0.5, 0.0)
            )
        };
    }

    let mb = physics.bodies.insert(base.build());

    let mut robot = physics.bodies.multibody_mut(mb).unwrap();

    robot.set_deactivation_threshold(None);

    RobotBodypartIndex {
        body: mb,
        base: BodyPartHandle(mb, robot.links_with_name("base").next().unwrap().0),
        swivel: BodyPartHandle(mb, robot.links_with_name("swivel").next().unwrap().0),
        link1: BodyPartHandle(mb, robot.links_with_name("link1").next().unwrap().0),
        link2: BodyPartHandle(mb, robot.links_with_name("link2").next().unwrap().0),
        gripper: BodyPartHandle(mb, robot.links_with_name("gripper").next().unwrap().0),
        finger_0: BodyPartHandle(mb, robot.links_with_name("finger_0").next().unwrap().0),
        finger_1: BodyPartHandle(mb, robot.links_with_name("finger_1").next().unwrap().0),
        finger_2: BodyPartHandle(mb, robot.links_with_name("finger_2").next().unwrap().0),
        finger_0_2: BodyPartHandle(mb, robot.links_with_name("finger_0_2").next().unwrap().0),
        finger_1_2: BodyPartHandle(mb, robot.links_with_name("finger_1_2").next().unwrap().0),
        finger_2_2: BodyPartHandle(mb, robot.links_with_name("finger_2_2").next().unwrap().0)
    }
}
