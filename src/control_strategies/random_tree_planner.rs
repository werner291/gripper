// use std::borrow::BorrowMut;
// use std::clone::Clone;
// use std::iter::Iterator;
// use std::ops::Range;
// use std::prelude::v1::Vec;
//
// use nalgebra::{Isometry3, Unit, Vector3, Point3};
// use nphysics3d::joint::RevoluteJoint;
// use nphysics3d::object::{BodyPart, DefaultBodyPartHandle};
// use rand::{Rng, thread_rng};
// use rand::prelude::ThreadRng;
//
// use crate::kinematics::KinematicModel;
// use crate::multibody_util::{get_joint, get_multibody_link};
// use crate::physics::PhysicsWorld;
// use crate::robot::{ArmJointMap, ArmJointVelocities, JointVelocities, RobotBodyPartIndex};
// use crate::robot::GripperDirection::Open;
// use crate::simulator_thread::ControllerStrategy;
// use float_ord::FloatOrd;
// use kiss3d::ncollide3d::na::distance_squared;
//
// pub struct ApproachAngles {
//     pub angles: ArmJointMap<f32>
// }
//
// impl ControllerStrategy for ApproachAngles {
//     fn apply_controller(&mut self, physics_world: &PhysicsWorld, robot: &RobotBodyPartIndex) -> JointVelocities {
//         let arm_joint_angles = ArmJointMap {
//             swivel: get_joint::<RevoluteJoint<f32>>(physics_world, robot.swivel).unwrap().angle(),
//             link1: get_joint::<RevoluteJoint<f32>>(physics_world, robot.link1).unwrap().angle(),
//             link2: get_joint::<RevoluteJoint<f32>>(physics_world, robot.link2).unwrap().angle(),
//             gripper: get_joint::<RevoluteJoint<f32>>(physics_world, robot.gripper).unwrap().angle()
//         };
//
//         let arm_joint_velocities = ArmJointVelocities {
//             swivel: (self.angles.swivel - arm_joint_angles.swivel).powi(3),
//             link1: (self.angles.link1 - arm_joint_angles.link1).powi(3),
//             link2: (self.angles.link2 - arm_joint_angles.link2).powi(3),
//             gripper: (self.angles.gripper - arm_joint_angles.gripper).powi(3),
//         }.limit_to_safe(1.0);
//
//         JointVelocities::joint_velocities_with_gripper(arm_joint_velocities, Open)
//     }
// }
//
//
// pub fn plan_random_tree(physics: &PhysicsWorld, robot: &RobotBodyPartIndex,
//                         target_position: &Point3<f32>, target_heading: &Vector3<f32>) -> ArmJointMap<f32> {
//
//     let km = KinematicModel::from_multibody(physics, &[robot.base, robot.swivel, robot.link1, robot.link2, robot.gripper]);
//
//     let starting_angles : Vec<f32> = [robot.swivel, robot.link1, robot.link2, robot.gripper].iter().map(|bph| {
//         get_joint::<RevoluteJoint<f32>>(physics, *bph).unwrap().angle()
//     }).collect();
//
//     let mut rng = thread_rng();
//
//     let angles = (0..10000).map(|_| {
//         let mut candidate = starting_angles.clone();
//         km.mutate_angles(&mut candidate, &mut rng, 2.1);
//         candidate
//     }).min_by_key(|c| FloatOrd({
//         evaluate_prediction(target_position, target_heading, &km, &c)
//     }))
//         .unwrap();
//
//     println!("DIST: {:?}", &km.predict(&angles).translation);
//
//     ArmJointMap {
//         swivel: angles[0],
//         link1: angles[1],
//         link2: angles[2],
//         gripper: angles[3]
//     }
// }
//
//
// fn gradient_prediction(target_position: &Point3<f32>, target_heading: &Vector3<f32>, km: &KinematicModel, angles: &[f32]) -> Vec<f32> {
//     let prediction = km.predict(c);
//     let prediction_pos = &prediction * Point3::new(0.0, 0.0, 0.0);
//     let prediction_heading = &prediction * Vector3::new(0.0, 1.0, 0.0);
//     distance_squared(&prediction_pos, target_position) + (target_heading - &prediction_heading).norm_squared()
// }
//
// fn iso_dist(a: &Isometry3<f32>, b: &Isometry3<f32>) -> f32 {
//
//         (&a.rotation.into_inner() - &b.rotation.into_inner()).norm_squared()
//             + (&a.translation.vector - &b.translation.vector).norm_squared()
// }