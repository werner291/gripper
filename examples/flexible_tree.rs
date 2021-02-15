
extern crate gripper_experiment;

use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::f32::consts::PI;
use std::iter::IntoIterator;
use std::marker::Sized;
use std::option::Option::{None, Some};
use std::option::Option;
use std::rc::Rc;
use std::vec::Vec;

use float_ord::FloatOrd;
use kiss3d::resource::Mesh;
use nalgebra::{distance_squared, Isometry3, Point3, Translation3, Vector3};
use nphysics3d::algebra::Velocity3;
use nphysics3d::joint::{BallConstraint, RevoluteJoint};
use nphysics3d::ncollide3d::shape::{Ball, ShapeHandle};
use nphysics3d::object::{Body, BodyPartHandle, ColliderDesc, DefaultBodyHandle, FEMVolume, FEMVolumeDesc, RigidBody, RigidBodyDesc};

use gripper_experiment::{robot, spawn_utilities};
use gripper_experiment::robot::{ArmJointMap, JointMap, JointVelocities, RobotBodyPartIndex};
use gripper_experiment::graphics::Graphics;
use gripper_experiment::kinematics::KinematicModel;
use gripper_experiment::multibody_util::*;
use gripper_experiment::inverse_kinematics;
use gripper_experiment::physics::PhysicsWorld;
use gripper_experiment::control_strategies::to_angles;
use gripper_experiment::simulator_thread::{run_synced_to_graphics, snapshot_physics, apply_motor_speeds};
use std::convert::{Into, TryInto};
use gripper_experiment::inverse_kinematics::BestAttempt;

fn make_branch_mesh() {
    let branch_origin = Point3::new(-2.0, 2.0, 2.0);

    // let branch_direction = Vector3::new(1.0, 0.0, 0.0);
    //
    // let branch_points = (0..10).flat_map(|i| {
    //
    //     let ring_center = branch_origin + branch_direction * i as f32 * 0.1;
    //
    //     (0..10).map(|j| {
    //         let theta = 2.0 * PI * j as f32 / 10.0;
    //         ring_center + Vector3::new(theta.cos(), 0.0, theta.sin()) * 0.5
    //     }).collect::<Vec<_>>()
    //
    // }).collect();
}

fn main() {
    let mut p = PhysicsWorld::new();
    let mut g = Graphics::init();

    let (mut fem_sn, branch_bh) = spawn_utilities::spawn_flexible_rod(&mut p, &mut g, Point3::new(-5.0, 8.0, 2.0));

    fem_sn.set_color(0.5,0.25,0.125);

    let branch_body = p.bodies.get(branch_bh).unwrap().downcast_ref::<FEMVolume<f32>>().unwrap();

    let target_attachment_point = Point3::new(1.0,  1.0, 2.0);


    let attach_to_cell = bodypart_closest_to_point(branch_body, &target_attachment_point);
    let cell_middle = branch_body.world_point_at_material_point(branch_body.part(attach_to_cell).unwrap(), &Point3::new(0.0, 0.0, 0.0));

    let (mut apple_sn, apple_bh) = spawn_utilities::spawn_ball(&mut p, &mut g, 0.35, cell_middle + Vector3::new(0.0, -1.0, 0.0));
    apple_sn.set_color(1.0,0.0,0.0);

    let mut stem = BallConstraint::new(BodyPartHandle(apple_bh, 0),
                                       BodyPartHandle(branch_bh, attach_to_cell),
                                       Point3::new(0.0, 1.0, 0.0),
                                       Point3::new(0.0, 0.0, 0.0));

    stem.set_break_force(100.0);

    p.joint_constraints.insert(stem);

    let robot = robot::make_robot(&mut p, &mut g);

    let k = robot::kinematic_model_from_robot(&mut p, &robot);

    run_synced_to_graphics(g, p, move |p| {
        let target_angles = angles_to_grab_apple(p, apple_bh, &robot, &k);
        let arm_target_velocities = to_angles::joint_velocities_towards_angles(&target_angles, &p, &robot);
        let target_velocities = JointVelocities::from_arm_and_finger(arm_target_velocities, robot::FINGERS_OPEN);

        apply_motor_speeds(&robot, p, target_velocities);
    });
}

fn angles_to_grab_apple(p: &mut PhysicsWorld, apple_bh: DefaultBodyHandle, robot: &RobotBodyPartIndex, k: &KinematicModel) -> ArmJointMap<f32> {
    let joint_angles = [robot.swivel, robot.link1, robot.link2, robot.gripper].iter().map(|bph| robot::revolute_joint_angle(&p, *bph).unwrap()).collect::<Vec<_>>();

    let plan_result = inverse_kinematics::gradient_guided_planner(&k,
                                                                  &(p.bodies.rigid_body(apple_bh).unwrap().position() * Point3::new(0.0, 0.0, 0.0)),
                                                                  &Vector3::new(0.0, 1.0, 0.0),
                                                                  100, 100,
                                                                  Some(&joint_angles)
    );
    let target_angles: ArmJointMap<f32> = plan_result.unwrap_or_else(|e: BestAttempt| {
        eprintln!("Inverse kinematics solver failed, using best attempt with remaining distance {}", e.remaining_distance);
        e.angles
    }).as_slice().try_into().unwrap();
    target_angles
}

fn bodypart_closest_to_point(branch_body: &FEMVolume<f32>, target_attachment_point: &Point3<f32>) -> usize {
    (0..branch_body.num_parts()).min_by_key(|i| {
        // FEMVolume's BodyPart unfortunately have a lot of unimplemented methods.
        let cell_middle = branch_body.world_point_at_material_point(branch_body.part(*i).unwrap(), &Point3::new(0.0, 0.0, 0.0));
        FloatOrd(distance_squared(&cell_middle, &target_attachment_point))
    }).unwrap()
}