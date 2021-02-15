extern crate gripper_experiment;

use std::borrow::BorrowMut;
use std::cell::RefCell;
use std::clone::Clone;
use std::convert::{Into, TryInto};
use std::f32::consts::PI;
use std::iter::IntoIterator;
use std::marker::Sized;
use std::option::Option;
use std::option::Option::{None, Some};
use std::rc::Rc;
use std::vec::Vec;

use float_ord::FloatOrd;
use kiss3d::resource::Mesh;
use kiss3d::scene::SceneNode;
use nalgebra::{distance_squared, Isometry3, Point3, Translation3, Vector3};
use nphysics3d::algebra::Velocity3;
use nphysics3d::joint::{BallConstraint, RevoluteJoint};
use nphysics3d::ncollide3d::shape::{Ball, ShapeHandle};
use nphysics3d::object::{
    Body, BodyPartHandle, BodySet, BodyStatus, ColliderDesc, DefaultBodyHandle,
    DefaultColliderHandle, FEMVolume, FEMVolumeDesc, RigidBody, RigidBodyDesc,
};

use gripper_experiment::control_strategies::gradient_descent_control::grabbed;
use gripper_experiment::control_strategies::{to_angles, ControllerStrategy};
use gripper_experiment::graphics::Graphics;
use gripper_experiment::inverse_kinematics;
use gripper_experiment::inverse_kinematics::BestAttempt;
use gripper_experiment::kinematics::KinematicModel;
use gripper_experiment::multibody_util::*;
use gripper_experiment::physics::PhysicsWorld;
use gripper_experiment::robot::joint_map::{ArmJointMap, JointMap, JointVelocities};
use gripper_experiment::robot::{joint_map, spawn, GripperDirection, RobotBodyPartIndex};
use gripper_experiment::simulator_thread::{
    apply_motor_speeds, run_synced_to_graphics, snapshot_physics,
};
use gripper_experiment::{robot, spawn_utilities};

use crate::ControllerState::{BringingToTarget, Grabbing, Idle};

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

    let (mut fem_sn, branch_bh) =
        spawn_utilities::spawn_flexible_rod(&mut p, &mut g, Point3::new(-5.0, 8.0, 2.0));

    fem_sn.set_color(0.5, 0.25, 0.125);

    let apple_bh = spawn_apple_on_branch(&mut p, &mut g, branch_bh, &Point3::new(1.0, 1.0, 2.0));

    let robot = spawn::make_robot(&mut p, &mut g);

    let (target_sn, target_bh, target_ch) =
        add_apple_collection_target(&mut p, &mut g, Point3::new(-2.0, 0.0, -2.0));
    let receptacle_position =
        p.bodies.rigid_body(target_bh).unwrap().position() * Point3::new(0.0, 0.0, 0.0);

    let mut controller = ApplePickerController::new(apple_bh, receptacle_position, &p, &robot);

    run_synced_to_graphics(g, p, move |p| {
        apply_motor_speeds(&robot, p, controller.apply_controller(p, &robot));
    });
}

enum ControllerState {
    Idle,
    Approaching { motion_plan: ArmJointMap<f32> },
    Grabbing,
    BringingToTarget { motion_plan: ArmJointMap<f32> },
}

struct ApplePickerController {
    state: ControllerState,
    apple_bh: DefaultBodyHandle,
    receptacle_position: Point3<f32>,
    kinematic_model: KinematicModel,
}

impl ApplePickerController {
    fn new(
        apple_bh: DefaultBodyHandle,
        receptacle_position: Point3<f32>,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> Self {
        Self {
            state: ControllerState::Idle,
            apple_bh,
            receptacle_position,
            kinematic_model: robot::kinematic_model_from_robot(physics_world, &robot),
        }
    }

    fn make_motion_plan(
        &self,
        robot: &RobotBodyPartIndex,
        p: &PhysicsWorld,
        target_position: &Point3<f32>,
    ) -> ArmJointMap<f32> {
        let joint_angles: [f32; 4] = robot::arm_joint_angles(&p, &robot).into();

        let plan_result = inverse_kinematics::gradient_guided_planner(
            &self.kinematic_model,
            target_position,
            &Vector3::new(0.0, 1.0, 0.0),
            100,
            100,
            Some(&joint_angles),
        );

        let target_angles: ArmJointMap<f32> = plan_result.unwrap_or_else(|e: BestAttempt| {
            eprintln!("Inverse kinematics solver failed, using best attempt with remaining distance {}", e.remaining_distance);
            e.angles
        }).as_slice().try_into().unwrap();

        target_angles
    }

    // motion_plan: ApplePickerController::make_motion_plan(robot, physics_world),
}

impl ControllerStrategy for ApplePickerController {
    fn apply_controller(
        &mut self,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) -> JointVelocities {
        self.apply_state_transitions(physics_world, robot);

        self.compute_joint_velocities(robot, physics_world)
    }
}

fn add_apple_collection_target(
    p: &mut PhysicsWorld,
    g: &mut Graphics,
    middle_point: Point3<f32>,
) -> (SceneNode, DefaultBodyHandle, DefaultColliderHandle) {
    const RADIUS: f32 = 0.5;

    let rb = RigidBodyDesc::new()
        .translation(middle_point.coords)
        .status(BodyStatus::Static)
        .build();

    let ball = p.bodies.insert(rb);
    let ball_ch = p.colliders.insert(
        ColliderDesc::new(ShapeHandle::new(Ball::new(RADIUS)))
            .sensor(true)
            .build(BodyPartHandle(ball, 0)),
    );
    let mut ball_sn = g.window.add_sphere(RADIUS);
    g.bp_to_sn.push((ball_sn.clone(), BodyPartHandle(ball, 0)));

    ball_sn.set_color(1.0, 0.0, 1.0);

    (ball_sn, ball, ball_ch)
}

fn spawn_apple_on_branch(
    mut p: &mut PhysicsWorld,
    mut g: &mut Graphics,
    branch_bh: DefaultBodyHandle,
    target_attachment_point: &Point3<f32>,
) -> DefaultBodyHandle {
    let branch_body = p
        .bodies
        .get(branch_bh)
        .unwrap()
        .downcast_ref::<FEMVolume<f32>>()
        .unwrap();
    let attach_to_cell = bodypart_closest_to_point(branch_body, &target_attachment_point);
    let cell_middle = branch_body.world_point_at_material_point(
        branch_body.part(attach_to_cell).unwrap(),
        &Point3::new(0.0, 0.0, 0.0),
    );
    let apple_position = cell_middle + Vector3::new(0.0, -1.0, 0.0);

    let (mut apple_sn, apple_bh) =
        spawn_utilities::spawn_ball(&mut p, &mut g, 0.35, apple_position);
    apple_sn.set_color(1.0, 0.0, 0.0);

    let mut stem = BallConstraint::new(
        BodyPartHandle(apple_bh, 0),
        BodyPartHandle(branch_bh, attach_to_cell),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 0.0),
    );

    stem.set_break_force(100.0);

    p.joint_constraints.insert(stem);

    apple_bh
}

fn bodypart_closest_to_point(
    branch_body: &FEMVolume<f32>,
    target_attachment_point: &Point3<f32>,
) -> usize {
    (0..branch_body.num_parts())
        .min_by_key(|i| {
            // FEMVolume's BodyPart unfortunately have a lot of unimplemented methods.
            let cell_middle = branch_body.world_point_at_material_point(
                branch_body.part(*i).unwrap(),
                &Point3::new(0.0, 0.0, 0.0),
            );
            FloatOrd(distance_squared(&cell_middle, &target_attachment_point))
        })
        .unwrap()
}

impl ApplePickerController {
    fn apply_state_transitions(
        &mut self,
        physics_world: &PhysicsWorld,
        robot: &RobotBodyPartIndex,
    ) {
        let joint_angles: [f32; 4] = robot::arm_joint_angles(&physics_world, &robot).into();
        let tip_position =
            self.kinematic_model.predict(&joint_angles).tip_position * Point3::new(0.0, 0.0, 0.0);

        match self.state {
            ControllerState::Idle => {
                if let Some(apple) = physics_world.bodies.rigid_body(self.apple_bh) {
                    let apple_position = apple.position() * Point3::new(0.0, 0.0, 0.0);

                    self.state = ControllerState::Approaching {
                        motion_plan: self.make_motion_plan(robot, physics_world, &apple_position),
                    };
                }
            }
            ControllerState::Approaching { .. } => {
                let apple_position = physics_world
                    .bodies
                    .rigid_body(self.apple_bh)
                    .expect("Apple should be in the physics world when approaching.")
                    .position()
                    * Point3::new(0.0, 0.0, 0.0);
                if distance_squared(&tip_position, &apple_position) < 0.01 {
                    self.state = Grabbing;
                }
            }
            ControllerState::Grabbing => {
                if grabbed(physics_world, robot, self.apple_bh) {
                    self.state = BringingToTarget {
                        motion_plan: self.make_motion_plan(
                            robot,
                            physics_world,
                            &self.receptacle_position,
                        ),
                    };
                }
            }
            ControllerState::BringingToTarget { .. } => {
                if distance_squared(&tip_position, &self.receptacle_position) < 0.01 {
                    self.state = Idle;
                }
            }
        }
    }
}

impl ApplePickerController {
    fn compute_joint_velocities(
        &mut self,
        robot: &RobotBodyPartIndex,
        physics_world: &PhysicsWorld,
    ) -> JointMap<f32> {
        match self.state {
            Idle => JointVelocities {
                swivel: 0.0,
                link1: 0.0,
                link2: 0.0,
                gripper: 0.0,
                finger_0: 0.0,
                finger_1: 0.0,
                finger_2: 0.0,
                finger_0_2: 0.0,
                finger_1_2: 0.0,
                finger_2_2: 0.0,
            },
            ControllerState::Approaching { motion_plan } => {
                let arm_target_velocities = to_angles::joint_velocities_towards_angles(
                    &motion_plan,
                    &physics_world,
                    &robot,
                );

                JointVelocities::from_arm_and_finger(arm_target_velocities, joint_map::FINGERS_OPEN)
            }
            Grabbing => JointVelocities::from_arm_and_finger(
                ArmJointMap {
                    swivel: 0.0,
                    link1: 0.0,
                    link2: 0.0,
                    gripper: 0.0,
                },
                joint_map::FINGERS_CLOSE,
            ),
            ControllerState::BringingToTarget { motion_plan } => {
                let arm_target_velocities = to_angles::joint_velocities_towards_angles(
                    &motion_plan,
                    &physics_world,
                    &robot,
                );

                JointVelocities::from_arm_and_finger(
                    arm_target_velocities,
                    joint_map::FINGERS_CLOSE,
                )
            }
        }
    }
}
