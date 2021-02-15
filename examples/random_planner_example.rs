extern crate gripper_experiment;

use std::boxed::Box;

use nalgebra::{Point3, Vector3};

use gripper_experiment::control_strategies::to_angles::ApproachAngles;
use gripper_experiment::control_strategies::ControllerStrategy;
use gripper_experiment::graphics::Graphics;
use gripper_experiment::inverse_kinematics::gradient_guided_planner;
use gripper_experiment::kinematics::KinematicModel;
use gripper_experiment::physics::PhysicsWorld;
use gripper_experiment::robot;
use gripper_experiment::robot::joint_map::ArmJointMap;
use gripper_experiment::robot::spawn;
use gripper_experiment::simulator_thread;
use gripper_experiment::spawn_utilities::{make_ground, make_pinned_ball};
use std::option::Option::None;

fn main() {
    let mut graphics = Graphics::init();

    let mut physics = PhysicsWorld::new();

    let robot = spawn::make_robot(&mut physics, &mut graphics);

    make_ground(&mut physics, &mut graphics);

    let (_, ball_bh) = make_pinned_ball(&mut physics, &mut graphics);

    let target_pos =
        physics.bodies.rigid_body(ball_bh).unwrap().position() * Point3::new(0.0, 0.0, 0.0);
    let target_heading = Vector3::new(0.0, -1.0, 0.0);

    let km = KinematicModel::from_multibody(
        &physics,
        robot.base,
        &[robot.swivel, robot.link1, robot.link2, robot.gripper],
        Vector3::new(0.0, 1.0, 0.0),
    );

    let inv_k = gradient_guided_planner(&km, &target_pos, &target_heading, 100, 1000, None).unwrap();

    let mut ctrl = ApproachAngles {
        angles: ArmJointMap {
            swivel: inv_k[0],
            link1: inv_k[1],
            link2: inv_k[2],
            gripper: inv_k[3],
        },
    };

    dbg!(&ctrl.angles);

    simulator_thread::run_synced_to_graphics(graphics, physics, move |physics| {
        simulator_thread::apply_motor_speeds(&robot, physics, ctrl.apply_controller(physics, &robot));
    });
}
