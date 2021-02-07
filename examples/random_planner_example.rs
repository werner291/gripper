//
// extern crate gripper_experiment;
//
// use gripper_experiment::graphics::Graphics;
// use gripper_experiment::physics::PhysicsWorld;
// use gripper_experiment::{robot, simulation};
// use gripper_experiment::spawn_utilities::{make_ground, make_pinned_ball};
// use gripper_experiment::control_strategies::random_tree_planner::{ApproachAngles, plan_random_tree};
// use std::boxed::Box;
// use nalgebra::{Vector3, Point3};
//
// fn main() {
//     let mut graphics = Graphics::init();
//
//     let mut physics = PhysicsWorld::new();
//
//     let robot = robot::make_robot(&mut physics, &mut graphics);
//
//     make_ground(&mut physics, &mut graphics);
//
//     let (_, ball_bh) = make_pinned_ball(&mut physics, &mut graphics);
//
//     let target_pos = Point3::new(3.0, 1.0, 0.0);
//     let target_heading = Vector3::new(0.0,-1.0,0.0);
//
//     let ctrl = ApproachAngles {
//         angles: plan_random_tree(&physics, &robot, &target_pos, &target_heading)
//     };
//
//     dbg!(&ctrl.angles);
//
//     simulation::run_synced_to_graphics(&mut graphics, physics, robot, Box::new(ctrl))
// }
fn main() {}