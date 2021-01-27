use kiss3d::event::{Action, Key};
use kiss3d::window::Window;
use nphysics3d::joint::RevoluteJoint;
use nphysics3d::object::DefaultBodyPartHandle;

use crate::physics::PhysicsWorld;
use crate::robot;
use crate::robot::RobotBodypartIndex;

/// Allows for basic keyboard-based control of the robot.
///
/// Simply call once every frame with the Window, PhysicsWorld and RobotBodypartIndex,
/// and the robot's motors will be directly controllable by pressing keys.
///
/// See / modify this method's body to look up or change the key mapping if necessary.
pub fn control_robot_by_keys(
    window: &Window,
    mut physics: &mut PhysicsWorld,
    robot: &RobotBodypartIndex,
) {
    // FIXME store mapping in a datastructure instead.
    revjoint_control(&window, &mut physics, Key::Q, Key::A, robot.swivel);
    revjoint_control(&window, &mut physics, Key::W, Key::S, robot.link1);
    revjoint_control(&window, &mut physics, Key::E, Key::D, robot.link2);
    revjoint_control(&window, &mut physics, Key::R, Key::F, robot.gripper);
    revjoint_control(&window, &mut physics, Key::T, Key::G, robot.finger_0);
    revjoint_control(&window, &mut physics, Key::T, Key::G, robot.finger_1);
    revjoint_control(&window, &mut physics, Key::T, Key::G, robot.finger_2);
    revjoint_control(&window, &mut physics, Key::Y, Key::H, robot.finger_0_2);
    revjoint_control(&window, &mut physics, Key::Y, Key::H, robot.finger_1_2);
    revjoint_control(&window, &mut physics, Key::Y, Key::H, robot.finger_2_2);
}

/// Utility function that controls an individual motor with backwards/forwards keys.
fn revjoint_control(
    window: &Window,
    mut physics: &mut PhysicsWorld,
    back: Key,
    forward: Key,
    bph: DefaultBodyPartHandle,
) {
    robot::get_joint_mut::<RevoluteJoint<f32>>(&mut physics, bph)
        .unwrap()
        .set_desired_angular_motor_velocity(key_forward_backward(&window, back, forward));
}

/// Small utility function that computes a value between -1 and 1
/// based on whether the forward and backward keys are pressed.
fn key_forward_backward(window: &Window, back: Key, forward: Key) -> f32 {
    let a = match window.get_key(back) {
        Action::Release => 0.0,
        Action::Press => -1.0,
    };
    let b = match window.get_key(forward) {
        Action::Release => 0.0,
        Action::Press => 1.0,
    };
    a + b
}
