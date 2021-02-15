//! A software package containing useful code for robotics of various kinds.

pub mod control_strategies;
pub mod graphics;
pub mod inverse_kinematics;
pub mod kinematics;
pub mod load_mesh;
pub mod multibody_util;
pub mod physics;
pub mod robot;
pub mod simulator_thread;
pub mod spawn_utilities;
pub mod sync_strategies;

extern crate nalgebra as na;
