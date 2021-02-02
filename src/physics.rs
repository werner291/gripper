use std::collections::HashMap;
use std::marker::Send;
use std::option::Option::Some;
use std::prelude::v1::{FnMut, Iterator};
use std::sync::mpsc::{channel, Receiver};
use std::thread;
use std::thread::JoinHandle;

use kiss3d::ncollide3d::na::Isometry3;
use nalgebra::Vector3;
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::nalgebra::RealField;
use nphysics3d::ncollide3d::broad_phase::BroadPhasePairFilter;
use nphysics3d::object::{
    Body, BodyPartHandle, BodySet, ColliderAnchor, ColliderSet, DefaultBodyHandle,
    DefaultBodyPartHandle, DefaultBodySet, DefaultColliderSet,
};
use nphysics3d::world::{
    BroadPhasePairFilterSets, DefaultGeometricalWorld, DefaultMechanicalWorld,
};

use crate::robot::RobotBodyPartIndex;
use crate::sync_strategies::WaitStrategy;
use std::boxed::Box;

/// A contact filter that ensures that the robot cannot collide with itself,
/// preventing glitchy physics from jointed parts.
///
/// FIXME: Would it be better to only prevent parts directly connected from contacting each other?
///
/// Stolen from https://github.com/dimforge/nphysics/blob/master/examples3d/broad_phase_filter3.rs
struct NoMultibodySelfContactFilter;

impl<N, Bodies, Colliders>
    BroadPhasePairFilter<N, BroadPhasePairFilterSets<'_, N, Bodies, Colliders>>
    for NoMultibodySelfContactFilter
where
    N: RealField,
    Bodies: BodySet<N>,
    Colliders: ColliderSet<N, Bodies::Handle>,
{
    fn is_pair_valid(
        &self,
        h1: Colliders::Handle,
        h2: Colliders::Handle,
        set: &BroadPhasePairFilterSets<'_, N, Bodies, Colliders>,
    ) -> bool {
        let a1 = set.colliders().get(h1).map(|c| c.anchor());
        let a2 = set.colliders().get(h2).map(|c| c.anchor());

        match (a1, a2) {
            (
                Some(ColliderAnchor::OnBodyPart {
                    body_part: part1, ..
                }),
                Some(ColliderAnchor::OnBodyPart {
                    body_part: part2, ..
                }),
            ) => part1.0 != part2.0, // Don't collide if the two parts belong to the same body.
            _ => true,
        }
    }
}

/// Small container struct for nphysics-related stuff
/// with parameters that make sense for the robot simulation.
///
/// Specifically, everything default, except for:
///  - Gravity of -9.81 towards negative y-axis
///  - Multibody parts in the same body cannot collide with each other.
///
/// To use, just call step() every frame.
///
pub struct PhysicsWorld {
    pub mechanical_world: DefaultMechanicalWorld<f32>,
    pub geometrical_world: DefaultGeometricalWorld<f32>,
    pub bodies: DefaultBodySet<f32>,
    pub colliders: DefaultColliderSet<f32>,
    pub joint_constraints: DefaultJointConstraintSet<f32>,
    pub force_generators: DefaultForceGeneratorSet<f32>,
}

impl PhysicsWorld {
    /// Initialize an empty PhysicsWorld with some default parameters.
    pub fn new() -> Self {
        PhysicsWorld {
            mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, -9.81, 0.0)),
            geometrical_world: DefaultGeometricalWorld::new(),
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),
        }
    }

    /// Step the PhysicsWorld forward by a small time step with default parameters,
    /// except with a NoMultibodySelfContactFilter to make sure the robot doesn't
    /// collide with itself.
    pub fn step(&mut self) {
        self.mechanical_world.step_with_filter(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joint_constraints,
            &mut self.force_generators,
            &NoMultibodySelfContactFilter,
        );
    }
}

/// A callback that implements logic to control the robot's motors.
/// TODO Too much mutability for my liking, better make a function returning motor speeds.
pub trait ControllerStrategy: Send + 'static {
    fn apply_controller(&mut self, physics_world: &mut PhysicsWorld, robot: &RobotBodyPartIndex);
}

impl<F> ControllerStrategy for F
where
    F: FnMut(&mut PhysicsWorld, &RobotBodyPartIndex) + Send + 'static,
{
    fn apply_controller(&mut self, pw: &mut PhysicsWorld, rob: &RobotBodyPartIndex) {
        self(pw, rob)
    }
}

/// Message sent from physics thread about the state of the world.
pub type PhysicsUpdate = HashMap<DefaultBodyPartHandle, Isometry3<f32>>;

/// Run the "simulation" part of the simulator app, independently of the graphics thread.
/// Returns the JoinHandle of the thread, as well a Receiver, which provides the position
/// of every body part in the simulation at every frame.
///
/// That way, the simulation can run at whatever pace makes sense, depending on the scenario.
///
/// For instance, the provided WaitStrategy can simply hold the simulation until the graphics
/// thread has completed drawing a frame, keeping both roughly in sync, but allowing the simulation
/// to take longer if it needs to.
///
/// The provided controller can also take however much time it needs, making sure that it is run
/// exactly once every time step of the simulation.
///
/// # Arguments
///
/// * `robot` - A RobotBodyPartIndex corresponding to a robot somewhere in the simulation.
/// * `controller` - A callback meant to hold the logic that controls the target motor speeds of the robot.
/// * `wait_strategy` - A method that should hold the calling thread until the next frame of the simulation should be computed.
/// * `physics` - The PhysicsWorld, initialized with whatever needs to be present in the simulation.
///
/// TODO: Maybe pass the Sender in as a parameter instead?
pub fn start_physics_thread<W>(
    robot: RobotBodyPartIndex,
    mut controller: Box<dyn ControllerStrategy>, // I hate that this is stateful...
    mut wait_strategy: W,
    mut physics: PhysicsWorld,
) -> (JoinHandle<()>, Receiver<PhysicsUpdate>)
where
    W: WaitStrategy,
{
    // Create achannel for updates about positions.
    let (snd, rcv) = channel();

    // Spawn the simulation main loop thread and move necessary valies into it.
    let join = thread::spawn(move || {
        loop {
            // Apply the waiting strategy, e.g. to synchronize with the graphics thread without blocking it.
            wait_strategy();

            // Apply a timestep in the physics engine.
            physics.step();

            // Apply the robot control callback.
            controller.apply_controller(&mut physics, &robot);

            // Update any interested parties in the positions of the various body parts.
            snd.send(snapshot_physics(&physics)).unwrap()
        }
    });

    (join, rcv)
}

/// Take a snapshot of the Physics engine that can be safely sent to the graphics thread.
fn snapshot_physics(physics: &PhysicsWorld) -> PhysicsUpdate {
    // For every body,
    physics
        .bodies
        .iter()
        .flat_map(|(bh, body): (DefaultBodyHandle, &dyn Body<f32>)| {
            // for every body part,
            (0..body.num_parts()).map(move |i| {
                // register the BodyPartHandle and world position,
                let bph = BodyPartHandle(bh, i);
                let pos = body.part(i).unwrap().position().clone();
                (bph, pos)
            })
        })
        .collect() // then put them all in a HashMap for easy lookup.
}
