use nalgebra::Vector3;
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::nalgebra::RealField;
use nphysics3d::ncollide3d::broad_phase::BroadPhasePairFilter;
use nphysics3d::object::{
    BodySet, ColliderAnchor, ColliderSet, DefaultBodySet, DefaultColliderSet,
};
use nphysics3d::world::{
    BroadPhasePairFilterSets, DefaultGeometricalWorld, DefaultMechanicalWorld,
};
use std::option::Option::Some;

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
