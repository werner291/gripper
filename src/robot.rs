use std::cell::RefCell;
use std::clone::Clone;
use std::f32::consts::{FRAC_PI_2, PI};
use std::iter::Iterator;
use std::option::Option::{None, Some};
use std::rc::Rc;
use std::string::{String, ToString};

use kiss3d::ncollide3d::na::{Isometry3, Rotation3, Unit, Vector3};
use kiss3d::ncollide3d::procedural::TriMesh;
use kiss3d::ncollide3d::shape::{ConvexHull, ShapeHandle};
use kiss3d::resource::{MaterialManager, Mesh, TextureManager};
use kiss3d::scene::{Object, SceneNode};
use nphysics3d::joint::{FixedJoint, RevoluteJoint};
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodyPartHandle, DefaultColliderHandle,
    MultibodyDesc,
};

use crate::graphics::Graphics;
use crate::physics::PhysicsWorld;
use crate::{load_mesh, multibody_util};
use std::prelude::v1::Vec;

#[derive(Debug, Clone)]
pub struct ArmJointMap<T> {
    pub swivel: T,
    pub link1: T,
    pub link2: T,
    pub gripper: T,
}

#[derive(Debug, Clone)]
pub struct FingerJointMap<T> {
    pub finger_0: T,
    pub finger_1: T,
    pub finger_2: T,
    pub finger_0_2: T,
    pub finger_1_2: T,
    pub finger_2_2: T,
}

#[derive(Debug, Clone)]
pub struct JointMap<T> {
    pub swivel: T,
    pub link1: T,
    pub link2: T,
    pub gripper: T,
    pub finger_0: T,
    pub finger_1: T,
    pub finger_2: T,
    pub finger_0_2: T,
    pub finger_1_2: T,
    pub finger_2_2: T,
}

impl<T> JointMap<T> {
    pub(crate) fn from_arm_and_finger(arm: ArmJointMap<T>, finger: FingerJointMap<T>) -> JointMap<T> {
        JointMap {
            swivel: arm.swivel,
            link1: arm.link1,
            link2: arm.link2,
            gripper: arm.gripper,
            finger_0: finger.finger_0,
            finger_1: finger.finger_1,
            finger_2: finger.finger_2,
            finger_0_2: finger.finger_0_2,
            finger_1_2: finger.finger_1_2,
            finger_2_2: finger.finger_2_2,
        }
    }
}

pub type JointVelocities = JointMap<f32>;
pub type ArmJointVelocities = ArmJointMap<f32>;

impl ArmJointVelocities {
    pub fn limit_to_safe(self, lim: f32) -> ArmJointVelocities {
        let max = self
            .swivel
            .abs()
            .max(self.link1.abs())
            .max(self.link2.abs())
            .max(self.gripper.abs());
        if max > lim {
            Self {
                swivel: lim * self.swivel / max,
                link1: lim * self.link1 / max,
                link2: lim * self.link2 / max,
                gripper: lim * self.gripper / max,
            }
        } else {
            self
        }
    }
}

pub const FINGERS_OPEN : FingerJointMap<f32> = FingerJointMap {
    finger_0: 0.5,
    finger_1: 0.5,
    finger_2: 0.5,
    finger_0_2: 0.5,
    finger_1_2: 0.5,
    finger_2_2: 0.5
};

pub const FINGERS_CLOSE : FingerJointMap<f32> = FingerJointMap {
    finger_0: -0.5,
    finger_1: -0.5,
    finger_2: -0.5,
    finger_0_2: -0.5,
    finger_1_2: -0.5,
    finger_2_2: -0.5
};

pub const ZERO_JOINT_VELOCITIES: JointVelocities = JointVelocities {
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
};

/// A struct that contains the body handle and body part handle of the various parts of a robot.
/// Note that each body part also has a name, should that be more convenient.
#[derive(Debug)]
pub struct RobotBodyPartIndex {
    pub body: DefaultBodyHandle,
    pub base: DefaultBodyPartHandle,
    pub swivel: DefaultBodyPartHandle,
    pub link1: DefaultBodyPartHandle,
    pub link2: DefaultBodyPartHandle,

    pub gripper: DefaultBodyPartHandle,
    pub gripper_collider: DefaultColliderHandle,

    pub finger_0: DefaultBodyPartHandle,
    pub finger_0_collider: DefaultColliderHandle,
    pub finger_1: DefaultBodyPartHandle,
    pub finger_1_collider: DefaultColliderHandle,
    pub finger_2: DefaultBodyPartHandle,

    pub finger_2_collider: DefaultColliderHandle,
    pub finger_0_2: DefaultBodyPartHandle,
    pub finger_0_2_collider: DefaultColliderHandle,
    pub finger_1_2: DefaultBodyPartHandle,
    pub finger_1_2_collider: DefaultColliderHandle,
    pub finger_2_2: DefaultBodyPartHandle,
    pub finger_2_2_collider: DefaultColliderHandle,
}

impl RobotBodyPartIndex {
    /// Provides an array of body part handles that correspond to parts
    /// that have a motorized revolute joint.
    ///
    /// Also provides a canonical mapping for motors to numerical channels.
    pub fn motor_parts(&self) -> [DefaultBodyPartHandle; 10] {
        [
            self.swivel,
            self.link1,
            self.link2,
            self.gripper,
            self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_0_2,
        ]
    }

    /// Array of motors that control the robot's fingers.
    ///
    /// Setting a positive speed on these opens the gripper, a negative speed closes them.
    pub fn finger_parts(&self) -> [DefaultBodyPartHandle; 6] {
        [
            self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_2_2,
        ]
    }

    pub fn gripper_colliders(&self) -> [DefaultColliderHandle; 7] {
        [
            self.gripper_collider,
            self.finger_0_collider,
            self.finger_1_collider,
            self.finger_2_collider,
            self.finger_0_2_collider,
            self.finger_1_2_collider,
            self.finger_2_2_collider,
        ]
    }
}

/// Build a physical and visible robot.
pub fn make_robot(physics: &mut PhysicsWorld, graphics: &mut Graphics) -> RobotBodyPartIndex {
    // Generates a Multibody of the robot, without any visible parts.
    let robot = make_multibody(physics);

    // Otherwise, the robot sometimes fails to respond to control signals if it goes to sleep.
    physics
        .bodies
        .get_mut(robot.body)
        .unwrap()
        .set_deactivation_threshold(None);

    build_graphics(&robot, graphics);

    robot
}

fn build_graphics(robot: &RobotBodyPartIndex, graphics: &mut Graphics) {
    // Allocate some scene nodes and load the appropriate meshes.
    let base = graphics.window.add_trimesh(
        load_mesh::trimesh_from_stl_sr(load_mesh::BASE_STL),
        Vector3::new(1.0, 1.0, 1.0),
    );
    let swivel = graphics.window.add_trimesh(
        load_mesh::trimesh_from_stl_sr(load_mesh::SWIVEL_STL),
        Vector3::new(1.0, 1.0, 1.0),
    );
    let link1 = graphics.window.add_trimesh(
        load_mesh::trimesh_from_stl_sr(load_mesh::ARMLINK_STL),
        Vector3::new(1.0, 1.0, 1.0),
    );
    let link2 = graphics.window.add_trimesh(
        load_mesh::trimesh_from_stl_sr(load_mesh::ARMLINK_STL),
        Vector3::new(1.0, 1.0, 1.0),
    );
    let gripper = graphics.window.add_trimesh(
        load_mesh::trimesh_from_stl_sr(load_mesh::GRIPPER_STL),
        Vector3::new(1.0, 1.0, 1.0),
    );

    graphics.bp_to_sn.extend_from_slice(&[
        (base, robot.base),
        (swivel, robot.swivel),
        (link1, robot.link1),
        (link2, robot.link2),
        (gripper, robot.gripper),
    ]);

    // Fingers are special due to them being copies of each other and rotated.
    for (bph, rot_i) in &[
        (robot.finger_0, 0),
        (robot.finger_1, 1),
        (robot.finger_2, 2),
        (robot.finger_0_2, 0),
        (robot.finger_1_2, 1),
        (robot.finger_2_2, 2),
    ] {
        let mesh = load_mesh::trimesh_from_stl_sr(load_mesh::PHALANX_STL);
        let transform = Isometry3::rotation(Vector3::new(
            0.0,
            std::f32::consts::FRAC_PI_2 + std::f32::consts::PI * (*rot_i) as f32 / 1.5,
            0.0,
        ));

        // Use the `wrap_transformed_trimesh` function since they need to be rotated,
        // which cannot be represented in the Multibody structure.
        let node = wrap_transformed_trimesh(mesh, transform);
        graphics
            .window
            .scene_mut()
            .add_child(node.clone() /* Rc<RefCell<_>> construction. */);

        graphics.bp_to_sn.push((node, *bph));
    }
}

const SWIVEL_SHIFT: f32 = 0.25;
const LINK1_SHIFT: f32 = 1.25;
const LINK_LENGTH: f32 = 2.5;

pub const FINGER_BASE_JOINT_MIN_ANGLE: f32 = 0.0;
pub const FINGER_BASE_JOINT_MAX_ANGLE: f32 = FRAC_PI_2;

pub const FINGER_MIDDLE_JOINT_MIN_ANGLE: f32 = -1.0;
pub const FINGER_MIDDLE_JOINT_MAX_ANGLE: f32 = -0.1;

/// Generates a Multibody of the robot, without colliders.
fn make_multibody(physics: &mut PhysicsWorld) -> RobotBodyPartIndex {

    // FIXMe Can this be done better? This really, truly is an awful lot of code.

    let joint = FixedJoint::new(Isometry3::identity());

    let mut base = MultibodyDesc::new(joint).name("base".to_string()).mass(1.0);

    let mut swivel = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::y()), 0.0);
        joint.enable_angular_motor();
        joint.set_desired_angular_motor_velocity(1.0);
        joint.set_max_angular_motor_torque(1.0);
        base.add_child(joint)
            .set_name("swivel".to_string())
            .set_parent_shift(Vector3::new(0.0, SWIVEL_SHIFT, 0.0))
    };

    let mut link1 = make_link(
        &mut swivel,
        Vector3::new(0.0, LINK1_SHIFT, 0.0),
        -PI * 0.75,
        PI * 0.75,
        Vector3::x(),
        "link1".to_string(),
    );

    let mut link2 = make_link(
        &mut link1,
        Vector3::new(0.0, LINK_LENGTH, 0.0),
        -PI * 0.75,
        PI * 0.75,
        Vector3::x(),
        "link2".to_string(),
    );
    let mut gripper = make_link(
        &mut link2,
        Vector3::new(0.0, LINK_LENGTH, 0.0),
        -FRAC_PI_2,
        FRAC_PI_2,
        Vector3::x(),
        "gripper".to_string(),
    );

    for i in 0..3 {
        let rot = Rotation3::new(Vector3::new(0.0, 2.0 * PI * (i as f32) / 3.0, 0.0));
        let mut phalanx_1 = make_link(
            &mut gripper,
            &rot * Vector3::new(0.0, 0.5, 0.3),
            FINGER_BASE_JOINT_MIN_ANGLE,
            FINGER_BASE_JOINT_MAX_ANGLE,
            &rot * Vector3::x(),
            format!("finger_{}", i),
        );
        make_link(
            &mut phalanx_1,
            &rot * Vector3::new(0.0, 0.5, 0.0),
            FINGER_MIDDLE_JOINT_MIN_ANGLE,
            FINGER_MIDDLE_JOINT_MAX_ANGLE,
            &rot * Vector3::x(),
            format!("finger_{}_2", i),
        );
    }

    let mb = physics.bodies.insert(base.build());

    let body = physics.bodies.multibody_mut(mb).unwrap();

    let base = BodyPartHandle(mb, body.links_with_name("base").next().unwrap().0);
    let swivel = BodyPartHandle(mb, body.links_with_name("swivel").next().unwrap().0);
    let link1 = BodyPartHandle(mb, body.links_with_name("link1").next().unwrap().0);
    let link2 = BodyPartHandle(mb, body.links_with_name("link2").next().unwrap().0);
    let gripper = BodyPartHandle(mb, body.links_with_name("gripper").next().unwrap().0);
    let finger_0 = BodyPartHandle(mb, body.links_with_name("finger_0").next().unwrap().0);
    let finger_1 = BodyPartHandle(mb, body.links_with_name("finger_1").next().unwrap().0);
    let finger_2 = BodyPartHandle(mb, body.links_with_name("finger_2").next().unwrap().0);
    let finger_0_2 = BodyPartHandle(mb, body.links_with_name("finger_0_2").next().unwrap().0);
    let finger_1_2 = BodyPartHandle(mb, body.links_with_name("finger_1_2").next().unwrap().0);
    let finger_2_2 = BodyPartHandle(mb, body.links_with_name("finger_2_2").next().unwrap().0);

    let gripper_mesh = load_mesh::trimesh_from_stl_sr(load_mesh::GRIPPER_STL);

    let gripper_collider =
        attach_collider_with_mesh(physics, gripper, gripper_mesh, Vector3::new(0.0, 0.0, 0.0));

    let finger_colliders = [
        finger_0, finger_1, finger_2, finger_0_2, finger_1_2, finger_2_2,
    ]
    .iter()
    .enumerate()
    .map(|(i, n)| {
        let mesh = load_mesh::trimesh_from_stl_sr(load_mesh::PHALANX_STL);

        attach_collider_with_mesh(
            physics,
            *n,
            mesh,
            Vector3::new(0.0, FRAC_PI_2 + PI * i as f32 / 1.5, 0.0),
        )
    })
    .collect::<Vec<_>>();

    RobotBodyPartIndex {
        body: mb,
        base,
        swivel,
        link1,
        link2,
        gripper,
        gripper_collider,
        finger_0,
        finger_0_collider: finger_colliders[0],
        finger_1,
        finger_1_collider: finger_colliders[1],
        finger_2,
        finger_2_collider: finger_colliders[2],
        finger_0_2,
        finger_0_2_collider: finger_colliders[3],
        finger_1_2,
        finger_1_2_collider: finger_colliders[4],
        finger_2_2,
        finger_2_2_collider: finger_colliders[5],
    }
}

fn attach_collider_with_mesh(
    physics: &mut PhysicsWorld,
    bph: DefaultBodyPartHandle,
    mesh: TriMesh<f32>,
    rotation_angleaxis: Vector3<f32>,
) -> DefaultColliderHandle {
    let shape =
        ConvexHull::try_from_points(mesh.coords.as_slice()).expect("Cannot create convex hull.");

    let collider = ColliderDesc::new(ShapeHandle::new(shape))
        .density(1.0)
        .rotation(rotation_angleaxis)
        .build(bph);

    physics.colliders.insert(collider)
}

fn make_link<'a>(
    swivel: &'a mut &mut MultibodyDesc<f32>,
    parent_shift: Vector3<f32>,
    min_angle: f32,
    max_angle: f32,
    rev_axis: Vector3<f32>,
    name: String,
) -> &'a mut MultibodyDesc<f32> {
    let mut joint = RevoluteJoint::new(Unit::new_normalize(rev_axis), 0.0);
    joint.enable_angular_motor();
    joint.set_max_angular_motor_torque(50.0);
    joint.enable_min_angle(min_angle);
    joint.enable_max_angle(max_angle);
    swivel
        .add_child(joint)
        .set_name(name)
        .set_parent_shift(parent_shift)
}

fn wrap_transformed_trimesh(trimesh: TriMesh<f32>, transform: Isometry3<f32>) -> SceneNode {
    let mesh = Rc::new(RefCell::new(Mesh::from_trimesh(trimesh, false)));
    let tex = TextureManager::get_global_manager(|tm| tm.get_default());
    let mat = MaterialManager::get_global_manager(|mm| mm.get_default());
    let mesh_obj = Object::new(mesh, 1.0, 1.0, 1.0, tex, mat);
    let local_sn = SceneNode::new(Vector3::new(1.0, 1.0, 1.0), transform, Some(mesh_obj));
    let mut sn = SceneNode::new(Vector3::new(1.0, 1.0, 1.0), Isometry3::identity(), None);
    sn.add_child(local_sn);
    sn
}

#[derive(Clone, Copy)]
pub enum GripperDirection {
    Open,
    Closed,
}

pub fn set_gripper_direction(
    physics: &mut PhysicsWorld,
    bdi: &RobotBodyPartIndex,
    dir: GripperDirection,
) {
    for bp in bdi.finger_parts().iter() {
        multibody_util::set_motor_speed(
            physics,
            *bp,
            match dir {
                GripperDirection::Open => 1.0,
                GripperDirection::Closed => -1.0,
            },
        );
    }
}

/// Extract the joint angles of every finger part of the robot.
pub fn gripper_finger_angles(
    physics: &PhysicsWorld,
    robot: &RobotBodyPartIndex,
) -> FingerJointMap<f32> {
    FingerJointMap {
        finger_0: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_0)
            .unwrap()
            .angle(),
        finger_1: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_1)
            .unwrap()
            .angle(),
        finger_2: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_2)
            .unwrap()
            .angle(),
        finger_0_2: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_0_2)
            .unwrap()
            .angle(),
        finger_1_2: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_1_2)
            .unwrap()
            .angle(),
        finger_2_2: multibody_util::get_joint::<RevoluteJoint<f32>>(physics, robot.finger_2_2)
            .unwrap()
            .angle(),
    }
}
