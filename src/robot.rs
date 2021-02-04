use std::cell::RefCell;
use std::clone::Clone;
use std::f32::consts::{FRAC_PI_2, PI};
use std::iter::Iterator;
use std::option::Option;
use std::option::Option::{None, Some};

use std::rc::Rc;
use std::string::{String, ToString};

use kiss3d::ncollide3d::na::{Isometry3, Rotation3, Unit, Vector3};

use kiss3d::ncollide3d::procedural::TriMesh;
use kiss3d::ncollide3d::shape::{ConvexHull, ShapeHandle};
use kiss3d::resource::{MaterialManager, Mesh, TextureManager};
use kiss3d::scene::{Object, SceneNode};
use nphysics3d::joint::{FixedJoint, Joint, RevoluteJoint};
use nphysics3d::object::{BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodyPartHandle, MultibodyDesc, MultibodyLink, BodyPart};

use crate::graphics::Graphics;
use crate::load_mesh;
use crate::physics::PhysicsWorld;

pub(crate) const NUM_CHANNELS: usize = 10;

// TODO This is way too risky, need to improve.
// pub const CHANNEL_SWIVEL: usize = 0;
// pub const CHANNEL_LINK1: usize = 1;
// pub const CHANNEL_LINK2: usize = 2;
// pub const CHANNEL_GRIPPER: usize = 3;
// pub const CHANNEL_FINGER_0: usize = 4;
// pub const CHANNEL_FINGER_1: usize = 5;
// pub const CHANNEL_FINGER_2: usize = 6;
// pub const CHANNEL_FINGER_0_2: usize = 7;
// pub const CHANNEL_FINGER_1_2: usize = 8;
// pub const CHANNEL_FINGER_2_2: usize = 9;

#[derive(Debug, Clone)]
pub struct ArmJointMap<T> {
    pub swivel: T,
    pub link1: T,
    pub link2: T,
    pub gripper: T
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

pub type JointVelocities = JointMap<f32>;
pub type ArmJointVelocities = ArmJointMap<f32>;

impl ArmJointVelocities {
    pub fn limit_to_safe(self, lim:f32) -> ArmJointVelocities {
        let max = self.swivel.abs().max(self.link1.abs()).max(self.link2.abs()).max(self.gripper.abs());
        if max > lim {
            Self {
                swivel: self.swivel / max,
                link1: self.link1 / max,
                link2: self.link2 / max,
                gripper: self.gripper / max
            }
        } else {
            self
        }
    }
}

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
    finger_2_2: 0.0
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
    pub finger_0: DefaultBodyPartHandle,
    pub finger_1: DefaultBodyPartHandle,
    pub finger_2: DefaultBodyPartHandle,
    pub finger_0_2: DefaultBodyPartHandle,
    pub finger_1_2: DefaultBodyPartHandle,
    pub finger_2_2: DefaultBodyPartHandle,
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
}

/// Build a physical and visible robot.
pub fn make_robot(physics: &mut PhysicsWorld, graphics: &mut Graphics) -> RobotBodyPartIndex {
    // Generates a Multibody of the robot, without any visible parts.
    let robot = make_multibody(physics);

    attach_colliders(physics, &robot);

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
        (base, robot.base.clone()),
        (swivel, robot.swivel.clone()),
        (link1, robot.link1.clone()),
        (link2, robot.link2.clone()),
        (gripper, robot.gripper.clone()),
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

fn attach_colliders(physics: &mut PhysicsWorld, robot: &RobotBodyPartIndex) {
    let gripper = load_mesh::trimesh_from_stl_sr(load_mesh::GRIPPER_STL);

    attach_collider_with_mesh(physics, robot.gripper, gripper, Vector3::new(0.0, 0.0, 0.0));

    for (i, n) in robot.finger_parts().iter().enumerate() {
        let mesh = load_mesh::trimesh_from_stl_sr(load_mesh::PHALANX_STL);

        attach_collider_with_mesh(
            physics,
            *n,
            mesh,
            Vector3::new(0.0, FRAC_PI_2 + PI * i as f32 / 1.5, 0.0),
        );
    }
}

const SWIVEL_SHIFT: f32 = 0.25;
const LINK1_SHIFT: f32 = 1.25;
const LINK_LENGTH: f32 = 2.5;

/// Generates a Multibody of the robot, without colliders.
fn make_multibody(physics: &mut PhysicsWorld) -> RobotBodyPartIndex {
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
            0.0,
            FRAC_PI_2,
            &rot * Vector3::x(),
            format!("finger_{}", i),
        );
        make_link(
            &mut phalanx_1,
            &rot * Vector3::new(0.0, 0.5, 0.0),
            -1.0,
            0.0,
            &rot * Vector3::x(),
            format!("finger_{}_2", i),
        );
    }

    let mb = physics.bodies.insert(base.build());

    let robot = physics.bodies.multibody_mut(mb).unwrap();

    RobotBodyPartIndex {
        body: mb,
        base: BodyPartHandle(mb, robot.links_with_name("base").next().unwrap().0),
        swivel: BodyPartHandle(mb, robot.links_with_name("swivel").next().unwrap().0),
        link1: BodyPartHandle(mb, robot.links_with_name("link1").next().unwrap().0),
        link2: BodyPartHandle(mb, robot.links_with_name("link2").next().unwrap().0),
        gripper: BodyPartHandle(mb, robot.links_with_name("gripper").next().unwrap().0),
        finger_0: BodyPartHandle(mb, robot.links_with_name("finger_0").next().unwrap().0),
        finger_1: BodyPartHandle(mb, robot.links_with_name("finger_1").next().unwrap().0),
        finger_2: BodyPartHandle(mb, robot.links_with_name("finger_2").next().unwrap().0),
        finger_0_2: BodyPartHandle(mb, robot.links_with_name("finger_0_2").next().unwrap().0),
        finger_1_2: BodyPartHandle(mb, robot.links_with_name("finger_1_2").next().unwrap().0),
        finger_2_2: BodyPartHandle(mb, robot.links_with_name("finger_2_2").next().unwrap().0),
    }
}

fn attach_collider_with_mesh(
    physics: &mut PhysicsWorld,
    bph: DefaultBodyPartHandle,
    mesh: TriMesh<f32>,
    rotation_angleaxis: Vector3<f32>,
) {
    let shape =
        ConvexHull::try_from_points(mesh.coords.as_slice()).expect("Cannot create convex hull.");

    let collider = ColliderDesc::new(ShapeHandle::new(shape))
        .density(1.0)
        .rotation(rotation_angleaxis)
        .build(bph);

    physics.colliders.insert(collider);
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

pub fn get_joint<J: Joint<f32>>(
    physics: &PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&J> {
    get_multibody_link(physics, part_handle)?
        .joint()
        .downcast_ref()
}

pub fn get_joint_mut<J: Joint<f32>>(
    physics: &mut PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&mut J> {
    get_multibody_link_mut(physics, part_handle)?
        .joint_mut()
        .downcast_mut()
}

pub fn get_multibody_link(
    physics: &PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&MultibodyLink<f32>> {
    physics.bodies.multibody(part_handle.0)?.link(part_handle.1)
}

pub fn get_multibody_link_mut(
    physics: &mut PhysicsWorld,
    part_handle: DefaultBodyPartHandle,
) -> Option<&mut MultibodyLink<f32>> {
    physics
        .bodies
        .multibody_mut(part_handle.0)?
        .link_mut(part_handle.1)
}

pub fn set_motor_speed(physics: &mut PhysicsWorld, part_handle: DefaultBodyPartHandle, speed: f32) {
    get_joint_mut::<RevoluteJoint<f32>>(physics, part_handle)
        .unwrap()
        .set_desired_angular_motor_velocity(speed);
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
        set_motor_speed(
            physics,
            *bp,
            match dir {
                GripperDirection::Open => 1.0,
                GripperDirection::Closed => -1.0,
            },
        );
    }
}

pub fn multibody_link_position(physics: &PhysicsWorld, bph: DefaultBodyPartHandle) -> Option<Isometry3<f32>> {
    get_multibody_link(physics, bph).map(MultibodyLink::position)
}
