use std::cell::RefCell;
use std::clone::Clone;
use std::convert::{From, Into};
use std::f32::consts::{FRAC_PI_2, PI};
use std::iter::Iterator;
use std::option::Option::{None, Some};
use std::option::Option;
use std::prelude::v1::Vec;
use std::rc::Rc;
use std::string::{String, ToString};

use kiss3d::ncollide3d::na::{Isometry3, Rotation3, Unit, Vector3};
use kiss3d::ncollide3d::pipeline::CollisionGroups;
use kiss3d::ncollide3d::procedural::TriMesh;
use kiss3d::ncollide3d::shape::{ConvexHull, ShapeHandle};
use kiss3d::ncollide3d::shape::TriMesh as sTriMesh;
use kiss3d::resource::{MaterialManager, Mesh, TextureManager};
use kiss3d::scene::{Object, SceneNode};
use nphysics3d::joint::{FixedJoint, Joint, RevoluteJoint};
use nphysics3d::object::{Body, BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodyPartHandle, DefaultColliderHandle, Multibody, MultibodyDesc, MultibodyLink};

use crate::load_mesh;
use crate::physics::PhysicsWorld;

/// A struct that contains the body handle and body part handle of the various parts of a robot.
/// Note that each body part also has a name, should that be more convenient.
#[derive(Debug)]
pub struct RobotBodypartIndex {
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

impl RobotBodypartIndex {
    /// Provides an array of body part handles that correspond to parts
    /// that have a motorized revolute joint.
    ///
    /// Also provides a canonical mapping for motors to numerical channels.
    pub fn motor_parts(&self) -> [DefaultBodyPartHandle; 10] {
        [   self.swivel,
            self.link1,
            self.link2,
            self.gripper,
            self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_0_2
        ]
    }

    /// Array of motors that control the robot's fingers.
    ///
    /// Setting a positive speed on these opens the gripper, a negative speed closes them.
    pub fn finger_parts(&self) -> [DefaultBodyPartHandle; 6] {
        [   self.finger_0,
            self.finger_1,
            self.finger_2,
            self.finger_0_2,
            self.finger_1_2,
            self.finger_0_2
        ]
    }
}

/// Build a physical and visible robot.
pub fn make_robot(physics: &mut PhysicsWorld, scene: &mut SceneNode, part_to_body: &mut Vec<(SceneNode, DefaultBodyPartHandle)>) -> RobotBodypartIndex {

    // Generates a Multibody of the robot, without any visible parts.
    let robot = make_multibody(physics);

    // Otherwise, the robot sometimes fails to respond to control signals if it goes to sleep.
    physics.bodies.get_mut(robot.body).unwrap().set_deactivation_threshold(None);

    // Allocate some scene nodes and load the appropriate meshes.
    let base = scene.add_trimesh(load_mesh::stl_to_trimesh("scad/base.stl"), Vector3::new(1.0, 1.0, 1.0));
    let swivel = scene.add_trimesh(load_mesh::stl_to_trimesh("scad/rotbase.stl"), Vector3::new(1.0, 1.0, 1.0));
    let link1 = scene.add_trimesh(load_mesh::stl_to_trimesh("scad/armlink.stl"), Vector3::new(1.0, 1.0, 1.0));
    let link2 = scene.add_trimesh(load_mesh::stl_to_trimesh("scad/armlink.stl"), Vector3::new(1.0, 1.0, 1.0));
    let gripper = scene.add_trimesh(load_mesh::stl_to_trimesh("scad/gripper.stl"), Vector3::new(1.0, 1.0, 1.0));

    // Fingers are special due to them being copies of each other and rotated.
    let [finger_0, finger_1, finger_2, finger_0_2, finger_1_2, finger_2_2] = [0, 1, 2, 0, 1, 2].map(|i| {
        let mesh = load_mesh::stl_to_trimesh("scad/phalanx.stl");
        let transform = Isometry3::rotation(Vector3::new(0.0, std::f32::consts::FRAC_PI_2 + std::f32::consts::PI * i as f32 / 1.5, 0.0));

        // Use the `wrap_transformed_trimesh` function since they need to be rotated,
        // which cannot be represented in the Multibody structure.
        let node = wrap_transformed_trimesh(mesh, transform);
        scene.add_child(node.clone() /* Rc<RefCell<_>> construction. */);
        node
    });

    // Update the part-to-body index, which will allow the simulator
    // to synchronize the body parts to their scene node counterparts.
    part_to_body.extend_from_slice(
        &[ (base, robot.base.clone()),
            (swivel, robot.swivel.clone()),
            (link1, robot.link1.clone()),
            (link2, robot.link2.clone()),
            (gripper, robot.gripper.clone()),
            (finger_0, robot.finger_0.clone()),
            (finger_1, robot.finger_1.clone()),
            (finger_2, robot.finger_2.clone()),
            (finger_0_2, robot.finger_0_2.clone()),
            (finger_1_2, robot.finger_1_2.clone()),
            (finger_2_2, robot.finger_2_2.clone())]
    );

    robot
}

/// Generates a Multibody of the robot, with colliders.
fn make_multibody(physics: &mut PhysicsWorld) -> RobotBodypartIndex {
    let mut joint = FixedJoint::new(Isometry3::identity());

    let mut base = MultibodyDesc::new(joint).name("base".to_string()).mass(1.0);

    let mut swivel = {
        let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::y()), 0.0);
        joint.enable_angular_motor();
        joint.set_desired_angular_motor_velocity(1.0);
        joint.set_max_angular_motor_torque(1.0);
        base.add_child(joint).set_name("swivel".to_string()).set_parent_shift(Vector3::new(0.0, 0.25, 0.0))
    };

    let mut link1 = make_link(&mut swivel, Vector3::new(0.0, 1.25, 0.0), -FRAC_PI_2, FRAC_PI_2, Vector3::x(), "link1".to_string());
    let mut link2 = make_link(&mut link1, Vector3::new(0.0, 2.5, 0.0), -FRAC_PI_2, FRAC_PI_2, Vector3::x(), "link2".to_string());
    let mut gripper = make_link(&mut link2, Vector3::new(0.0, 2.5, 0.0), -FRAC_PI_2, FRAC_PI_2, Vector3::x(), "gripper".to_string());

    for i in 0..3 {
        let rot = Rotation3::new(Vector3::new(0.0, 2.0 * PI * (i as f32) / 3.0, 0.0));
        let mut phalanx_1 = make_link(&mut gripper, &rot * Vector3::new(0.0, 0.5, 0.3), 0.0, FRAC_PI_2, &rot * Vector3::x(), format!("finger_{}", i), );
        make_link(&mut phalanx_1, &rot * Vector3::new(0.0, 0.5, 0.0), -0.4, FRAC_PI_2, &rot * Vector3::x(), format!("finger_{}_2", i), );
    }

    let mb = physics.bodies.insert(base.build());

    let gripper = load_mesh::stl_to_trimesh("scad/gripper.stl");

    attach_collider_with_mesh(physics, mb, "gripper", gripper, Vector3::new(0.0, 0.0, 0.0));

    for (i, n) in ["finger_0", "finger_1", "finger_2", "finger_0_2", "finger_1_2", "finger_2_2"].iter().enumerate() {
        let mesh = load_mesh::stl_to_trimesh("scad/phalanx.stl");

        attach_collider_with_mesh(physics, mb, n, mesh,
                                  Vector3::new(0.0, FRAC_PI_2 + PI * i as f32 / 1.5, 0.0));
    }

    let mut robot = physics.bodies.multibody_mut(mb).unwrap();

    RobotBodypartIndex {
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


fn attach_collider_with_mesh(physics: &mut PhysicsWorld, mb: DefaultBodyHandle, link_name: &str, mesh: TriMesh<f32>, rotation_angleaxis: Vector3<f32>) {
    let robot = physics.bodies.multibody(mb).unwrap();
    let link_id = robot.links_with_name(link_name).next().unwrap().0;
    let bph = BodyPartHandle(mb, link_id);

    let shape = ConvexHull::try_from_points(mesh.coords.as_slice()).expect("Cannot create convex hull.");

    let collider = ColliderDesc::new(ShapeHandle::new(shape))
        .density(1.0)
        .rotation(rotation_angleaxis)
        .build(bph);

    physics.colliders.insert(collider);
}


fn make_link<'a>(swivel: &'a mut &mut MultibodyDesc<f32>, parent_shift: Vector3<f32>, min_angle: f32, max_angle: f32, rev_axis: Vector3<f32>, name: String) -> &'a mut MultibodyDesc<f32> {
    let mut joint = RevoluteJoint::new(Unit::new_normalize(rev_axis), 0.0);
    joint.enable_angular_motor();
    joint.set_max_angular_motor_torque(10.0);
    joint.enable_min_angle(min_angle);
    joint.enable_max_angle(max_angle);
    swivel.add_child(joint).set_name(name).set_parent_shift(parent_shift)
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

pub fn get_joint_mut<J: Joint<f32>>(physics: &mut PhysicsWorld, part_handle: DefaultBodyPartHandle) -> Option<&mut J> {
    let mut mb: &mut Multibody<_> = physics.bodies.multibody_mut(part_handle.0)?;
    let mut link: &mut MultibodyLink<_> = mb.link_mut(part_handle.1)?;
    link.joint_mut().downcast_mut()
}
