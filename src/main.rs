#![feature(array_map)]

use std::convert::{From, Into};
use std::fs::OpenOptions;
use std::iter::{IntoIterator, Iterator};
use std::option::Option::{None, Some};
use std::prelude::v1::Vec;
use std::string::ToString;

use kiss3d::light::Light;
use kiss3d::ncollide3d::procedural::{IndexBuffer, TriMesh};
use kiss3d::scene::{SceneNode, Object};
use kiss3d::window::Window;
use na::{Isometry3, Point3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3};
use nphysics3d::joint::{FixedJoint, RevoluteJoint, Joint};
use nphysics3d::object::{BodyPartHandle, DefaultBodyHandle, DefaultBodyPartHandle, MultibodyDesc, Multibody, MultibodyLink};
use stl_io::{IndexedTriangle, Triangle, Vertex};

use crate::physics::PhysicsWorld;
use kiss3d::event::{Key, Action};
use crate::robot::RobotBodypartIndex;
use kiss3d::nalgebra::RealField;
use std::result::Result;
use std::option::Option;
use std::clone::Clone;
use std::rc::Rc;
use std::cell::RefCell;
use kiss3d::resource::{TextureManager, MaterialManager, Mesh};

mod physics;
mod robot;

extern crate kiss3d;
extern crate nalgebra as na;

fn main() {

    let mut window = Window::new("Kiss3d: cube");
    window.set_framerate_limit(Some(60));
    let mut physics = physics::PhysicsWorld::new();

    let robot = robot::make_robot(&mut physics);

    let scene = window.scene_mut();
    let base = scene.add_trimesh(stl_to_trimesh("scad/base.stl"), Vector3::new(1.0, 1.0, 1.0));
    let swivel = scene.add_trimesh(stl_to_trimesh("scad/rotbase.stl"), Vector3::new(1.0, 1.0, 1.0));
    let link1 = scene.add_trimesh(stl_to_trimesh("scad/armlink.stl"), Vector3::new(1.0, 1.0, 1.0));
    let link2 = scene.add_trimesh(stl_to_trimesh("scad/armlink.stl"), Vector3::new(1.0, 1.0, 1.0));
    let gripper = scene.add_trimesh(stl_to_trimesh("scad/gripper.stl"), Vector3::new(1.0, 1.0, 1.0));

    let [finger_0, finger_1, finger_2,finger_0_2, finger_1_2, finger_2_2] = [0,1,2,0,1,2].map(|i| {
        let mesh = stl_to_trimesh("scad/phalanx.stl");
        let transform = Isometry3::rotation(Vector3::new(0.0, std::f32::consts::FRAC_PI_2 + std::f32::consts::PI * i as f32 / 1.5, 0.0));
        add_transformed_trimesh(scene, mesh, transform)
    });

    let mut bp_to_sn = [
        (base, robot.base.clone()),
        (swivel, robot.swivel.clone()),
        (link1, robot.link1.clone()),
        (link2, robot.link2.clone()),
        (gripper, robot.gripper.clone()),
        (finger_0, robot.finger_0.clone()),
        (finger_1, robot.finger_1.clone()),
        (finger_2, robot.finger_2.clone()),
        (finger_0_2, robot.finger_0_2.clone()),
        (finger_1_2, robot.finger_1_2.clone()),
        (finger_2_2, robot.finger_2_2.clone())
    ];

    window.set_light(Light::StickToCamera);

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);

    while window.render() {
        // c.prepend_to_local_rotation(&rot);
        physics.step();

        for (sn,BodyPartHandle(bh, ph)) in bp_to_sn.iter_mut() {
            let pos : Isometry3<f32> = physics.bodies.get(*bh).unwrap().part(*ph).unwrap().position();

            sn.set_local_transformation(
                pos
            );
        }

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
}

fn add_transformed_trimesh(scene: &mut SceneNode, trimesh: TriMesh<f32>, transform: Isometry3<f32>) -> SceneNode {
    let mesh = Rc::new(RefCell::new(Mesh::from_trimesh(trimesh, false)));
    let tex = TextureManager::get_global_manager(|tm| tm.get_default());
    let mat = MaterialManager::get_global_manager(|mm| mm.get_default());
    let mesh_obj = Object::new(mesh, 1.0, 1.0, 1.0, tex, mat);
    let local_sn = SceneNode::new(Vector3::new(1.0, 1.0, 1.0), transform, Some(mesh_obj));
    let mut sn = SceneNode::new(Vector3::new(1.0, 1.0, 1.0), Isometry3::identity(), None);
    sn.add_child(local_sn);
    scene.add_child(sn.clone());
    sn
}

fn revjoint_control(window: &Window, mut physics: &mut PhysicsWorld, back: Key, forward: Key, bph: DefaultBodyPartHandle) {
    get_joint_mut::<RevoluteJoint<f32>>(&mut physics, bph).unwrap().set_desired_angular_motor_velocity(
        key_forward_backward(&window, back, forward)
    );
}

fn key_forward_backward(window: &Window, back: Key, forward: Key) -> f32 {
    let a = match window.get_key(back) {
        Action::Release => 0.0,
        Action::Press => -1.0
    };
    let b = match window.get_key(forward) {
        Action::Release => 0.0,
        Action::Press => 1.0
    };
    a+b
}

fn get_joint_mut<J: Joint<f32>>(physics: &mut PhysicsWorld, part_handle: DefaultBodyPartHandle) -> Option<&mut J> {
    let mut mb: &mut Multibody<_> = physics.bodies.multibody_mut(part_handle.0)?;
    let mut link: &mut MultibodyLink<_> = mb.link_mut(part_handle.1)?;
    link.joint_mut().downcast_mut()
}

// fn convert_isometry(pos: Isometry3<f32>) -> Isometry3<f32> {
//     Isometry3 {
//         rotation: UnitQuaternion::new_unchecked(Quaternion::new(pos.rotation.w as f32, pos.rotation.i as f32, pos.rotation.j as f32, pos.rotation.k as f32)),
//         translation: Translation3 {
//             vector: Vector3::new(pos.translation.x as f32, pos.translation.y as f32, pos.translation.z as f32)
//         }
//     }
// }

fn stl_to_trimesh(path: &str) -> TriMesh<f32> {
    let mut file = OpenOptions::new().read(true).open(path).unwrap();

    let stl = stl_io::read_stl(&mut file).unwrap();

    let vertices = stl.faces.iter().flat_map(|f: &IndexedTriangle| {
        f.vertices.iter().map(|i| {
            let v = stl.vertices[*i];
            Point3::new(v[0],v[1], v[2]) // Swapping is intentional
        }).collect::<Vec<_>>()
    }).collect();

    // let normals = stl.faces.iter().flat_map(|f: &IndexedTriangle| {
    //     let v = Vector3::new(f.normal[0], f.normal[2], f.normal[1]); // Swapping is intentional
    //     vec![v,v,v]
    // }).collect();

    let mut mesh = TriMesh::new(vertices, None, None, None);
    mesh.transform_by(&Isometry3::rotation(Vector3::new(-std::f32::consts::FRAC_PI_2,0.0,0.0)));
    mesh

}