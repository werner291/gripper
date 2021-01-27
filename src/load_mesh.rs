use std::fs::OpenOptions;
use std::iter::Iterator;
use std::option::Option::None;
use std::prelude::v1::Vec;

use kiss3d::ncollide3d::na::{Isometry3, Point3, Vector3};
use kiss3d::ncollide3d::procedural::TriMesh;
use stl_io::IndexedTriangle;

/// Loads the stl file at the specified path into a ncollide3d::procedural::TriMesh.
///
/// Note: Rotates the mesh to make the Y axis point up instead of Z,
/// since OpenSCAD uses a different convention.
pub fn stl_to_trimesh(path: &str) -> TriMesh<f32> {
    let mut file = OpenOptions::new().read(true).open(path).unwrap();

    let stl = stl_io::read_stl(&mut file).unwrap();

    let vertices = stl
        .faces
        .iter()
        .flat_map(|f: &IndexedTriangle| {
            f.vertices
                .iter()
                .map(|i| {
                    let v = stl.vertices[*i];
                    Point3::new(v[0], v[1], v[2])
                })
                .collect::<Vec<_>>()
        })
        .collect();

    let mut mesh = TriMesh::new(vertices, None, None, None);
    mesh.transform_by(&Isometry3::rotation(Vector3::new(
        -std::f32::consts::FRAC_PI_2,
        0.0,
        0.0,
    )));
    mesh
}
