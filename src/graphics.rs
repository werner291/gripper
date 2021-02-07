//! module concerning most graphic-related aspects of the simulator.

use std::option::Option::Some;
use std::prelude::v1::Vec;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nalgebra::{Isometry3, Point3};
use nphysics3d::object::DefaultBodyPartHandle;

use generational_arena::{Arena, Index};
use std::collections::HashMap;

/// Struct containing data necessary for visualisation of the simulation.
pub struct Graphics {
    pub window: Window,
    /// A table that associates body parts to scene nodes, used to update position information.
    pub bp_to_sn: Vec<(SceneNode, DefaultBodyPartHandle)>,
    frames_drawn: u64,
    traces: Arena<Trace>,
}

impl Graphics {

    /// Initialize the visualisation and open a window with some default settings.
    /// FIXME: The default camera settings are pretty bad, but default arcball camera is inaccessible
    pub fn init() -> Graphics {
        let mut window = Window::new("Robotic Gripper");
        window.set_framerate_limit(Some(60));
        window.set_light(Light::StickToCamera);

        Graphics {
            window,
            bp_to_sn: vec![],
            frames_drawn: 0,
            traces: Arena::new(),
        }
    }

    /// Start tracing a given target body part.
    /// Will draw a curve that displays a history of the center of the designated part.
    pub fn enable_trace(
        &mut self,
        target: DefaultBodyPartHandle,
        offset: Isometry3<f32>,
    ) -> TraceId {
        self.traces.insert(Trace {
            target,
            offset,
            points: vec![],
        })
    }

    /// Draw a frame, incrementing the view counter and processing any input/output as well.
    pub fn draw_frame(&mut self) -> bool {
        self.frames_drawn += 1;
        for (_, trace) in self.traces.iter() {
            trace.draw(&mut self.window)
        };
        self.window.render()
    }

    /// Graphics is mainly a view of a PhysicsWorld. Call this method to update that view,
    /// usually once every update of the physics world.
    pub fn synchronize_physics_to_graphics(
        &mut self,
        physics: &HashMap<DefaultBodyPartHandle, Isometry3<f32>>,
    ) {
        for (sn, bph) in self.bp_to_sn.iter_mut() {
            sn.set_local_transformation(physics[bph]);
        }

        for (_,tr) in self.traces.iter_mut() {
            tr.update(physics);
        }
    }
}


struct Trace {
    target: DefaultBodyPartHandle,
    offset: Isometry3<f32>,
    points: Vec<Point3<f32>>,
}

impl Trace {
    fn update(&mut self, physics :&HashMap<DefaultBodyPartHandle, Isometry3<f32>>) {
        self.points
            .push(&physics[&self.target] * &self.offset * Point3::new(0.0, 0.0, 0.0));
    }

    fn draw(&self, window: &mut Window) {
        if self.points.len() >= 2 {
            for i in 0..self.points.len() - 1 {
                window.draw_line(
                    &self.points[i],
                    &self.points[i + 1],
                    &Point3::new(1.0, 0.0, 0.0),
                );
            }
        }
    }
}

type TraceId = Index;