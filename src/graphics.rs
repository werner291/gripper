use std::option::Option::Some;
use std::prelude::v1::Vec;

use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nalgebra::{Isometry3, Point3};
use nphysics3d::object::{Body, BodyPart, BodyPartHandle, DefaultBodyPartHandle};

use crate::physics::PhysicsWorld;

use generational_arena::{Arena, Index};

struct Trace {
    target: DefaultBodyPartHandle,
    offset: Isometry3<f32>,
    points: Vec<Point3<f32>>,
}

type TraceId = Index;

pub struct Graphics {
    pub(crate) window: Window,
    pub(crate) bp_to_sn: Vec<(SceneNode, DefaultBodyPartHandle)>,
    pub(crate) frames_drawn: u64,
    traces: Arena<Trace>,
}

impl Graphics {
    pub(crate) fn init() -> Graphics {
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

    pub fn draw_frame(&mut self, physics: &PhysicsWorld) -> bool {
        self.frames_drawn += 1;
        self.synchronize_physics_to_graphics(physics);
        self.window.render()
    }

    pub fn synchronize_physics_to_graphics(&mut self, physics: &PhysicsWorld) {
        for (sn, BodyPartHandle(bh, ph)) in self.bp_to_sn.iter_mut() {
            let pos: Isometry3<f32> = physics
                .bodies
                .get(*bh)
                .unwrap()
                .part(*ph)
                .unwrap()
                .position();

            sn.set_local_transformation(pos);
        }
    }

    fn trace(&mut self, trace: &mut Trace, physics: &PhysicsWorld) {
        if self.frames_drawn % 10 == 0 {
            let target_body: &dyn Body<f32> = physics.bodies.get(trace.target.0).unwrap();
            let target_part: &dyn BodyPart<f32> = target_body.part(trace.target.1).unwrap();

            trace
                .points
                .push(target_part.position() * &trace.offset * Point3::new(0.0, 0.0, 0.0));
        }

        if trace.points.len() >= 2 {
            for i in 0..trace.points.len() - 1 {
                self.window.draw_line(
                    &trace.points[i],
                    &trace.points[i + 1],
                    &Point3::new(1.0, 0.0, 0.0),
                );
            }
        }
    }
}
