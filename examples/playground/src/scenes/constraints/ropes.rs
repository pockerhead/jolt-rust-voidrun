//! Things that hang: a plank bridge of point constraints, a rope of distance constraints, a
//! pulley pair and two boxes welded by a fixed constraint.

use oxijolt::{
    BodyId, DistanceConstraintSettings, FixedConstraintSettings, PhysicsWorld,
    PointConstraintSettings, PulleyConstraintSettings,
};

use super::Parts;
use crate::draw::{colours, DrawList};
use crate::math::{about_axis, position_f32, rvec};
use crate::scene::Result;
use crate::visual::Shaped;

/// Planks of the bridge and their length, metres.
const PLANKS: usize = 8;
const PLANK_LENGTH: f32 = 0.76;
/// The bridge's ends.
const BRIDGE: ([f32; 3], [f32; 3]) = ([-0.5, 1.8, -3.0], [5.5, 1.8, -3.0]);
/// The pulley's fixed points.
const PULLEY: [[f32; 3]; 2] = [[3.0, 4.5, 2.0], [4.4, 4.5, 2.0]];

/// The hanging things, and the points their ropes are drawn between.
pub struct Ropes {
    rope: Vec<BodyId>,
    rope_anchor: [f32; 3],
    pulley: [BodyId; 2],
}

impl Ropes {
    /// Builds the bridge, the rope, the pulley pair and the welded pair around x = 2.5.
    pub fn new(world: &mut PhysicsWorld, parts: &mut Parts) -> Result<Self> {
        Self::bridge(world, parts)?;

        // A rope of five balls laid out sideways from a fixed hook, so it swings down.
        let rope_anchor = [0.5, 4.5, 1.2];
        let hook = parts.fixed(world, &Shaped::cuboid([0.08; 3])?, rope_anchor)?;
        let ball = Shaped::sphere(0.12)?;
        let mut rope = Vec::new();
        let mut previous = (hook, rope_anchor);
        for i in 1..=5 {
            let at = [
                rope_anchor[0] + 0.4 * i as f32,
                rope_anchor[1],
                rope_anchor[2],
            ];
            let body = parts.dynamic(world, &ball, at, 1.0, colours::BODY)?;
            world.create_constraint(
                previous.0,
                body,
                &DistanceConstraintSettings::new(rvec(previous.1), rvec(at)),
            )?;
            rope.push(body);
            previous = (body, at);
        }

        // Two boxes over two fixed pulleys; the heavier one sinks and lifts the other.
        let box_shape = Shaped::cuboid([0.2; 3])?;
        let light_at = [PULLEY[0][0], 2.4, PULLEY[0][2]];
        let heavy_at = [PULLEY[1][0], 2.6, PULLEY[1][2]];
        let light = parts.dynamic(world, &box_shape, light_at, 2.0, colours::BODY_ALT)?;
        let heavy = parts.dynamic(world, &box_shape, heavy_at, 3.0, colours::BODY)?;
        let top = |at: [f32; 3]| [at[0], at[1] + 0.2, at[2]];
        world.create_constraint(
            light,
            heavy,
            &PulleyConstraintSettings::new(
                rvec(top(light_at)),
                rvec(PULLEY[0]),
                rvec(top(heavy_at)),
                rvec(PULLEY[1]),
            ),
        )?;

        // Two boxes welded at an angle, dropped as one.
        let left = parts.dynamic(world, &box_shape, [1.0, 2.5, 3.2], 2.0, colours::BODY_THIRD)?;
        let tilted = about_axis([0.0, 0.0, 1.0], 0.6);
        let right = parts.dynamic_rotated(
            world,
            &box_shape,
            [1.45, 2.8, 3.2],
            tilted,
            2.0,
            colours::BODY_THIRD,
        )?;
        world.create_constraint(
            left,
            right,
            &FixedConstraintSettings::default().auto_detect_point(true),
        )?;

        Ok(Self {
            rope,
            rope_anchor,
            pulley: [light, heavy],
        })
    }

    /// Planks hanging in a sag between two posts, each joined to the next by a point
    /// constraint at their shared edge.
    fn bridge(world: &mut PhysicsWorld, parts: &mut Parts) -> Result<()> {
        let (start, end) = BRIDGE;
        let span = end[0] - start[0];
        // The planks' slopes run evenly from -dip to +dip; find the dip whose run is the span.
        let run = |dip: f32| -> f32 {
            (0..PLANKS)
                .map(|i| {
                    let t = (i as f32 + 0.5) / PLANKS as f32;
                    PLANK_LENGTH * (dip * (2.0 * t - 1.0)).cos()
                })
                .sum()
        };
        let (mut low, mut high) = (0.0_f32, 1.5_f32);
        for _ in 0..40 {
            let mid = (low + high) / 2.0;
            if run(mid) > span {
                low = mid;
            } else {
                high = mid;
            }
        }
        let dip = (low + high) / 2.0;
        let post = Shaped::cuboid([0.1, 0.9, 0.5])?;
        let first_post = parts.fixed(world, &post, [start[0] - 0.1, 0.9, start[2]])?;
        let last_post = parts.fixed(world, &post, [end[0] + 0.1, 0.9, end[2]])?;
        let plank = Shaped::cuboid([PLANK_LENGTH / 2.0 - 0.02, 0.04, 0.4])?;
        let mut edge = start;
        let mut previous = first_post;
        for i in 0..PLANKS {
            let t = (i as f32 + 0.5) / PLANKS as f32;
            let slope = dip * (2.0 * t - 1.0);
            let (sin, cos) = slope.sin_cos();
            let next_edge = [
                edge[0] + PLANK_LENGTH * cos,
                edge[1] + PLANK_LENGTH * sin,
                edge[2],
            ];
            let centre = [
                (edge[0] + next_edge[0]) / 2.0,
                (edge[1] + next_edge[1]) / 2.0,
                edge[2],
            ];
            let rotation = about_axis([0.0, 0.0, 1.0], slope);
            let body =
                parts.dynamic_rotated(world, &plank, centre, rotation, 4.0, colours::BODY)?;
            world.create_constraint(previous, body, &PointConstraintSettings::new(rvec(edge)))?;
            previous = body;
            edge = next_edge;
        }
        world.create_constraint(
            previous,
            last_post,
            &PointConstraintSettings::new(rvec(edge)),
        )?;
        Ok(())
    }

    /// The rope and the pulley's ropes as lines.
    pub fn draw(&self, world: &PhysicsWorld, out: &mut DrawList) -> Result<()> {
        let mut previous = self.rope_anchor;
        for &ball in &self.rope {
            let at = position_f32(world.body(ball)?.position());
            out.line(previous, at, colours::QUERY);
            previous = at;
        }
        for (body, fixed) in self.pulley.iter().zip(PULLEY) {
            let at = position_f32(world.body(*body)?.position());
            out.line([at[0], at[1] + 0.2, at[2]], fixed, colours::QUERY);
        }
        out.line(PULLEY[0], PULLEY[1], colours::QUERY);
        Ok(())
    }
}
