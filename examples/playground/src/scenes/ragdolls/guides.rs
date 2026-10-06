//! Guides for a ragdoll's joint limits: each joint's allowed cone, pyramid or hinge arc in its
//! parent's current frame, and a line from the joint toward where the child part is now.

use glam::{Quat, Vec3};
use oxijolt::{BodyId, PhysicsWorld};

use super::humanoid::{bind_rotation, limits, Limits, PARTS};
use crate::draw::{colours, DrawList};
use crate::math::{glam, glam_quat, position};
use crate::scene::Result;

/// Length of the guide lines, metres.
const REACH: f32 = 0.22;
/// Steps along a cone, a pyramid or an arc.
const STEPS: usize = 16;
/// Colour of the limits.
const GUIDE: [f32; 3] = colours::KINEMATIC;

/// Draws the guides of the ragdoll whose part bodies are `bodies`, in skeleton order.
pub fn draw(world: &PhysicsWorld, bodies: &[BodyId], out: &mut DrawList) -> Result<()> {
    for (index, part) in PARTS.iter().enumerate() {
        let (Some(parent), Some(limits)) = (part.parent, limits(index)) else {
            continue;
        };
        let parent = parent as usize;
        let reading = world.body(bodies[parent])?;
        // From the bind pose's model space into the world, through the parent's pose now.
        let turn =
            glam_quat(reading.rotation()) * glam_quat(bind_rotation(&PARTS[parent])).inverse();
        let origin = position(reading.position()) - turn * Vec3::from(PARTS[parent].centre);
        let (at, directions, closed) = boundary(limits);
        let apex = origin + turn * Vec3::from(at);
        let tips: Vec<Vec3> = directions
            .iter()
            .map(|&direction| apex + turn * direction * REACH)
            .collect();
        for (step, pair) in tips.windows(2).enumerate() {
            out.line(pair[0].to_array(), pair[1].to_array(), GUIDE);
            if closed && step % 4 == 0 {
                out.line(apex.to_array(), pair[0].to_array(), GUIDE);
            }
        }
        if closed {
            out.line(tips[tips.len() - 1].to_array(), tips[0].to_array(), GUIDE);
        } else {
            for tip in [tips[0], tips[tips.len() - 1]] {
                out.line(apex.to_array(), tip.to_array(), GUIDE);
            }
        }
        let child = position(world.body(bodies[index])?.position());
        let bone = apex + (child - apex).normalize_or_zero() * (1.3 * REACH);
        out.line(apex.to_array(), bone.to_array(), colours::HIGHLIGHT);
    }
    Ok(())
}

/// The joint point and the directions along the edge of what `limits` allow, in model space,
/// and whether the edge closes on itself.
fn boundary(limits: Limits) -> ([f32; 3], Vec<Vec3>, bool) {
    match limits {
        Limits::Cone {
            at,
            twist_axis,
            plane_axis,
            half_cone,
            ..
        } => {
            let frame = Frame::new(glam(twist_axis), glam(plane_axis));
            let t = (half_cone / 2.0).tan();
            let directions = (0..STEPS)
                .map(|step| {
                    let angle = step as f32 / STEPS as f32 * std::f32::consts::TAU;
                    frame.swung(t * angle.cos(), t * angle.sin())
                })
                .collect();
            (at, directions, true)
        }
        Limits::Hip {
            at,
            twist_axis,
            plane_axis,
            flexion,
            abduction,
            ..
        } => {
            // Jolt's pyramid bounds the half angles of the swing about the plane axis and its
            // normal: the edge runs around the rectangle of those bounds.
            let frame = Frame::new(glam(twist_axis), glam(plane_axis));
            let corners = [
                (flexion.0, abduction.0),
                (flexion.1, abduction.0),
                (flexion.1, abduction.1),
                (flexion.0, abduction.1),
            ];
            let side = STEPS / 4;
            let mut directions = Vec::with_capacity(STEPS);
            for (corner, &(a0, b0)) in corners.iter().enumerate() {
                let (a1, b1) = corners[(corner + 1) % 4];
                for step in 0..side {
                    let f = step as f32 / side as f32;
                    let (a, b) = (a0 + (a1 - a0) * f, b0 + (b1 - b0) * f);
                    directions.push(frame.swung((a / 2.0).tan(), (b / 2.0).tan()));
                }
            }
            (at, directions, true)
        }
        Limits::Hinge {
            at,
            axis,
            normal,
            min,
            max,
        } => {
            let (axis, normal) = (glam(axis), glam(normal));
            let directions = (0..=STEPS)
                .map(|step| {
                    let angle = min + (max - min) * step as f32 / STEPS as f32;
                    Quat::from_axis_angle(axis, angle) * normal
                })
                .collect();
            (at, directions, false)
        }
    }
}

/// A joint's constraint frame: X along the twist axis, Y along the plane axis.
struct Frame {
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Frame {
    fn new(twist_axis: Vec3, plane_axis: Vec3) -> Self {
        Self {
            x: twist_axis,
            y: plane_axis,
            z: twist_axis.cross(plane_axis),
        }
    }

    /// The twist axis turned by the swing whose quaternion has `tan_y` and `tan_z` as its y and
    /// z parts over its w part.
    fn swung(&self, tan_y: f32, tan_z: f32) -> Vec3 {
        let swing = Quat::from_xyzw(0.0, tan_y, tan_z, 1.0).normalize();
        let local = swing * Vec3::X;
        self.x * local.x + self.y * local.y + self.z * local.z
    }
}
