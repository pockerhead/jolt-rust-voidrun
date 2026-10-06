//! Joints that limit rotation: a lamp on a cone, a pendulum on a swing-twist joint, a swing on
//! a six-DOF joint with limits, and a puck whose body allows only translation in X and Z and
//! rotation about Y.

use oxijolt::{
    AllowedDofs, BodyId, BodySettings, ConeConstraintSettings, PhysicsWorld, SixDofAxis,
    SixDofConstraintAxis, SixDofConstraintSettings, SwingTwistConstraintSettings, Vec3,
};

use super::Parts;
use crate::digest::Digest;
use crate::draw::{colours, DrawList};
use crate::math::{position_f32, rvec};
use crate::scene::{Milestones, Result};
use crate::visual::Shaped;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const DOWN: Vec3 = Vec3::new(0.0, -1.0, 0.0);

/// Height of the hooks the lamp, the pendulum and the swing hang from.
const HOOK_HEIGHT: f32 = 4.5;
/// Where the puck starts.
const PUCK: [f32; 3] = [10.5, 0.06, 2.5];

/// The jointed bodies.
pub struct Joints {
    puck: BodyId,
    /// Ticks the puck has moved while it stayed in its plane.
    puck_ticks: u32,
    puck_worst: f32,
}

impl Joints {
    /// Builds the joints around x = 10.
    pub fn new(world: &mut PhysicsWorld, parts: &mut Parts) -> Result<Self> {
        let beam = parts.fixed(
            world,
            &Shaped::cuboid([2.6, 0.1, 0.1])?,
            [10.5, HOOK_HEIGHT + 0.1, -2.0],
        )?;
        let hook = |x: f32| [x, HOOK_HEIGHT, -2.0];

        // A lamp whose axis stays within 30° of straight down, set swinging.
        let lamp_shape = Shaped::capsule(0.35, 0.12)?;
        let lamp_at = [8.5, HOOK_HEIGHT - 0.47, -2.0];
        let lamp = parts.dynamic(world, &lamp_shape, lamp_at, 3.0, colours::PLAYER)?;
        world.create_constraint(
            beam,
            lamp,
            &ConeConstraintSettings::new(rvec(hook(8.5)), DOWN, 30.0_f32.to_radians()),
        )?;
        world
            .body_mut(lamp)?
            .set_linear_velocity(Vec3::new(2.5, 0.0, 1.5))?;

        // A pendulum on a swing-twist joint: 40° of swing one way, 15° the other, a little
        // twist.
        let rod = Shaped::capsule(0.5, 0.08)?;
        let rod_at = [10.5, HOOK_HEIGHT - 0.58, -2.0];
        let pendulum = parts.dynamic(world, &rod, rod_at, 2.0, colours::BODY_ALT)?;
        world.create_constraint(
            beam,
            pendulum,
            &SwingTwistConstraintSettings::new(rvec(hook(10.5)), DOWN, X)
                .half_cone_angles(40.0_f32.to_radians(), 15.0_f32.to_radians())
                .twist_limits(-0.2, 0.2),
        )?;
        world
            .body_mut(pendulum)?
            .set_linear_velocity(Vec3::new(1.0, 0.0, 3.0))?;

        // A swing seat on a six-DOF joint: fixed in translation, free to swing ±45° toward Z
        // (about the joint's Y, world X), ±10° sideways and not about its rope (the joint's X).
        let seat = Shaped::cuboid([0.35, 0.04, 0.2])?;
        let seat_at = [12.5, HOOK_HEIGHT - 1.2, -2.0];
        let swing = parts.dynamic(world, &seat, seat_at, 4.0, colours::BODY_THIRD)?;
        let limited = |degrees: f32| SixDofAxis::Limited {
            min: -degrees.to_radians(),
            max: degrees.to_radians(),
        };
        let mut joint = SixDofConstraintSettings::new(rvec(hook(12.5)), DOWN, X)
            .axis(SixDofConstraintAxis::RotationX, SixDofAxis::Fixed)
            .axis(SixDofConstraintAxis::RotationY, limited(45.0))
            .axis(SixDofConstraintAxis::RotationZ, limited(10.0));
        for axis in [
            SixDofConstraintAxis::TranslationX,
            SixDofConstraintAxis::TranslationY,
            SixDofConstraintAxis::TranslationZ,
        ] {
            joint = joint.axis(axis, SixDofAxis::Fixed);
        }
        world.create_constraint(beam, swing, &joint)?;
        world
            .body_mut(swing)?
            .set_linear_velocity(Vec3::new(0.0, 0.0, 2.5))?;

        // A puck that keeps to the floor's plane whatever it is given.
        let puck_shape = Shaped::cylinder(0.05, 0.3)?;
        let puck = parts.add(
            world,
            &puck_shape,
            &BodySettings::new_dynamic()
                .position(rvec(PUCK))
                .allowed_dofs(
                    AllowedDofs::TRANSLATION_X
                        | AllowedDofs::TRANSLATION_Z
                        | AllowedDofs::ROTATION_Y,
                )
                .linear_velocity(Vec3::new(-1.5, 2.0, 1.0))
                .angular_velocity(Vec3::new(2.0, 4.0, 1.0))
                .friction(0.1),
            colours::KINEMATIC,
        )?;
        Ok(Self {
            puck,
            puck_ticks: 0,
            puck_worst: 0.0,
        })
    }

    /// Checks that the puck moves and stays in its plane, upright.
    pub fn check(&mut self, world: &PhysicsWorld, milestones: &mut Milestones) -> Result<()> {
        let puck = world.body(self.puck)?;
        let at = position_f32(puck.position());
        let rotation = puck.rotation();
        let off_plane = (at[1] - PUCK[1])
            .abs()
            .max(rotation.x.abs())
            .max(rotation.z.abs());
        self.puck_worst = self.puck_worst.max(off_plane);
        let moving = crate::math::glam(puck.linear_velocity()).length() > 0.05;
        if moving && self.puck_worst < 1.0e-3 {
            self.puck_ticks += 1;
        }
        if self.puck_ticks >= 60 {
            milestones.reach("puck stayed in its plane");
        }
        Ok(())
    }

    /// The HUD line of the puck.
    pub fn draw(&self, out: &mut DrawList) {
        out.hud.push(format!(
            "puck: furthest out of its plane {:.4} (m or quaternion component)",
            self.puck_worst
        ));
    }

    /// Folds the puck's check into `digest`.
    pub fn write_state(&self, digest: &mut Digest) {
        digest.u32(self.puck_ticks);
        digest.f32(self.puck_worst);
    }
}
