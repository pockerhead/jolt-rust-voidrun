//! Motorized mechanisms: a windmill on a hinge with a velocity motor, an elevator on a slider
//! with a position motor, a gear pair driven by a hinge motor, a rack and pinion, and a cart
//! driven around a closed path.

use oxijolt::{
    BodyId, ConstraintId, GearConstraintSettings, HermitePath, HermitePathPoint, HingeConstraint,
    HingeConstraintSettings, MotorSettings, MotorState, PathConstraint, PathConstraintSettings,
    PathRotationConstraint, PhysicsWorld, RackAndPinionConstraintSettings, SliderConstraint,
    SliderConstraintSettings, SpringSettings, Vec3,
};

use super::Parts;
use crate::digest::Digest;
use crate::draw::{colours, DrawList};
use crate::input::Input;
use crate::math::{about_axis, glam, position_f32, rvec};
use crate::scene::{Milestones, Result, DT};
use crate::visual::Shaped;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

/// The elevator's top, metres above its start.
const ELEVATOR_TOP: f32 = 2.5;
/// The windmill's target speed at start, rad/s.
const WINDMILL_SPEED: f32 = 1.0;

/// The motorized mechanisms.
pub struct Mechanisms {
    windmill: ConstraintId<HingeConstraint>,
    windmill_body: BodyId,
    windmill_target: f32,
    elevator: ConstraintId<SliderConstraint>,
    elevator_up: bool,
    gear_drive: ConstraintId<HingeConstraint>,
    gears: [BodyId; 2],
    pinion_drive: ConstraintId<HingeConstraint>,
    cart: ConstraintId<PathConstraint>,
    cart_body: BodyId,
    cart_travel: f32,
    cart_last: [f32; 3],
}

impl Mechanisms {
    /// Builds the mechanisms around x = -7.
    pub fn new(world: &mut PhysicsWorld, parts: &mut Parts) -> Result<Self> {
        let post = Shaped::cuboid([0.15, 1.5, 0.15])?;
        let base = parts.fixed(world, &post, [-11.0, 1.5, 0.0])?;

        // Windmill: a cross of two blades on a hinge about Z at the post's top.
        let blade = Shaped::cuboid([1.4, 0.12, 0.04])?;
        let quarter = about_axis([0.0, 0.0, 1.0], std::f32::consts::FRAC_PI_2);
        let cross = Shaped::compound(&[
            (&blade, [0.0; 3], oxijolt::Quat::IDENTITY, 0),
            (&blade, [0.0; 3], quarter, 0),
        ])?;
        let hub = [-11.0, 3.2, 0.3];
        let windmill_body = parts.dynamic(world, &cross, hub, 20.0, colours::BODY_ALT)?;
        let windmill = world.create_constraint(
            base,
            windmill_body,
            &HingeConstraintSettings::new(rvec(hub), Z, X),
        )?;
        {
            let mut motor = world.constraint_mut(windmill)?;
            motor.set_target_angular_velocity(WINDMILL_SPEED)?;
            motor.set_motor_state(MotorState::Velocity);
        }

        // Elevator: a platform on a vertical slider with a position motor, stiffer than Jolt's
        // default 2 Hz so the platform sags less under its weight.
        let stiff_motor = MotorSettings::default().spring(SpringSettings::FrequencyAndDamping {
            frequency: 4.0,
            damping: 1.0,
        });
        let shaft = parts.fixed(world, &Shaped::cuboid([0.1, 2.0, 0.1])?, [-8.4, 2.0, -0.6])?;
        let platform_at = [-8.4, 0.2, 0.2];
        let platform = parts.dynamic(
            world,
            &Shaped::cuboid([0.6, 0.08, 0.6])?,
            platform_at,
            30.0,
            colours::BODY,
        )?;
        let elevator = world.create_constraint(
            shaft,
            platform,
            &SliderConstraintSettings::new(rvec(platform_at), Y, X)
                .limits(0.0, ELEVATOR_TOP)
                .motor(stiff_motor),
        )?;
        world
            .constraint_mut(elevator)?
            .set_motor_state(MotorState::Position);

        // Gears: two discs about Z, the second twice the size and half the speed.
        let disc = |radius: f32| Shaped::cylinder(0.05, radius);
        let on_z = about_axis([1.0, 0.0, 0.0], std::f32::consts::FRAC_PI_2);
        let gear_base = parts.fixed(world, &Shaped::cuboid([1.0, 0.1, 0.1])?, [-5.4, 0.1, -0.2])?;
        let hinged_disc = |world: &mut PhysicsWorld, parts: &mut Parts, radius, at: [f32; 3]| {
            let body =
                parts.dynamic_rotated(world, &disc(radius)?, at, on_z, 5.0, colours::BODY_THIRD)?;
            let hinge = world.create_constraint(
                gear_base,
                body,
                &HingeConstraintSettings::new(rvec(at), Z, X),
            )?;
            Result::Ok((body, hinge))
        };
        let (gear1, hinge1) = hinged_disc(world, parts, 0.4, [-5.9, 1.4, 0.3])?;
        let (gear2, hinge2) = hinged_disc(world, parts, 0.8, [-4.65, 1.4, 0.3])?;
        world.create_constraint(
            gear1,
            gear2,
            &GearConstraintSettings::new(Z, Z, 2.0).hinges(hinge1, hinge2),
        )?;
        {
            let mut motor = world.constraint_mut(hinge1)?;
            motor.set_target_angular_velocity(2.0)?;
            motor.set_motor_state(MotorState::Velocity);
        }

        // Rack and pinion: a disc of radius 0.3 turning moves a bar along X, 1/0.3 rad per m.
        let rack_base =
            parts.fixed(world, &Shaped::cuboid([1.2, 0.05, 0.1])?, [-5.3, 0.05, 1.6])?;
        let pinion_at = [-5.3, 0.75, 1.9];
        let pinion = parts.dynamic_rotated(
            world,
            &disc(0.3)?,
            pinion_at,
            on_z,
            3.0,
            colours::BODY_THIRD,
        )?;
        let pinion_drive = world.create_constraint(
            rack_base,
            pinion,
            &HingeConstraintSettings::new(rvec(pinion_at), Z, X),
        )?;
        let rack_at = [-5.3, 0.37, 1.9];
        let rack = parts.dynamic(
            world,
            &Shaped::cuboid([0.9, 0.06, 0.06])?,
            rack_at,
            3.0,
            colours::BODY,
        )?;
        let rack_slider = world.create_constraint(
            rack_base,
            rack,
            &SliderConstraintSettings::new(rvec(rack_at), X, Y).limits(-0.9, 0.9),
        )?;
        world.create_constraint(
            pinion,
            rack,
            &RackAndPinionConstraintSettings::new(Z, X, 1.0 / 0.3)
                .constraints(pinion_drive, rack_slider),
        )?;
        world
            .constraint_mut(pinion_drive)?
            .set_motor_state(MotorState::Velocity);

        // A cart driven around a closed loop of eight points.
        let track = parts.fixed(world, &Shaped::cuboid([0.2, 0.2, 0.2])?, [-8.0, 0.2, 4.0])?;
        let loop_point = |i: usize| {
            let angle = i as f32 / 8.0 * std::f32::consts::TAU;
            let (sin, cos) = angle.sin_cos();
            HermitePathPoint {
                position: Vec3::new(2.0 * cos, 0.5, 1.2 * sin),
                tangent: Vec3::new(-2.0 * sin * 0.8, 0.0, 1.2 * cos * 0.8),
            }
        };
        let path = HermitePath::new(Y, (0..8).map(loop_point).collect(), true)?;
        let start = loop_point(0).position;
        let cart_at = [-8.0 + start.x, 0.2 + start.y, 4.0 + start.z];
        let cart_body = parts.dynamic(
            world,
            &Shaped::cuboid([0.25, 0.12, 0.18])?,
            cart_at,
            5.0,
            colours::KINEMATIC,
        )?;
        let cart = world.create_constraint(
            track,
            cart_body,
            &PathConstraintSettings::new(path)
                .rotation_constraint(PathRotationConstraint::ConstrainToPath),
        )?;
        {
            let mut motor = world.constraint_mut(cart)?;
            motor.set_target_velocity(1.5)?;
            motor.set_motor_state(MotorState::Velocity);
        }

        Ok(Self {
            windmill,
            windmill_body,
            windmill_target: WINDMILL_SPEED,
            elevator,
            elevator_up: false,
            gear_drive: hinge1,
            gears: [gear1, gear2],
            pinion_drive,
            cart,
            cart_body,
            cart_travel: 0.0,
            cart_last: cart_at,
        })
    }

    /// Applies the keys: arrows change the windmill's speed, E sends the elevator up or down;
    /// the pinion turns one way, then the other, every two seconds.
    pub fn update(&mut self, world: &mut PhysicsWorld, input: &Input, tick: u32) -> Result<()> {
        if input.held.up_down != 0.0 {
            self.windmill_target =
                (self.windmill_target + input.held.up_down * 0.5 * DT).clamp(-3.0, 3.0);
            world
                .constraint_mut(self.windmill)?
                .set_target_angular_velocity(self.windmill_target)?;
        }
        if input.edges.action {
            self.elevator_up = !self.elevator_up;
            let target = if self.elevator_up { ELEVATOR_TOP } else { 0.0 };
            world
                .constraint_mut(self.elevator)?
                .set_target_position(target)?;
        }
        let pinion_speed = if (tick / 120).is_multiple_of(2) { 1.2 } else { -1.2 };
        world
            .constraint_mut(self.pinion_drive)?
            .set_target_angular_velocity(pinion_speed)?;
        Ok(())
    }

    /// Records the milestones after a step.
    pub fn check(&mut self, world: &PhysicsWorld, milestones: &mut Milestones) -> Result<()> {
        if glam(world.body(self.windmill_body)?.angular_velocity()).length() > 0.5 {
            milestones.reach("windmill turning");
        }
        if world.constraint(self.elevator)?.current_position() > ELEVATOR_TOP - 0.1 {
            milestones.reach("elevator at top");
        }
        let spin1 = world.body(self.gears[0])?.angular_velocity().z;
        let spin2 = world.body(self.gears[1])?.angular_velocity().z;
        if spin1 > 1.0 && (spin2 + spin1 / 2.0).abs() < 0.2 {
            milestones.reach("gear driven");
        }
        let at = position_f32(world.body(self.cart_body)?.position());
        self.cart_travel += glam::Vec3::from(at).distance(glam::Vec3::from(self.cart_last));
        self.cart_last = at;
        if self.cart_travel >= 3.0 {
            milestones.reach("cart moved along path");
        }
        Ok(())
    }

    /// The HUD lines of the motors.
    pub fn draw(&self, world: &PhysicsWorld, out: &mut DrawList) -> Result<()> {
        let windmill = world.body(self.windmill_body)?.angular_velocity().z;
        out.hud.push(format!(
            "windmill: target {:.1} rad/s, turning {windmill:.1} rad/s (arrows change it)",
            self.windmill_target
        ));
        let elevator = world.constraint(self.elevator)?.current_position();
        let target = if self.elevator_up { ELEVATOR_TOP } else { 0.0 };
        out.hud.push(format!(
            "elevator: target {target:.1} m, at {elevator:.2} m (E)"
        ));
        let angle = world.constraint(self.gear_drive)?.current_angle();
        let fraction = world.constraint(self.cart)?.path_fraction();
        out.hud.push(format!(
            "small gear at {angle:.2} rad; cart at path fraction {fraction:.2}"
        ));
        Ok(())
    }

    /// Folds the mechanisms' control state into `digest`.
    pub fn write_state(&self, digest: &mut Digest) {
        digest.f32(self.windmill_target);
        digest.bool(self.elevator_up);
        digest.f32(self.cart_travel);
    }
}
