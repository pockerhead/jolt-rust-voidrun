//! Rigid body contact events: what joltc's contact listener reports, copied into Rust values.

use std::ffi::c_void;
use std::fmt;

use oxijolt_sys::*;

use super::estimate::estimate;
use super::{Callback, CollisionEstimate, ListenerContext};
use crate::world::WorldTag;
use crate::{limits, BodyId, ContactSettingsError, RVec3, SubShapeId, Vec3};

/// The two sides of a contact: each body with the sub-shape (compound child, heightfield
/// triangle) that touches. Jolt orders the sides so that `body1` has the lower id
/// ([`BodyId::to_raw`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SubShapeIdPair {
    /// The body with the lower id.
    pub body1: BodyId,
    /// The sub-shape of `body1` that touches.
    pub sub_shape1: SubShapeId,
    /// The body with the higher id.
    pub body2: BodyId,
    /// The sub-shape of `body2` that touches.
    pub sub_shape2: SubShapeId,
}

/// One contact point, as a point on each body's surface, in world space (metres).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    /// The point on body 1.
    pub on1: RVec3,
    /// The point on body 2.
    pub on2: RVec3,
}

/// The contact between two sub-shapes in one step (Jolt `ContactManifold`).
#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// The bodies and sub-shapes that touch.
    pub pair: SubShapeIdPair,
    /// Unit normal in world space along which body 2 moves out of body 1.
    pub normal: Vec3,
    /// How far the bodies overlap along `normal`, in metres. Negative for a speculative
    /// contact, which Jolt reports before the bodies touch.
    pub penetration_depth: f32,
    /// The contact points, at most 64 (Jolt's manifold reduction usually leaves 4 or fewer).
    pub points: Vec<ContactPoint>,
    /// The user data of the [`PhysicsMaterial`](crate::PhysicsMaterial) of each side's
    /// sub-shape, `None` for a sub-shape without one.
    pub materials: [Option<u64>; 2],
}

/// How Jolt resolves a contact (Jolt `ContactSettings`), with its values for this contact.
///
/// A [`ContactListener`](crate::ContactListener) may change them through the setters, which
/// refuse values outside the ranges they state; `docs/limits.md` (section "Contact settings")
/// says why. The setters check against the contact the value was read for. A listener that
/// replaces the whole value, for example with settings kept from another contact, gets it
/// checked again against the contact it is called for; see
/// [`ContactListener`](crate::ContactListener) for what happens when that check fails.
///
/// Equality and `Debug` cover the values Jolt resolves the contact with.
#[derive(Clone, Copy)]
pub struct ContactSettings {
    combined_friction: f32,
    combined_restitution: f32,
    inv_mass_scale1: f32,
    inv_inertia_scale1: f32,
    inv_mass_scale2: f32,
    inv_inertia_scale2: f32,
    is_sensor: bool,
    relative_linear_surface_velocity: Vec3,
    relative_angular_surface_velocity: Vec3,
    /// The facts of the contact the value was read for, which the setters check against.
    facts: ContactFacts,
}

/// What a contact's settings are checked against: facts of the contact, which a listener cannot
/// change.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct ContactFacts {
    /// Whether either body is a sensor, which keeps the contact a sensor contact.
    pub(super) sensor_body: bool,
    /// The largest distance from body 1's centre of mass to a contact point, in metres: the
    /// lever of the angular surface velocity.
    pub(super) lever_arm: f64,
}

impl ContactSettings {
    /// Jolt's settings of a contact with that contact's facts.
    pub(super) fn new(settings: &JPH_ContactSettings, facts: ContactFacts) -> Self {
        Self {
            combined_friction: settings.combinedFriction,
            combined_restitution: settings.combinedRestitution,
            inv_mass_scale1: settings.invMassScale1,
            inv_inertia_scale1: settings.invInertiaScale1,
            inv_mass_scale2: settings.invMassScale2,
            inv_inertia_scale2: settings.invInertiaScale2,
            is_sensor: settings.isSensor != 0,
            relative_linear_surface_velocity: Vec3::from_jph(
                settings.relativeLinearSurfaceVelocity,
            ),
            relative_angular_surface_velocity: Vec3::from_jph(
                settings.relativeAngularSurfaceVelocity,
            ),
            facts,
        }
    }

    /// These settings for the contact with `facts`, or why that contact cannot take them.
    ///
    /// Checks every value as its setter would for that contact, so a value moved over from
    /// another contact is held to this one's sensor bodies and lever.
    pub(super) fn checked_for(mut self, facts: ContactFacts) -> Result<Self, ContactSettingsError> {
        self.facts = facts;
        check(
            limits::is_friction(self.combined_friction),
            ContactSettingsError::Friction,
        )?;
        check(
            is_unit_interval(self.combined_restitution),
            ContactSettingsError::Restitution,
        )?;
        check(
            limits::is_contact_scale(self.inv_mass_scale1)
                && limits::is_contact_scale(self.inv_mass_scale2),
            ContactSettingsError::InverseMassScale,
        )?;
        check(
            limits::is_contact_scale(self.inv_inertia_scale1)
                && limits::is_contact_scale(self.inv_inertia_scale2),
            ContactSettingsError::InverseInertiaScale,
        )?;
        check(
            self.is_sensor || !facts.sensor_body,
            ContactSettingsError::SensorBody,
        )?;
        let (linear, angular) = (
            self.relative_linear_surface_velocity,
            self.relative_angular_surface_velocity,
        );
        check(
            limits::is_linear_velocity(linear)
                && limits::is_angular_velocity(angular)
                && self.is_surface_velocity(linear, angular),
            ContactSettingsError::SurfaceVelocity,
        )?;
        Ok(self)
    }

    /// Writes every value back to joltc's copy, which joltc then copies to Jolt.
    pub(super) fn write_to(&self, settings: &mut JPH_ContactSettings) {
        settings.combinedFriction = self.combined_friction;
        settings.combinedRestitution = self.combined_restitution;
        settings.invMassScale1 = self.inv_mass_scale1;
        settings.invInertiaScale1 = self.inv_inertia_scale1;
        settings.invMassScale2 = self.inv_mass_scale2;
        settings.invInertiaScale2 = self.inv_inertia_scale2;
        settings.isSensor = u32::from(self.is_sensor);
        settings.relativeLinearSurfaceVelocity = self.relative_linear_surface_velocity.to_jph();
        settings.relativeAngularSurfaceVelocity = self.relative_angular_surface_velocity.to_jph();
    }

    /// The values Jolt resolves the contact with, without the facts they are checked against.
    fn values(&self) -> (f32, f32, [f32; 4], bool, Vec3, Vec3) {
        (
            self.combined_friction,
            self.combined_restitution,
            [
                self.inv_mass_scale1,
                self.inv_inertia_scale1,
                self.inv_mass_scale2,
                self.inv_inertia_scale2,
            ],
            self.is_sensor,
            self.relative_linear_surface_velocity,
            self.relative_angular_surface_velocity,
        )
    }

    /// The setter facts as bits, for the canonical order.
    pub(super) fn rule_bits(&self) -> (bool, u64) {
        (self.facts.sensor_body, self.facts.lever_arm.to_bits())
    }

    /// The friction of the contact, by default Jolt's `sqrt(friction1 * friction2)` of the two
    /// bodies.
    pub fn combined_friction(&self) -> f32 {
        self.combined_friction
    }

    /// Sets the friction, `0..=`[`limits::MAX_FRICTION`] like a body's.
    pub fn set_combined_friction(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(limits::is_friction(value), ContactSettingsError::Friction)?;
        self.combined_friction = value;
        Ok(())
    }

    /// The restitution of the contact, by default the larger of the two bodies'.
    pub fn combined_restitution(&self) -> f32 {
        self.combined_restitution
    }

    /// Sets the restitution, `0..=1` like a body's.
    pub fn set_combined_restitution(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(is_unit_interval(value), ContactSettingsError::Restitution)?;
        self.combined_restitution = value;
        Ok(())
    }

    /// Factor on body 1's inverse mass for this contact; 1 by default.
    pub fn inv_mass_scale1(&self) -> f32 {
        self.inv_mass_scale1
    }

    /// Sets the factor on body 1's inverse mass, 0 or [`limits::MIN_CONTACT_SCALE`]`..=1`: 0
    /// makes body 1 immovable for this contact. When no dynamic body of the contact keeps a
    /// factor above 0, Jolt drops the contact and the bodies pass through each other.
    pub fn set_inv_mass_scale1(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(
            limits::is_contact_scale(value),
            ContactSettingsError::InverseMassScale,
        )?;
        self.inv_mass_scale1 = value;
        Ok(())
    }

    /// Factor on body 1's inverse inertia for this contact; 1 by default.
    pub fn inv_inertia_scale1(&self) -> f32 {
        self.inv_inertia_scale1
    }

    /// Sets the factor on body 1's inverse inertia, 0 or
    /// [`limits::MIN_CONTACT_SCALE`]`..=1`.
    pub fn set_inv_inertia_scale1(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(
            limits::is_contact_scale(value),
            ContactSettingsError::InverseInertiaScale,
        )?;
        self.inv_inertia_scale1 = value;
        Ok(())
    }

    /// Factor on body 2's inverse mass for this contact; 1 by default.
    pub fn inv_mass_scale2(&self) -> f32 {
        self.inv_mass_scale2
    }

    /// Sets the factor on body 2's inverse mass, 0 or [`limits::MIN_CONTACT_SCALE`]`..=1`.
    pub fn set_inv_mass_scale2(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(
            limits::is_contact_scale(value),
            ContactSettingsError::InverseMassScale,
        )?;
        self.inv_mass_scale2 = value;
        Ok(())
    }

    /// Factor on body 2's inverse inertia for this contact; 1 by default.
    pub fn inv_inertia_scale2(&self) -> f32 {
        self.inv_inertia_scale2
    }

    /// Sets the factor on body 2's inverse inertia, 0 or
    /// [`limits::MIN_CONTACT_SCALE`]`..=1`.
    pub fn set_inv_inertia_scale2(&mut self, value: f32) -> Result<(), ContactSettingsError> {
        check(
            limits::is_contact_scale(value),
            ContactSettingsError::InverseInertiaScale,
        )?;
        self.inv_inertia_scale2 = value;
        Ok(())
    }

    /// Whether the contact only reports and does not push the bodies apart; by default when
    /// either body is a sensor.
    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    /// Makes the contact a sensor contact or an ordinary one. A contact with a sensor body
    /// stays a sensor contact ([`ContactSettingsError::SensorBody`]), as Jolt requires.
    pub fn set_is_sensor(&mut self, value: bool) -> Result<(), ContactSettingsError> {
        check(
            value || !self.facts.sensor_body,
            ContactSettingsError::SensorBody,
        )?;
        self.is_sensor = value;
        Ok(())
    }

    /// Velocity of body 2's surface relative to body 1's at the contact, in m/s in world space
    /// (a conveyor belt); zero by default.
    pub fn relative_linear_surface_velocity(&self) -> Vec3 {
        self.relative_linear_surface_velocity
    }

    /// Sets the relative linear surface velocity, within [`limits::MAX_LINEAR_VELOCITY`];
    /// together with the angular one see
    /// [`set_relative_angular_surface_velocity`](Self::set_relative_angular_surface_velocity).
    pub fn set_relative_linear_surface_velocity(
        &mut self,
        value: Vec3,
    ) -> Result<(), ContactSettingsError> {
        let valid = limits::is_linear_velocity(value)
            && self.is_surface_velocity(value, self.relative_angular_surface_velocity);
        check(valid, ContactSettingsError::SurfaceVelocity)?;
        self.relative_linear_surface_velocity = value;
        Ok(())
    }

    /// Angular velocity of body 2's surface relative to body 1's, in rad/s in world space;
    /// zero by default.
    pub fn relative_angular_surface_velocity(&self) -> Vec3 {
        self.relative_angular_surface_velocity
    }

    /// Sets the relative angular surface velocity, within [`limits::MAX_ANGULAR_VELOCITY`].
    ///
    /// Jolt applies `v + ω × r` at the contact, `r` reaching from body 1's centre of mass to
    /// the contact, so the two velocities are also bounded together: `|v| + |ω| · R` may not
    /// exceed [`limits::MAX_LINEAR_VELOCITY`], `R` the largest distance from body 1's centre
    /// of mass to a point of this contact.
    pub fn set_relative_angular_surface_velocity(
        &mut self,
        value: Vec3,
    ) -> Result<(), ContactSettingsError> {
        let valid = limits::is_angular_velocity(value)
            && self.is_surface_velocity(self.relative_linear_surface_velocity, value);
        check(valid, ContactSettingsError::SurfaceVelocity)?;
        self.relative_angular_surface_velocity = value;
        Ok(())
    }

    /// Whether `|linear| + |angular| · R <= MAX_LINEAR_VELOCITY`, in `f64`.
    fn is_surface_velocity(&self, linear: Vec3, angular: Vec3) -> bool {
        let length = |v: Vec3| {
            let [x, y, z] = [v.x, v.y, v.z].map(f64::from);
            (x * x + y * y + z * z).sqrt()
        };
        length(linear) + length(angular) * self.facts.lever_arm
            <= f64::from(limits::MAX_LINEAR_VELOCITY)
    }
}

impl PartialEq for ContactSettings {
    fn eq(&self, other: &Self) -> bool {
        self.values() == other.values()
    }
}

impl fmt::Debug for ContactSettings {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ContactSettings")
            .field("combined_friction", &self.combined_friction)
            .field("combined_restitution", &self.combined_restitution)
            .field("inv_mass_scale1", &self.inv_mass_scale1)
            .field("inv_inertia_scale1", &self.inv_inertia_scale1)
            .field("inv_mass_scale2", &self.inv_mass_scale2)
            .field("inv_inertia_scale2", &self.inv_inertia_scale2)
            .field("is_sensor", &self.is_sensor)
            .field(
                "relative_linear_surface_velocity",
                &self.relative_linear_surface_velocity,
            )
            .field(
                "relative_angular_surface_velocity",
                &self.relative_angular_surface_velocity,
            )
            .finish()
    }
}

/// `Ok` when `valid`, otherwise `error`.
pub(super) fn check(valid: bool, error: ContactSettingsError) -> Result<(), ContactSettingsError> {
    if valid {
        Ok(())
    } else {
        Err(error)
    }
}

/// Whether `value` is finite and in `0..=1`.
fn is_unit_interval(value: f32) -> bool {
    (0.0..=1.0).contains(&value)
}

/// A change of a rigid body contact in a step.
///
/// Jolt reports a contact when it appears (Added), in each later step that it lasts
/// (Persisted), and in the step after it ended (Removed): when the bodies separated, when one
/// of them fell asleep, or when one was removed from the world, so a Removed pair can name a
/// body that no longer exists. Contacts Jolt dropped because a fixed-size buffer was full (see
/// [`StepReport`](crate::StepReport)) are not reported. Character movement and vehicle wheels
/// use their own collision queries and report nothing here; a character's inner body and a
/// vehicle's chassis are ordinary bodies and do.
#[derive(Clone, Debug, PartialEq)]
pub enum ContactEvent {
    /// A contact that was not there in the previous step.
    Added {
        /// Where the bodies touch.
        manifold: ContactManifold,
        /// The settings Jolt resolves the contact with.
        settings: ContactSettings,
        /// Jolt's estimate of the impact, when
        /// [`EventSettings::collision_estimates`](crate::EventSettings::collision_estimates) is
        /// on and the contact is not a sensor contact.
        estimate: Option<CollisionEstimate>,
    },
    /// A contact that was there in the previous step too.
    Persisted {
        /// Where the bodies touch.
        manifold: ContactManifold,
        /// The settings Jolt resolves the contact with.
        settings: ContactSettings,
    },
    /// A contact of the previous step that is gone.
    Removed(SubShapeIdPair),
}

impl ContactEvent {
    /// The bodies and sub-shapes of the contact.
    pub fn pair(&self) -> SubShapeIdPair {
        match self {
            Self::Added { manifold, .. } | Self::Persisted { manifold, .. } => manifold.pair,
            Self::Removed(pair) => *pair,
        }
    }
}

/// Contact settings a [`ContactListener`](crate::ContactListener) returned that do not fit the
/// contact it was called for, such as a value kept from another contact (see
/// [`ContactSettings`]). Jolt resolved the contact with its own settings.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContactSettingsRejection {
    /// The contact the listener was called for.
    pub pair: SubShapeIdPair,
    /// The first rule the returned settings break.
    pub error: ContactSettingsError,
}

/// Reads the manifold Jolt passes to a contact callback.
///
/// # Safety
/// The bodies and the manifold are the live ones of the callback.
unsafe fn read_manifold(
    world: WorldTag,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
) -> ContactManifold {
    // SAFETY: the callback arguments are live for the callback (contract); every getter only
    // reads, and the point indices are below the manifold's point count.
    unsafe {
        let sub_shape1 = JPH_ContactManifold_GetSubShapeID1(manifold);
        let sub_shape2 = JPH_ContactManifold_GetSubShapeID2(manifold);
        let pair = SubShapeIdPair {
            body1: BodyId::new(JPH_Body_GetID(body1), world),
            sub_shape1: SubShapeId::new(sub_shape1),
            body2: BodyId::new(JPH_Body_GetID(body2), world),
            sub_shape2: SubShapeId::new(sub_shape2),
        };
        let mut normal = Vec3::ZERO.to_jph();
        JPH_ContactManifold_GetWorldSpaceNormal(manifold, &mut normal);
        let count = JPH_ContactManifold_GetPointCount(manifold);
        let points = (0..count)
            .map(|index| {
                let (mut on1, mut on2) = (RVec3::ZERO.to_jph(), RVec3::ZERO.to_jph());
                JPH_ContactManifold_GetWorldSpaceContactPointOn1(manifold, index, &mut on1);
                JPH_ContactManifold_GetWorldSpaceContactPointOn2(manifold, index, &mut on2);
                ContactPoint {
                    on1: RVec3::from_jph(on1),
                    on2: RVec3::from_jph(on2),
                }
            })
            .collect();
        ContactManifold {
            pair,
            normal: Vec3::from_jph(normal),
            penetration_depth: JPH_ContactManifold_GetPenetrationDepth(manifold),
            points,
            materials: [
                body_material(body1, sub_shape1),
                body_material(body2, sub_shape2),
            ],
        }
    }
}

/// The user data of the material of `body`'s sub-shape `sub_shape`.
///
/// # Safety
/// `body` is a live body of a contact callback and `sub_shape` the manifold's id for it.
unsafe fn body_material(body: *const JPH_Body, sub_shape: JPH_SubShapeID) -> Option<u64> {
    // SAFETY: `body` is live (contract). joltc's `JPH_Body_GetShape` takes a mutable pointer
    // but only calls Jolt's const `Body::GetShape`, so nothing is written through the cast.
    // The shape is the body's, which Jolt does not change during a step, and `sub_shape` is
    // the id Jolt reported for it.
    unsafe {
        let shape = JPH_Body_GetShape(body.cast_mut());
        crate::material::shape_material(shape, SubShapeId::new(sub_shape))
    }
}

/// Which of the two callbacks with a manifold ran.
#[derive(Clone, Copy)]
enum Kind {
    Added,
    Persisted,
}

/// Records an added or persisted contact when the settings ask for it, and lets the user
/// listener change its settings.
///
/// # Safety
/// The arguments are the live ones of a joltc contact callback.
unsafe fn on_manifold(
    context: &ListenerContext,
    kind: Kind,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    native_manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    let wanted = match kind {
        Kind::Added => context.settings.contacts,
        Kind::Persisted => context.settings.persisted_contacts,
    };
    if !wanted && context.listener.is_none() {
        return;
    }
    // SAFETY: the arguments are live for the callback (contract); `settings` is joltc's local
    // copy, which joltc copies back to Jolt after the callback.
    let (manifold, facts, settings) = unsafe {
        let manifold = read_manifold(context.world, body1, body2, native_manifold);
        let facts = ContactFacts {
            sensor_body: JPH_Body_IsSensor(body1) || JPH_Body_IsSensor(body2),
            lever_arm: lever_arm(body1, &manifold),
        };
        (manifold, facts, &mut *settings)
    };
    let mut contact_settings = ContactSettings::new(settings, facts);
    if let Some(listener) = &context.listener {
        let mut changed = contact_settings;
        let returned = context.call_listener(|| match kind {
            Kind::Added => listener.contact_added(&manifold, &mut changed),
            Kind::Persisted => listener.contact_persisted(&manifold, &mut changed),
        });
        if returned.is_some() && changed != contact_settings {
            match changed.checked_for(facts) {
                Ok(accepted) => {
                    accepted.write_to(settings);
                    contact_settings = accepted;
                }
                Err(error) => context.reject(manifold.pair, error),
            }
        }
    }
    if wanted {
        let event = match kind {
            Kind::Added => {
                let estimate = if context.settings.collision_estimates {
                    // SAFETY: the arguments are the live ones of joltc's `OnContactAdded`
                    // (contract), and `contact_settings` are the ones Jolt resolves it with.
                    unsafe {
                        estimate(
                            body1,
                            body2,
                            native_manifold,
                            &contact_settings,
                            context.solver,
                        )
                    }
                } else {
                    None
                };
                ContactEvent::Added {
                    manifold,
                    settings: contact_settings,
                    estimate,
                }
            }
            Kind::Persisted => ContactEvent::Persisted {
                manifold,
                settings: contact_settings,
            },
        };
        context.batch().contacts.push(event);
    }
}

/// The largest distance from body 1's centre of mass to a point of `manifold`, on either body,
/// in metres.
///
/// # Safety
/// `body1` is the live body 1 of a contact callback, which may be read.
unsafe fn lever_arm(body1: *const JPH_Body, manifold: &ContactManifold) -> f64 {
    let mut com = RVec3::ZERO.to_jph();
    // SAFETY: `body1` is live (contract); Jolt allows reading bodies in contact callbacks
    // (`ContactListener.h`), and `com` is a live local.
    unsafe { JPH_Body_GetCenterOfMassPosition(body1, &mut com) };
    let com = real_coordinates(RVec3::from_jph(com));
    manifold
        .points
        .iter()
        .flat_map(|point| [point.on1, point.on2])
        .map(|point| {
            let [x, y, z] = real_coordinates(point);
            let [dx, dy, dz] = [x - com[0], y - com[1], z - com[2]];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .fold(0.0, f64::max)
}

/// The coordinates of `p` in `f64`.
#[allow(clippy::unnecessary_cast)] // `Real` is `f32` without the `double-precision` feature.
fn real_coordinates(p: RVec3) -> [f64; 3] {
    [p.x as f64, p.y as f64, p.z as f64]
}

/// joltc's `OnContactAdded`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of `Listeners` and live arguments.
pub(super) unsafe extern "C" fn on_contact_added(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    // SAFETY: joltc passes the listener's `userData` and live arguments (contract).
    unsafe {
        let context = ListenerContext::from_user_data(user_data);
        context.guarded((), Callback::ContactAdded, || {
            on_manifold(context, Kind::Added, body1, body2, manifold, settings)
        });
    }
}

/// joltc's `OnContactPersisted`.
///
/// # Safety
/// As for [`on_contact_added`].
pub(super) unsafe extern "C" fn on_contact_persisted(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    // SAFETY: as for `on_contact_added`.
    unsafe {
        let context = ListenerContext::from_user_data(user_data);
        context.guarded((), Callback::ContactPersisted, || {
            on_manifold(context, Kind::Persisted, body1, body2, manifold, settings)
        });
    }
}

/// joltc's `OnContactRemoved`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of `Listeners` and the live pair.
pub(super) unsafe extern "C" fn on_contact_removed(
    user_data: *mut c_void,
    pair: *const JPH_SubShapeIDPair,
) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    context.guarded((), Callback::ContactRemoved, || {
        if !context.settings.contacts {
            return;
        }
        let world = context.world;
        // SAFETY: `pair` is joltc's cast of the live Jolt pair (contract); the extension's
        // getters read it through Jolt's accessors.
        let pair = unsafe {
            SubShapeIdPair {
                body1: BodyId::new(JPH_SubShapeIDPair_GetBody1ID(pair), world),
                sub_shape1: SubShapeId::new(JPH_SubShapeIDPair_GetSubShapeID1(pair)),
                body2: BodyId::new(JPH_SubShapeIDPair_GetBody2ID(pair), world),
                sub_shape2: SubShapeId::new(JPH_SubShapeIDPair_GetSubShapeID2(pair)),
            }
        };
        context.batch().contacts.push(ContactEvent::Removed(pair));
    });
}
