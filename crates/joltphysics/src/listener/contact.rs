//! Rigid body contact events: what joltc's contact listener reports, copied into Rust values.

use std::ffi::c_void;

use joltphysics_sys::*;

use super::{Callback, ListenerContext};
use crate::world::WorldTag;
use crate::{BodyId, RVec3, SubShapeId, Vec3};

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
#[derive(Clone, Copy, Debug, PartialEq)]
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
}

impl ContactSettings {
    pub(super) fn from_jph(settings: &JPH_ContactSettings) -> Self {
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
        }
    }

    /// The friction of the contact, by default Jolt's `sqrt(friction1 * friction2)` of the two
    /// bodies.
    pub fn combined_friction(&self) -> f32 {
        self.combined_friction
    }

    /// The restitution of the contact, by default the larger of the two bodies'.
    pub fn combined_restitution(&self) -> f32 {
        self.combined_restitution
    }

    /// Factor on body 1's inverse mass for this contact; 1 by default.
    pub fn inv_mass_scale1(&self) -> f32 {
        self.inv_mass_scale1
    }

    /// Factor on body 1's inverse inertia for this contact; 1 by default.
    pub fn inv_inertia_scale1(&self) -> f32 {
        self.inv_inertia_scale1
    }

    /// Factor on body 2's inverse mass for this contact; 1 by default.
    pub fn inv_mass_scale2(&self) -> f32 {
        self.inv_mass_scale2
    }

    /// Factor on body 2's inverse inertia for this contact; 1 by default.
    pub fn inv_inertia_scale2(&self) -> f32 {
        self.inv_inertia_scale2
    }

    /// Whether the contact only reports and does not push the bodies apart; by default when
    /// either body is a sensor.
    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    /// Velocity of body 2's surface relative to body 1's at the contact, in m/s in world space
    /// (a conveyor belt); zero by default.
    pub fn relative_linear_surface_velocity(&self) -> Vec3 {
        self.relative_linear_surface_velocity
    }

    /// Angular velocity of body 2's surface relative to body 1's, in rad/s in world space;
    /// zero by default.
    pub fn relative_angular_surface_velocity(&self) -> Vec3 {
        self.relative_angular_surface_velocity
    }
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

/// Records an added or persisted contact when the settings ask for it.
///
/// # Safety
/// The arguments are the live ones of a joltc contact callback.
unsafe fn record_manifold(
    context: &ListenerContext,
    kind: Kind,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    manifold: *const JPH_ContactManifold,
    settings: *mut JPH_ContactSettings,
) {
    let wanted = match kind {
        Kind::Added => context.settings.contacts,
        Kind::Persisted => context.settings.persisted_contacts,
    };
    if !wanted {
        return;
    }
    // SAFETY: the arguments are live for the callback (contract); `settings` is joltc's local
    // copy, only read here.
    let (manifold, settings) = unsafe {
        (
            read_manifold(context.world, body1, body2, manifold),
            ContactSettings::from_jph(&*settings),
        )
    };
    let event = match kind {
        Kind::Added => ContactEvent::Added { manifold, settings },
        Kind::Persisted => ContactEvent::Persisted { manifold, settings },
    };
    context.batch().contacts.push(event);
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
            record_manifold(context, Kind::Added, body1, body2, manifold, settings)
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
            record_manifold(context, Kind::Persisted, body1, body2, manifold, settings)
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
