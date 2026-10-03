//! Soft body contact events: what the extension's soft body contact listener reports.

use std::ffi::c_void;

use joltphysics_sys::*;

use super::{Callback, ListenerContext};
use crate::{BodyId, Quat, RVec3, Real, Vec3};

/// How Jolt resolves the contacts between a soft body and another body (Jolt
/// `SoftBodyContactSettings`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyContactSettings {
    inv_mass_scale1: f32,
    inv_mass_scale2: f32,
    inv_inertia_scale2: f32,
    is_sensor: bool,
}

impl SoftBodyContactSettings {
    pub(super) fn from_jph(settings: &JPH_SoftBodyContactSettings) -> Self {
        Self {
            inv_mass_scale1: settings.invMassScale1,
            inv_mass_scale2: settings.invMassScale2,
            inv_inertia_scale2: settings.invInertiaScale2,
            is_sensor: settings.isSensor,
        }
    }

    /// Factor on the soft body's vertex inverse masses for these contacts; 1 by default.
    pub fn inv_mass_scale1(&self) -> f32 {
        self.inv_mass_scale1
    }

    /// Factor on the other body's inverse mass; 1 by default.
    pub fn inv_mass_scale2(&self) -> f32 {
        self.inv_mass_scale2
    }

    /// Factor on the other body's inverse inertia; 1 by default.
    pub fn inv_inertia_scale2(&self) -> f32 {
        self.inv_inertia_scale2
    }

    /// Whether the other body only reports and does not push the vertices; by default when it
    /// is a sensor.
    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }
}

/// Whether Jolt processes the contacts between a soft body and another body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SoftBodyValidateResult {
    /// The vertices collide with the other body.
    AcceptContact,
    /// The vertices pass through the other body for this step.
    RejectContact,
}

/// A soft body's bounding box overlapped another body's in a step (Jolt
/// `OnSoftBodyContactValidate`). It says nothing about whether a vertex touched the body.
/// Jolt may report one pair twice in a step: once in the soft body's own collision pass and
/// once while the other body moves with continuous collision detection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyValidation {
    /// The soft body.
    pub soft_body: BodyId,
    /// The other body, a rigid body: soft bodies do not collide with each other.
    pub other: BodyId,
    /// The settings Jolt resolves the contacts with.
    pub settings: SoftBodyContactSettings,
    /// Whether Jolt processes the contacts.
    pub result: SoftBodyValidateResult,
}

/// One vertex of a soft body touching another body in a step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftBodyVertexContact {
    /// Index of the vertex in the soft body's shared settings.
    pub vertex: u32,
    /// The body the vertex touches.
    pub body: BodyId,
    /// The contact point on the other body's surface, in world space (metres).
    pub position: RVec3,
    /// Unit normal in world space pointing from the vertex into the other body: Jolt's contact
    /// normal, minus the outward surface normal of the other body.
    pub normal: Vec3,
}

/// The contacts of one soft body in one step (Jolt `OnSoftBodyContactAdded`).
///
/// Jolt clears a soft body's vertex contacts at the end of each step, so these events are the
/// only way to see them; [`were_bodies_in_contact`](crate::PhysicsWorld::were_bodies_in_contact)
/// is false for soft bodies.
#[derive(Clone, Debug, PartialEq)]
pub struct SoftBodyContacts {
    /// The soft body.
    pub soft_body: BodyId,
    /// The vertices that touch a body, in vertex order.
    pub vertices: Vec<SoftBodyVertexContact>,
    /// The sensors whose shapes the soft body overlaps, by body id.
    pub sensors: Vec<BodyId>,
}

/// The extension's `OnSoftBodyContactValidate`. Jolt may call it while another thread moves
/// the soft body (continuous collision detection), so it reads the two ids and the settings
/// only.
///
/// # Safety
/// Called by the extension with the `userData` of a listener of `Listeners` and live
/// arguments.
pub(super) unsafe extern "C" fn on_soft_body_contact_validate(
    user_data: *mut c_void,
    soft_body: *const JPH_Body,
    other_body: *const JPH_Body,
    settings: *mut JPH_SoftBodyContactSettings,
) -> JPH_SoftBodyValidateResult {
    // SAFETY: the extension passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    let accept = JPH_SoftBodyValidateResult_AcceptContact;
    context.guarded(accept, Callback::SoftBodyValidate, || {
        if !context.settings.soft_body_validations {
            return accept;
        }
        // SAFETY: the bodies and settings are live for the callback (contract); the id getter
        // reads a member Jolt does not change during a step, and the settings are the
        // extension's local copy.
        let validation = unsafe {
            SoftBodyValidation {
                soft_body: BodyId::new(JPH_Body_GetID(soft_body), context.world),
                other: BodyId::new(JPH_Body_GetID(other_body), context.world),
                settings: SoftBodyContactSettings::from_jph(&*settings),
                result: SoftBodyValidateResult::AcceptContact,
            }
        };
        context.batch().soft_body_validations.push(validation);
        accept
    })
}

/// The extension's `OnSoftBodyContactAdded`. Jolt updates the soft body only on the calling
/// thread here, so its pose may be read.
///
/// # Safety
/// As for [`on_soft_body_contact_validate`]; `manifold` is live for the call.
pub(super) unsafe extern "C" fn on_soft_body_contact_added(
    user_data: *mut c_void,
    soft_body: *const JPH_Body,
    manifold: *const JPH_SoftBodyManifold,
) {
    // SAFETY: the extension passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    context.guarded((), Callback::SoftBodyAdded, || {
        if !context.settings.soft_body_contacts {
            return;
        }
        // SAFETY: the soft body and manifold are live for the callback (contract).
        let contacts = unsafe { read_contacts(context, soft_body, manifold) };
        context.batch().soft_body_contacts.push(contacts);
    });
}

/// Copies the contacts of a soft body manifold into world space.
///
/// The manifold's points and normals are in the soft body's centre-of-mass frame relative to
/// the pose of this step, which Jolt changes after the callback, so they are converted here.
///
/// # Safety
/// The soft body and manifold are the live ones of `OnSoftBodyContactAdded`.
unsafe fn read_contacts(
    context: &ListenerContext,
    soft_body: *const JPH_Body,
    manifold: *const JPH_SoftBodyManifold,
) -> SoftBodyContacts {
    let world = context.world;
    let mut com = RVec3::ZERO.to_jph();
    let mut rotation = Quat::IDENTITY.to_jph();
    // SAFETY: the soft body and manifold are live (contract) and only read; every index passed
    // to the manifold getters is below its counts, and the outputs are live locals.
    unsafe {
        JPH_Body_GetCenterOfMassPosition(soft_body, &mut com);
        JPH_Body_GetRotation(soft_body, &mut rotation);
        let (com, rotation) = (RVec3::from_jph(com), Quat::from_jph(rotation));
        let mut vertices = Vec::new();
        for vertex in 0..JPH_SoftBodyManifold_GetVertexCount(manifold) {
            let mut point = Vec3::ZERO.to_jph();
            let mut normal = Vec3::ZERO.to_jph();
            if JPH_SoftBodyManifold_GetLocalContactPoint(manifold, vertex, &mut point)
                && JPH_SoftBodyManifold_GetContactNormal(manifold, vertex, &mut normal)
            {
                let offset = rotation.rotate(Vec3::from_jph(point));
                vertices.push(SoftBodyVertexContact {
                    vertex,
                    body: BodyId::new(
                        JPH_SoftBodyManifold_GetContactBodyID(manifold, vertex),
                        world,
                    ),
                    position: RVec3::new(
                        com.x + Real::from(offset.x),
                        com.y + Real::from(offset.y),
                        com.z + Real::from(offset.z),
                    ),
                    normal: rotation.rotate(Vec3::from_jph(normal)),
                });
            }
        }
        let mut sensors: Vec<BodyId> = (0..JPH_SoftBodyManifold_GetNumSensorContacts(manifold))
            .map(|index| {
                BodyId::new(
                    JPH_SoftBodyManifold_GetSensorContactBodyID(manifold, index),
                    world,
                )
            })
            .collect();
        // Jolt reorders its sensor list by swap-remove; sorting by id makes the order canonical.
        sensors.sort_unstable_by_key(|body| body.to_raw());
        SoftBodyContacts {
            soft_body: BodyId::new(JPH_Body_GetID(soft_body), world),
            vertices,
            sensors,
        }
    }
}
