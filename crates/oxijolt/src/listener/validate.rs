//! Rigid contact validation: the hit Jolt asks a [`ContactListener`](crate::ContactListener) to
//! accept, and the answer.

use std::ffi::c_void;

use oxijolt_sys::*;

use super::{Callback, ListenerContext};
use crate::{BodyId, RVec3, Real, SubShapeId, Vec3};

/// A hit Jolt asks a [`ContactListener`](crate::ContactListener) to accept (Jolt
/// `CollideShapeResult` in `OnContactValidate`), before it becomes a contact.
///
/// The bodies are ordered as Jolt collides them, which differs from the id order of
/// [`SubShapeIdPair`](crate::SubShapeIdPair): in the discrete collision stage `body1` has the
/// higher motion type (dynamic over kinematic over static) and, for equal motion types, the
/// lower id; in the continuous stage (a [`MotionQuality::LinearCast`](crate::MotionQuality)
/// body) `body1` is the body being cast.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct ContactCandidate {
    /// The body that is collided against `body2`; see the ordering above.
    pub body1: BodyId,
    /// The other body.
    pub body2: BodyId,
    /// [`BodySettings::user_data`](crate::BodySettings::user_data) of `body1` and `body2`.
    pub user_data: [u64; 2],
    /// The sub-shape of `body1` that is hit.
    pub sub_shape1: SubShapeId,
    /// The sub-shape of `body2` that is hit.
    pub sub_shape2: SubShapeId,
    /// The deepest point on `body1`'s surface, in world space (metres). In the continuous stage
    /// it is where `body1` is at the time of impact, computed as if `body2` did not move.
    pub point_on1: RVec3,
    /// The deepest point on `body2`'s surface, in world space (metres). In the continuous stage
    /// `body2` is taken at its pose at the start of the step; Jolt shifts both points by
    /// `body2`'s motion only after validation.
    pub point_on2: RVec3,
    /// The direction along which `body2` moves out of `body1`, in world space; only the direction
    /// is meaningful, not the length.
    pub penetration_axis: Vec3,
    /// How far the shapes overlap, in metres; negative for a speculative contact, which Jolt
    /// reports before the shapes touch.
    pub penetration_depth: f32,
}

/// What a [`ContactListener`](crate::ContactListener) answers for a [`ContactCandidate`] (Jolt
/// `ValidateResult`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ValidateResult {
    /// Accepts this hit and every later one of the pair in this collision pass, without asking
    /// again (Jolt's answer without a listener).
    AcceptAllContactsForThisBodyPair,
    /// Accepts this hit and asks again for the pair's next hit.
    AcceptContact,
    /// Rejects this hit and asks again for the pair's next hit.
    RejectContact,
    /// Rejects this hit and every later one of the pair in this collision pass.
    RejectAllContactsForThisBodyPair,
}

impl ValidateResult {
    fn to_jph(self) -> JPH_ValidateResult {
        match self {
            Self::AcceptAllContactsForThisBodyPair => {
                JPH_ValidateResult_AcceptAllContactsForThisBodyPair
            }
            Self::AcceptContact => JPH_ValidateResult_AcceptContact,
            Self::RejectContact => JPH_ValidateResult_RejectContact,
            Self::RejectAllContactsForThisBodyPair => {
                JPH_ValidateResult_RejectAllContactsForThisBodyPair
            }
        }
    }
}

/// `base + offset` in world space.
#[allow(clippy::unnecessary_cast)] // `Real` is `f32` without the `double-precision` feature.
fn offset_point(base: RVec3, offset: Vec3) -> RVec3 {
    RVec3::new(
        base.x + offset.x as Real,
        base.y + offset.y as Real,
        base.z + offset.z as Real,
    )
}

/// Reads the hit the extension's listener passes to `OnContactValidate`.
///
/// # Safety
/// The bodies, `base_offset` and `result` are the live, non-null arguments of that callback.
unsafe fn read_candidate(
    context: &ListenerContext,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    base_offset: *const JPH_RVec3,
    result: *const JPH_CollideShapeResult,
) -> ContactCandidate {
    // SAFETY: the arguments are live for the callback (contract), and Jolt holds both bodies
    // locked while it calls it. joltc's `JPH_Body_GetUserData` takes a mutable pointer but only
    // calls Jolt's const `Body::GetUserData`, so nothing is written through the cast.
    let (base, hit, ids, user_data) = unsafe {
        (
            RVec3::from_jph(*base_offset),
            &*result,
            [JPH_Body_GetID(body1), JPH_Body_GetID(body2)],
            [
                JPH_Body_GetUserData(body1.cast_mut()),
                JPH_Body_GetUserData(body2.cast_mut()),
            ],
        )
    };
    ContactCandidate {
        body1: BodyId::new(ids[0], context.world),
        body2: BodyId::new(ids[1], context.world),
        user_data,
        sub_shape1: SubShapeId::new(hit.subShapeID1),
        sub_shape2: SubShapeId::new(hit.subShapeID2),
        point_on1: offset_point(base, Vec3::from_jph(hit.contactPointOn1)),
        point_on2: offset_point(base, Vec3::from_jph(hit.contactPointOn2)),
        penetration_axis: Vec3::from_jph(hit.penetrationAxis),
        penetration_depth: hit.penetrationDepth,
    }
}

/// The extension's `OnContactValidate`. Without a user listener, or when it does not return,
/// every contact of the pair is accepted, as Jolt does without a listener.
///
/// # Safety
/// Called by the extension's listener with the `userData` of a listener of `Listeners` and live
/// arguments.
pub(super) unsafe extern "C" fn on_contact_validate(
    user_data: *mut c_void,
    body1: *const JPH_Body,
    body2: *const JPH_Body,
    base_offset: *const JPH_RVec3,
    result: *const JPH_CollideShapeResult,
) -> JPH_ValidateResult {
    const ACCEPT_ALL: ValidateResult = ValidateResult::AcceptAllContactsForThisBodyPair;
    // SAFETY: the extension passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    let answer = context.guarded(ACCEPT_ALL, Callback::ContactValidate, || {
        let Some(listener) = &context.listener else {
            return ACCEPT_ALL;
        };
        // SAFETY: the arguments are live for the callback (contract).
        let candidate = unsafe { read_candidate(context, body1, body2, base_offset, result) };
        context
            .call_listener(|| listener.contact_validate(&candidate))
            .unwrap_or(ACCEPT_ALL)
    });
    answer.to_jph()
}
