//! Body activation events: what joltc's body activation listener reports.

use std::ffi::c_void;

use oxijolt_sys::*;

use super::{Callback, ListenerContext};
use crate::BodyId;

/// A body woke up or fell asleep.
///
/// Jolt reports a body as activated when it is created or added awake and when it is woken
/// (a velocity or position change, a collision with an awake body), and as deactivated when it
/// falls asleep and when an awake body is removed from the world, so a Deactivated id can name
/// a body that no longer exists. [`restore_state`](crate::PhysicsWorld::restore_state) changes
/// which bodies are awake without reporting it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ActivationEvent {
    /// The body is awake.
    Activated(BodyId),
    /// The body is asleep or was removed while awake.
    Deactivated(BodyId),
}

impl ActivationEvent {
    /// The body that changed.
    pub fn body(&self) -> BodyId {
        match *self {
            Self::Activated(body) | Self::Deactivated(body) => body,
        }
    }
}

/// Records an activation change when the settings ask for it. Jolt calls this under its
/// active-bodies mutex, so it records the supplied id only.
fn record(context: &ListenerContext, raw: JPH_BodyID, activated: bool) {
    if !context.settings.body_activation {
        return;
    }
    let body = BodyId::new(raw, context.world);
    let event = if activated {
        ActivationEvent::Activated(body)
    } else {
        ActivationEvent::Deactivated(body)
    };
    context.batch().activations.push(event);
}

/// joltc's `OnBodyActivated`.
///
/// # Safety
/// Called by joltc with the `userData` of a listener of `Listeners`.
pub(super) unsafe extern "C" fn on_body_activated(user_data: *mut c_void, id: JPH_BodyID, _: u64) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    context.guarded((), Callback::BodyActivated, || record(context, id, true));
}

/// joltc's `OnBodyDeactivated`.
///
/// # Safety
/// As for [`on_body_activated`].
pub(super) unsafe extern "C" fn on_body_deactivated(
    user_data: *mut c_void,
    id: JPH_BodyID,
    _: u64,
) {
    // SAFETY: joltc passes the listener's `userData` (contract).
    let context = unsafe { ListenerContext::from_user_data(user_data) };
    context.guarded((), Callback::BodyDeactivated, || record(context, id, false));
}
