//! The one path from shape settings to a shape.

use oxijolt_sys::*;

use super::{Shape, ShapeSettings};
use crate::error::JoltMessage;
use crate::owned::Owned;
use crate::ShapeError;

impl ShapeSettings {
    /// Runs Jolt's `Create` on the settings and takes over the shape.
    ///
    /// Fails with [`ShapeError::Rejected`] carrying Jolt's message when Jolt refuses the
    /// settings. Jolt caches the result in the settings, so later setters have no effect.
    pub(crate) fn create(&self) -> Result<Shape, ShapeError> {
        // One byte more than a message keeps, and the NUL: a longer message shows as cut.
        let mut message = [0u8; JoltMessage::CAPACITY + 2];
        // SAFETY: the settings are live and owned by `self`; every joltc settings type derives
        // from `ShapeSettings` with single inheritance, so the generic handle reaches Jolt's
        // virtual `Create`. `message` is a live buffer of the capacity passed. A returned shape
        // holds one reference, which the guard takes over.
        let shape = unsafe {
            Owned::from_raw(JPH_ShapeSettings_CreateShapeWithError(
                self.as_ptr(),
                message.as_mut_ptr().cast(),
                message.len() as u32,
            ))
        };
        shape
            .map(Shape)
            .ok_or_else(|| ShapeError::Rejected(JoltMessage::from_c_buffer(&message)))
    }
}
