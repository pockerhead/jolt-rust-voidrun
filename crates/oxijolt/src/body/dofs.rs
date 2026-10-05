//! The degrees of freedom a body may move in (Jolt `EAllowedDOFs`).

use std::ops::BitOr;

use oxijolt_sys::*;

/// The degrees of freedom a body may move in: translation along and rotation about the world
/// axes (Jolt `EAllowedDOFs`), set at creation with
/// [`BodySettings::allowed_dofs`](crate::BodySettings::allowed_dofs).
///
/// Combine the constants with `|`. The axes are world axes, not the body's own: a body that may
/// rotate only about Z keeps rotating about world Z whatever its rotation. Jolt masks the
/// locked axes out of the body's velocities and inverse inertia (`MotionProperties`), so
/// forces, impulses and contacts do not move the body along them.
///
/// A value must keep a translation axis; this is a rule of this crate, Jolt itself accepts some
/// rotation-only sets. A body with fewer than six degrees of freedom cannot join a constraint,
/// carry a vehicle, be a ragdoll part or go through a rotated
/// [`rebase`](crate::PhysicsWorld::rebase) ([`BodyError::RestrictedDofs`]): their checks assume
/// an unmasked body.
///
/// ```
/// use oxijolt::AllowedDofs;
///
/// let side_view =
///     AllowedDofs::TRANSLATION_X | AllowedDofs::TRANSLATION_Y | AllowedDofs::ROTATION_Z;
/// assert_eq!(side_view, AllowedDofs::PLANE_2D);
/// assert!(AllowedDofs::ALL.contains(side_view));
/// ```
///
/// [`BodyError::RestrictedDofs`]: crate::BodyError::RestrictedDofs
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AllowedDofs(u8);

impl AllowedDofs {
    /// Every degree of freedom; the default.
    pub const ALL: Self = Self(0b11_1111);
    /// Translation along world X.
    pub const TRANSLATION_X: Self = Self(0b00_0001);
    /// Translation along world Y.
    pub const TRANSLATION_Y: Self = Self(0b00_0010);
    /// Translation along world Z.
    pub const TRANSLATION_Z: Self = Self(0b00_0100);
    /// Rotation about world X.
    pub const ROTATION_X: Self = Self(0b00_1000);
    /// Rotation about world Y.
    pub const ROTATION_Y: Self = Self(0b01_0000);
    /// Rotation about world Z.
    pub const ROTATION_Z: Self = Self(0b10_0000);
    /// Translation along world X and Y and rotation about world Z: motion in the XY plane.
    pub const PLANE_2D: Self = Self(0b10_0011);

    /// Whether every degree of freedom of `other` is also in `self`.
    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Whether at least one translation axis is free.
    pub(crate) fn has_translation(self) -> bool {
        self.0 & 0b111 != 0
    }

    pub(crate) fn to_jph(self) -> JPH_AllowedDOFs {
        JPH_AllowedDOFs::from(self.0)
    }

    /// The value Jolt reports; Jolt keeps the bits it was created with, which only the constants
    /// and `|` produce.
    pub(crate) fn from_jph(value: JPH_AllowedDOFs) -> Self {
        // Jolt stores the value as a `uint8` (`EAllowedDOFs`), so the conversion is exact.
        Self(u8::try_from(value).unwrap_or_else(|_| unreachable!("Jolt stores DOFs in 8 bits")))
    }
}

impl Default for AllowedDofs {
    /// [`AllowedDofs::ALL`].
    fn default() -> Self {
        Self::ALL
    }
}

impl BitOr for AllowedDofs {
    type Output = Self;

    fn bitor(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}
