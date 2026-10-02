//! Plain value types for vectors and rotations.
//!
//! Jolt uses a right-handed coordinate system with Y up. Lengths are in metres, velocities in
//! metres per second and angular velocities in radians per second. Conversions to and from
//! Jolt copy the fields and never round, so values read back with the same bits.

use joltphysics_sys::{JPH_Quat, JPH_RVec3, JPH_Vec3};

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
pub use joltphysics_sys::Real;

/// Whether `value` is finite and positive.
pub(crate) fn is_finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

/// Whether `value` is finite and not negative.
pub(crate) fn is_finite_non_negative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

/// A 3D vector of `f32`, used for directions, velocities, forces and extents.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    /// X component.
    pub x: f32,
    /// Y component (up).
    pub y: f32,
    /// Z component.
    pub z: f32,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Creates a vector from its components.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub(crate) fn to_jph(self) -> JPH_Vec3 {
        JPH_Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    pub(crate) fn from_jph(value: JPH_Vec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub(crate) fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub(crate) fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub(crate) fn scale(self, factor: f32) -> Self {
        Self::new(self.x * factor, self.y * factor, self.z * factor)
    }

    /// The unit vector along `self`, or zero when `self` is too short to have a direction.
    pub(crate) fn normalized_or_zero(self) -> Self {
        let length_squared = self.dot(self);
        if length_squared <= 1.0e-12 {
            Self::ZERO
        } else {
            self.scale(1.0 / length_squared.sqrt())
        }
    }
}

impl From<[f32; 3]> for Vec3 {
    fn from([x, y, z]: [f32; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<Vec3> for [f32; 3] {
    fn from(value: Vec3) -> Self {
        [value.x, value.y, value.z]
    }
}

/// A world-space position. Its components are [`Real`]: `f64` with the `double-precision`
/// feature, `f32` otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RVec3 {
    /// X component.
    pub x: Real,
    /// Y component (up).
    pub y: Real,
    /// Z component.
    pub z: Real,
}

impl RVec3 {
    /// The origin.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    /// Creates a position from its components.
    pub const fn new(x: Real, y: Real, z: Real) -> Self {
        Self { x, y, z }
    }

    pub(crate) fn to_jph(self) -> JPH_RVec3 {
        JPH_RVec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    pub(crate) fn from_jph(value: JPH_RVec3) -> Self {
        Self::new(value.x, value.y, value.z)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

impl From<[Real; 3]> for RVec3 {
    fn from([x, y, z]: [Real; 3]) -> Self {
        Self::new(x, y, z)
    }
}

impl From<RVec3> for [Real; 3] {
    fn from(value: RVec3) -> Self {
        [value.x, value.y, value.z]
    }
}

/// A rotation quaternion `(x, y, z, w)` with `w` the scalar part. Jolt expects unit length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    /// X component of the vector part.
    pub x: f32,
    /// Y component of the vector part.
    pub y: f32,
    /// Z component of the vector part.
    pub z: f32,
    /// Scalar part.
    pub w: f32,
}

impl Quat {
    /// The rotation that does nothing.
    pub const IDENTITY: Self = Self::from_xyzw(0.0, 0.0, 0.0, 1.0);

    /// Creates a quaternion from its components; `w` is the scalar part.
    pub const fn from_xyzw(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    pub(crate) fn to_jph(self) -> JPH_Quat {
        JPH_Quat {
            x: self.x,
            y: self.y,
            z: self.z,
            w: self.w,
        }
    }

    pub(crate) fn from_jph(value: JPH_Quat) -> Self {
        Self::from_xyzw(value.x, value.y, value.z, value.w)
    }

    pub(crate) fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite() && self.w.is_finite()
    }

    /// Unit length within the tolerance of Jolt's `Quat::IsNormalized` (`|length² − 1| <= 1e-5`).
    pub(crate) fn is_normalized(&self) -> bool {
        let length_squared = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        (length_squared - 1.0).abs() <= 1.0e-5
    }

    /// Whether this is a finite unit quaternion, within Jolt's tolerance: what Jolt expects of
    /// every rotation.
    pub(crate) fn is_valid_rotation(&self) -> bool {
        self.is_finite() && self.is_normalized()
    }
}

impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl From<[f32; 4]> for Quat {
    /// Takes the components in `[x, y, z, w]` order.
    fn from([x, y, z, w]: [f32; 4]) -> Self {
        Self::from_xyzw(x, y, z, w)
    }
}

impl From<Quat> for [f32; 4] {
    /// Returns the components in `[x, y, z, w]` order.
    fn from(value: Quat) -> Self {
        [value.x, value.y, value.z, value.w]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Values whose bits a lossy conversion would change.
    const AWKWARD: [f32; 4] = [
        -0.0,
        f32::MIN_POSITIVE / 2.0,
        f32::MAX,
        f32::from_bits(0x7fc0_1234),
    ];

    fn bits3(v: [f32; 3]) -> [u32; 3] {
        v.map(f32::to_bits)
    }

    #[test]
    fn vec3_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let v = Vec3::new(value, -1.5, value);
            let back = Vec3::from_jph(v.to_jph());
            assert_eq!(bits3(back.into()), bits3(v.into()));
        }
    }

    #[test]
    fn rvec3_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let v = RVec3::new(Real::from(value), 2.25, Real::from(value));
            let back: [Real; 3] = RVec3::from_jph(v.to_jph()).into();
            let original: [Real; 3] = v.into();
            assert_eq!(back.map(Real::to_bits), original.map(Real::to_bits));
        }
    }

    #[test]
    fn quat_round_trips_through_jolt_with_the_same_bits() {
        for value in AWKWARD {
            let q = Quat::from_xyzw(value, 0.5, value, -0.0);
            let back: [f32; 4] = Quat::from_jph(q.to_jph()).into();
            let original: [f32; 4] = q.into();
            assert_eq!(back.map(f32::to_bits), original.map(f32::to_bits));
        }
    }

    #[test]
    fn quat_defaults_to_identity() {
        assert_eq!(Quat::default(), Quat::IDENTITY);
    }

    #[test]
    fn is_normalized_uses_jolts_tolerance() {
        assert!(Quat::IDENTITY.is_normalized());
        let half = std::f32::consts::FRAC_1_SQRT_2;
        assert!(Quat::from_xyzw(0.0, half, 0.0, half).is_normalized());
        assert!(Quat::from_xyzw(0.5, 0.5, 0.5, 0.5).is_normalized());
        assert!(!Quat::from_xyzw(0.0, 0.0, 0.0, 2.0).is_normalized());
    }

    #[test]
    fn valid_rotations_are_finite_unit_quaternions() {
        assert!(Quat::IDENTITY.is_valid_rotation());
        assert!(!Quat::from_xyzw(f32::NAN, 0.0, 0.0, 1.0).is_valid_rotation());
        assert!(!Quat::from_xyzw(0.0, 0.0, 0.0, 2.0).is_valid_rotation());
    }

    #[test]
    fn vector_helpers() {
        let v = Vec3::new(3.0, 0.0, 4.0);
        assert_eq!(v.dot(Vec3::new(1.0, 1.0, 1.0)), 7.0);
        assert_eq!(v.length(), 5.0);
        assert_eq!(v.scale(2.0), Vec3::new(6.0, 0.0, 8.0));
        assert_eq!(v.normalized_or_zero(), Vec3::new(0.6, 0.0, 0.8));
        assert_eq!(Vec3::ZERO.normalized_or_zero(), Vec3::ZERO);
        assert_eq!(Vec3::new(1e-7, 0.0, 0.0).normalized_or_zero(), Vec3::ZERO);
    }
}
