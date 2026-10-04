//! `From` conversions between the math types and [mint](https://docs.rs/mint)'s (feature `mint`).
//!
//! [`Vec3`] converts to `mint::Vector3<f32>`, [`RVec3`] to `mint::Vector3<Real>` and [`Quat`] to
//! `mint::Quaternion<f32>`. Each conversion copies the fields, so a value converted there and back
//! has the same bits. It neither normalizes nor checks: the calls that take a value check it as
//! usual.

use crate::math::{Quat, RVec3, Real, Vec3};

impl From<mint::Vector3<f32>> for Vec3 {
    fn from(value: mint::Vector3<f32>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<Vec3> for mint::Vector3<f32> {
    fn from(value: Vec3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<mint::Vector3<Real>> for RVec3 {
    fn from(value: mint::Vector3<Real>) -> Self {
        Self::new(value.x, value.y, value.z)
    }
}

impl From<RVec3> for mint::Vector3<Real> {
    fn from(value: RVec3) -> Self {
        Self {
            x: value.x,
            y: value.y,
            z: value.z,
        }
    }
}

impl From<mint::Quaternion<f32>> for Quat {
    /// Takes `v` as the vector part and `s` as the scalar part `w`.
    fn from(value: mint::Quaternion<f32>) -> Self {
        Self::from_xyzw(value.v.x, value.v.y, value.v.z, value.s)
    }
}

impl From<Quat> for mint::Quaternion<f32> {
    /// Puts the vector part in `v` and the scalar part `w` in `s`.
    fn from(value: Quat) -> Self {
        Self {
            v: mint::Vector3 {
                x: value.x,
                y: value.y,
                z: value.z,
            },
            s: value.w,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `f32` values whose bits a lossy conversion would change: negative zero, a subnormal, the
    /// largest finite value and a quiet NaN with a payload.
    const AWKWARD: [f32; 4] = [
        -0.0,
        f32::MIN_POSITIVE / 2.0,
        f32::MAX,
        f32::from_bits(0x7fc0_1234),
    ];

    /// The same kinds of values in [`Real`].
    fn awkward_real() -> [Real; 4] {
        [
            -0.0,
            Real::MIN_POSITIVE / 2.0,
            Real::MAX,
            Real::from_bits(Real::NAN.to_bits() | 0x1234),
        ]
    }

    #[test]
    fn vec3_round_trips_through_mint_with_the_same_bits() {
        for value in AWKWARD {
            let v = Vec3::new(value, -1.5, value);
            let there = mint::Vector3::<f32>::from(v);
            let there_bits = [there.x, there.y, there.z].map(f32::to_bits);
            assert_eq!(there_bits, <[f32; 3]>::from(v).map(f32::to_bits));
            let back = Vec3::from(there);
            assert_eq!(
                <[f32; 3]>::from(back).map(f32::to_bits),
                <[f32; 3]>::from(v).map(f32::to_bits)
            );
        }
    }

    #[test]
    fn rvec3_round_trips_through_mint_with_the_same_bits() {
        for value in awkward_real() {
            let p = RVec3::new(value, 2.25, value);
            let there = mint::Vector3::<Real>::from(p);
            let there_bits = [there.x, there.y, there.z].map(Real::to_bits);
            assert_eq!(there_bits, <[Real; 3]>::from(p).map(Real::to_bits));
            let back = RVec3::from(there);
            assert_eq!(
                <[Real; 3]>::from(back).map(Real::to_bits),
                <[Real; 3]>::from(p).map(Real::to_bits)
            );
        }
    }

    #[test]
    fn quat_round_trips_through_mint_with_the_same_bits() {
        for value in AWKWARD {
            let q = Quat::from_xyzw(value, 0.5, value, -0.0);
            let there = mint::Quaternion::<f32>::from(q);
            let there_bits = [there.v.x, there.v.y, there.v.z, there.s].map(f32::to_bits);
            assert_eq!(there_bits, <[f32; 4]>::from(q).map(f32::to_bits));
            let back = Quat::from(there);
            assert_eq!(
                <[f32; 4]>::from(back).map(f32::to_bits),
                <[f32; 4]>::from(q).map(f32::to_bits)
            );
        }
    }

    #[test]
    fn quat_scalar_part_is_mints_s() {
        let there = mint::Quaternion::<f32>::from(Quat::from_xyzw(1.0, 2.0, 3.0, 4.0));
        assert_eq!(
            [there.v.x, there.v.y, there.v.z, there.s],
            [1.0, 2.0, 3.0, 4.0]
        );
    }
}
