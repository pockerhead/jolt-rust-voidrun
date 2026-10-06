//! A 64-bit FNV-1a digest of typed values, to compare scene runs bit for bit.

use oxijolt::{Quat, RVec3, Vec3};

use crate::draw::DrawList;

/// FNV-1a over the little-endian bytes of the values folded in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Digest(u64);

impl Default for Digest {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Digest {
    /// Folds raw bytes.
    pub fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    /// Folds a flag.
    pub fn bool(&mut self, value: bool) {
        self.bytes(&[u8::from(value)]);
    }

    /// Folds an integer.
    pub fn u32(&mut self, value: u32) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds an integer.
    pub fn i32(&mut self, value: i32) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds an integer.
    pub fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds the bits of a float.
    pub fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }

    /// Folds the bits of each component.
    pub fn f32s(&mut self, values: &[f32]) {
        for &value in values {
            self.f32(value);
        }
    }

    /// Folds the bits of each component.
    pub fn vec3(&mut self, value: Vec3) {
        self.f32s(&<[f32; 3]>::from(value));
    }

    /// Folds the bits of each component.
    pub fn quat(&mut self, value: Quat) {
        self.f32s(&<[f32; 4]>::from(value));
    }

    /// Folds the bits of each component, in the precision of positions.
    pub fn rvec3(&mut self, value: RVec3) {
        for component in <[oxijolt::Real; 3]>::from(value) {
            self.bytes(&component.to_le_bytes());
        }
    }

    /// The digest of everything folded so far.
    pub fn finish(&self) -> u64 {
        self.0
    }

    /// Folds what a draw list shows: every solid in order (description id and version, pose,
    /// colour) and every surface vertex. Lines and text are left out, so the wireframe and the
    /// HUD do not change a run's digest.
    pub fn draw_list(&mut self, list: &DrawList) {
        self.u64(list.solids.len() as u64);
        for solid in &list.solids {
            self.u32(solid.visual.id);
            self.u32(solid.visual.version);
            self.f32s(&solid.position);
            self.f32s(&solid.rotation);
            self.f32s(&solid.colour);
        }
        self.u64(list.surfaces.len() as u64);
        for surface in &list.surfaces {
            for vertex in &surface.vertices {
                self.f32s(vertex);
            }
        }
    }
}
