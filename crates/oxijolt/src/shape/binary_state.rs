//! Shape binary state ("cooked" shapes): a built shape with its children and materials saved to
//! bytes and restored, so large meshes are built once at asset-build time.
//!
//! The bytes are a 48-byte header followed by the joltc extension's payload:
//!
//! | Offset | Size | Field |
//! |---|---|---|
//! | 0 | 8 | magic `OXJSHAPE` |
//! | 8 | 4 | [`BinaryStateError::FORMAT_VERSION`], little-endian |
//! | 12 | 4 | flags: bit 0 double precision, bit 1 cross-platform determinism, bit 2 little-endian |
//! | 16 | 16 | build id: XXH64 with seeds 0 and 1 of the Jolt and joltc commits and the extension revision |
//! | 32 | 8 | payload length, little-endian |
//! | 40 | 8 | XXH64 (seed 0) of the header with this field zeroed, then of the payload |

use std::collections::BTreeMap;
use std::mem::size_of;

use oxijolt_sys::*;

use super::compound::{check_expanded, fits_jolt_ids, sub_shape_ids};
use super::{initialize, Shape};
use crate::error::{BinaryStateError, JoltMessage};
use crate::ShapeError;

mod hash;

use hash::{xxh64, Xxh64};

const MAGIC: [u8; 8] = *b"OXJSHAPE";
/// Bytes of the header before the payload.
const HEADER_LEN: usize = 48;
const CHECKSUM_AT: usize = 40;
const FLAG_DOUBLE_PRECISION: u32 = 1;
const FLAG_CROSS_PLATFORM_DETERMINISTIC: u32 = 1 << 1;
const FLAG_LITTLE_ENDIAN: u32 = 1 << 2;
/// Bits in a Jolt `SubShapeID`.
const SUB_SHAPE_ID_BITS: u32 = 32;

impl Shape {
    /// Saves the shape with its children and materials to bytes that
    /// [`restore_binary_state`](Self::restore_binary_state) turns back into an equal shape (Jolt's
    /// cooked shape data, `Shape::SaveBinaryState` per shape). Build large meshes once, at
    /// asset-build time, save them, and restore them when a level loads: restoring copies Jolt's
    /// built data instead of building it again.
    ///
    /// Every shape kind this crate builds is supported. A child or material shared by several
    /// parents is saved once and shared again after restoring; a mesh keeps its
    /// [`MeshSettings::max_convex_extent`](crate::MeshSettings::max_convex_extent), compound
    /// children their user data, and [`PhysicsMaterial`](crate::PhysicsMaterial)s their user data.
    /// Two saves of equal shapes give equal bytes. The bytes start with a header that names this
    /// build (format version, Jolt and joltc commits, precision, determinism mode, byte order)
    /// and a checksum; the layout is in `docs/shape-cooking.md`.
    ///
    /// # Errors
    /// [`ShapeError::BinaryState`] with [`BinaryStateError::Rejected`] when joltc refuses a shape
    /// or material kind it cannot save; shapes and materials made by this crate are all
    /// supported.
    ///
    /// # Example
    /// ```
    /// use oxijolt::prelude::math::Vec3;
    /// use oxijolt::Shape;
    ///
    /// # fn main() -> Result<(), oxijolt::ShapeError> {
    /// let crate_box = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    /// let bytes = crate_box.save_binary_state()?;
    /// // SAFETY: the bytes were saved just above by this build and not changed.
    /// let restored = unsafe { Shape::restore_binary_state(&bytes) }?;
    /// assert_eq!(restored.save_binary_state()?, bytes);
    /// # Ok(())
    /// # }
    /// ```
    pub fn save_binary_state(&self) -> Result<Vec<u8>, ShapeError> {
        let payload = self.save_payload()?;
        let mut bytes = Vec::with_capacity(HEADER_LEN + payload.len());
        bytes.extend_from_slice(&header(&payload));
        bytes.extend_from_slice(&payload);
        Ok(bytes)
    }

    /// Restores a shape from bytes written by [`save_binary_state`](Self::save_binary_state).
    ///
    /// The header must name this build, and the checksum must match, before anything reaches
    /// Jolt. joltc then checks each record's envelope (type, lengths, child and material
    /// indices) before Jolt reads the record, and the record's child and material counts after
    /// Jolt read it, before they are attached. After restoring, the shape's local bounds must
    /// lie within [`limits::MAX_SHAPE_EXTENT`](crate::limits::MAX_SHAPE_EXTENT) and a compound
    /// must fit Jolt's sub-shape ids and
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`](crate::limits::MAX_EXPANDED_SUB_SHAPES), the rules
    /// [`Shape::new_compound`] applies.
    ///
    /// # Safety
    /// `bytes` are the unchanged output of [`save_binary_state`](Self::save_binary_state) of a
    /// build with the same build id, in any process; they may have been stored or sent on the
    /// way. The header, the checksum and joltc's record checks refuse bytes of another build,
    /// truncated bytes and most damage with an error, but passing them does not make other bytes
    /// valid: Jolt does not validate the inside of its own records (array lengths, mesh tree
    /// offsets, hull indices), so changed bytes that pass, whether damaged or crafted, can make
    /// Jolt read and write out of bounds. Do not restore bytes from a source you do not trust,
    /// such as another player.
    ///
    /// # Errors
    /// [`ShapeError::BinaryState`] with:
    /// - [`BinaryStateError::NotBinaryState`] when the magic does not match;
    /// - [`BinaryStateError::OtherBuild`] when the format version, flags or build id differ;
    /// - [`BinaryStateError::Truncated`] when the bytes end before the header or the payload;
    /// - [`BinaryStateError::Corrupt`] when the checksum does not match;
    /// - [`BinaryStateError::Malformed`] for bytes after the payload or a restored shape that
    ///   breaks a rule above;
    /// - [`BinaryStateError::Rejected`] when joltc refuses the record structure.
    ///
    /// [`ShapeError::InitFailed`] when Jolt could not be initialised.
    pub unsafe fn restore_binary_state(bytes: &[u8]) -> Result<Shape, ShapeError> {
        let payload = checked_payload(bytes).map_err(ShapeError::BinaryState)?;
        initialize()?;
        let mut message = [0u8; JoltMessage::CAPACITY + 2];
        // SAFETY: Jolt is initialised; `payload` is live for the call and holds `payload.len()`
        // bytes, which joltc only reads; `message` is a live buffer of the capacity passed. The
        // caller guarantees that the bytes are the unchanged output of a save of this build (the
        // header checked above, which names the build, is part of them), so every record inside
        // is one Jolt wrote. A returned shape holds one reference, which `from_raw` takes over.
        let shape = unsafe {
            JPH_Shape_RestoreBinaryState(
                payload.as_ptr().cast(),
                payload.len(),
                message.as_mut_ptr().cast(),
                message.len() as u32,
            )
        };
        if shape.is_null() {
            return Err(rejected(&message));
        }
        // SAFETY: the shape is live and holds one reference that this call hands over.
        let shape = unsafe { Shape::from_raw(shape) }?;
        check_restored(shape)
    }

    /// The joltc payload of this shape.
    fn save_payload(&self) -> Result<Vec<u8>, ShapeError> {
        let mut message = [0u8; JoltMessage::CAPACITY + 2];
        // SAFETY: the shape is live for the call and only read; `message` is a live buffer of
        // the capacity passed. The returned state is owned here and destroyed below.
        let state = unsafe {
            JPH_Shape_SaveBinaryState(
                self.as_ptr(),
                message.as_mut_ptr().cast(),
                message.len() as u32,
            )
        };
        if state.is_null() {
            return Err(rejected(&message));
        }
        // SAFETY: `state` is live until the destroy below.
        let size = unsafe { JPH_ShapeBinaryState_GetSize(state) };
        let mut payload = vec![0u8; size];
        // SAFETY: `state` is live and `payload` holds `size` writable bytes; joltc copies exactly
        // that many. Every byte is written by this crate's extension or by Jolt's
        // `SaveBinaryState`, which writes no indeterminate byte for the supported shape kinds
        // (docs/limits.md, "Shape binary state"). `state` is destroyed once, here.
        unsafe {
            JPH_ShapeBinaryState_CopyData(state, payload.as_mut_ptr().cast(), size);
            JPH_ShapeBinaryState_Destroy(state);
        }
        Ok(payload)
    }
}

fn rejected(message: &[u8]) -> ShapeError {
    ShapeError::BinaryState(BinaryStateError::Rejected(JoltMessage::from_c_buffer(
        message,
    )))
}

/// The flags of this build: precision, determinism mode and byte order.
fn build_flags() -> u32 {
    let mut flags = 0;
    if size_of::<Real>() == size_of::<f64>() {
        flags |= FLAG_DOUBLE_PRECISION;
    }
    if CROSS_PLATFORM_DETERMINISTIC_ENABLED {
        flags |= FLAG_CROSS_PLATFORM_DETERMINISTIC;
    }
    if cfg!(target_endian = "little") {
        flags |= FLAG_LITTLE_ENDIAN;
    }
    flags
}

/// The build id: the native library's Jolt and joltc commits and extension revision, hashed.
fn build_id() -> [u8; 16] {
    let text = format!("jolt={JOLT_COMMIT};joltc={JOLTC_COMMIT};joltc_ext={JOLTC_EXT_REVISION}");
    let mut id = [0; 16];
    id[..8].copy_from_slice(&xxh64(text.as_bytes(), 0).to_le_bytes());
    id[8..].copy_from_slice(&xxh64(text.as_bytes(), 1).to_le_bytes());
    id
}

/// The header of `payload` for this build.
fn header(payload: &[u8]) -> [u8; HEADER_LEN] {
    let mut header = [0; HEADER_LEN];
    header[..8].copy_from_slice(&MAGIC);
    header[8..12].copy_from_slice(&BinaryStateError::FORMAT_VERSION.to_le_bytes());
    header[12..16].copy_from_slice(&build_flags().to_le_bytes());
    header[16..32].copy_from_slice(&build_id());
    header[32..40].copy_from_slice(&(payload.len() as u64).to_le_bytes());
    let checksum = checksum(&header, payload);
    header[CHECKSUM_AT..].copy_from_slice(&checksum.to_le_bytes());
    header
}

/// XXH64 of `header` with its checksum field zeroed, then of `payload`.
fn checksum(header: &[u8; HEADER_LEN], payload: &[u8]) -> u64 {
    let mut zeroed = *header;
    zeroed[CHECKSUM_AT..].fill(0);
    let mut hasher = Xxh64::new(0);
    hasher.update(&zeroed);
    hasher.update(payload);
    hasher.finish()
}

/// The payload of `bytes` once the header names this build and the checksum matches.
fn checked_payload(bytes: &[u8]) -> Result<&[u8], BinaryStateError> {
    let (header, payload) = match bytes.split_first_chunk::<HEADER_LEN>() {
        Some(split) => split,
        None if bytes.len() >= MAGIC.len() && bytes[..MAGIC.len()] != MAGIC => {
            return Err(BinaryStateError::NotBinaryState)
        }
        None => return Err(BinaryStateError::Truncated),
    };
    if header[..8] != MAGIC {
        return Err(BinaryStateError::NotBinaryState);
    }
    let field = |range: std::ops::Range<usize>| &header[range];
    if field(8..12) != BinaryStateError::FORMAT_VERSION.to_le_bytes()
        || field(12..16) != build_flags().to_le_bytes()
        || field(16..32) != build_id()
    {
        return Err(BinaryStateError::OtherBuild);
    }
    let mut length = [0; 8];
    length.copy_from_slice(field(32..40));
    let length = u64::from_le_bytes(length);
    if length > payload.len() as u64 {
        return Err(BinaryStateError::Truncated);
    }
    if length < payload.len() as u64 {
        return Err(BinaryStateError::Malformed("bytes follow the payload"));
    }
    if field(CHECKSUM_AT..HEADER_LEN) != checksum(header, payload).to_le_bytes() {
        return Err(BinaryStateError::Corrupt);
    }
    Ok(payload)
}

/// `shape` when it follows the rules every shape this crate builds follows: local bounds within
/// [`limits::MAX_SHAPE_EXTENT`](crate::limits::MAX_SHAPE_EXTENT), sub-shape ids Jolt can form and
/// at most [`limits::MAX_EXPANDED_SUB_SHAPES`](crate::limits::MAX_EXPANDED_SUB_SHAPES) shapes.
fn check_restored(shape: Shape) -> Result<Shape, ShapeError> {
    let malformed = |rule| ShapeError::BinaryState(BinaryStateError::Malformed(rule));
    let shape = shape
        .within_extent_bounds()
        .map_err(|_| malformed("the shape's bounds exceed limits::MAX_SHAPE_EXTENT"))?;
    // SAFETY: `shape` keeps its whole graph alive for the call.
    let ids = unsafe { sub_shape_ids(shape.as_ptr(), &mut BTreeMap::new()) };
    if ids.width > SUB_SHAPE_ID_BITS || !fits_jolt_ids(ids) {
        return Err(malformed("the shape's sub-shape ids do not fit 32 bits"));
    }
    check_expanded(ids.expanded)
        .map_err(|_| malformed("the shape expands beyond limits::MAX_EXPANDED_SUB_SHAPES"))?;
    Ok(shape)
}

#[cfg(test)]
mod tests;
