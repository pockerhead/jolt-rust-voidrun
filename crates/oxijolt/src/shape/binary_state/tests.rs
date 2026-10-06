use std::error::Error as _;

use super::*;
use crate::{CompoundChild, Quat, Vec3};

fn compound() -> Shape {
    let cube = Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    let child = |shape, x, user_data| CompoundChild {
        shape,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    };
    Shape::new_compound(&[
        child(&cube, 0.0, 1),
        child(&ball, 2.0, 2),
        child(&cube, 4.0, 3),
    ])
    .unwrap()
}

fn restore(bytes: &[u8]) -> Result<Shape, ShapeError> {
    // SAFETY: every test passes bytes this build saved, or such bytes changed so that the header
    // and checksum checks in Rust or joltc's envelope checks refuse them before Jolt reads the
    // changed record (each test asserts the refusal). The one changed record Jolt reads is a box
    // whose three half-extent floats were replaced: a box record holds no length, offset or
    // index, so Jolt reads just those floats.
    unsafe { Shape::restore_binary_state(bytes) }
}

fn error(bytes: &[u8]) -> BinaryStateError {
    match restore(bytes) {
        Err(ShapeError::BinaryState(error)) => error,
        Err(other) => panic!("unexpected error {other:?}"),
        Ok(_) => panic!("restored damaged bytes"),
    }
}

/// `bytes` with the header's checksum recomputed, as a writer of another build would.
fn resealed(mut bytes: Vec<u8>) -> Vec<u8> {
    let (header, payload) = bytes.split_at_mut(HEADER_LEN);
    let header: &mut [u8; HEADER_LEN] = header.try_into().unwrap();
    let sum = checksum(header, payload);
    header[CHECKSUM_AT..].copy_from_slice(&sum.to_le_bytes());
    bytes
}

/// A seeded generator of indices (SplitMix64), so the flips are the same on every run.
struct SplitMix(u64);

impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

#[test]
fn the_format_version_is_the_extension_s() {
    // SAFETY: a plain function without arguments.
    let native = unsafe { JPH_Shape_GetBinaryStateVersion() };
    assert_eq!(BinaryStateError::FORMAT_VERSION, native);
}

#[test]
fn the_header_names_this_build() {
    let bytes = compound().save_binary_state().unwrap();
    assert_eq!(&bytes[..8], b"OXJSHAPE");
    assert_eq!(bytes[8..12], 1u32.to_le_bytes());
    let flags = u32::from_le_bytes(bytes[12..16].try_into().unwrap());
    assert_eq!(flags & FLAG_LITTLE_ENDIAN, FLAG_LITTLE_ENDIAN);
    assert_eq!(
        flags & FLAG_DOUBLE_PRECISION != 0,
        size_of::<Real>() == size_of::<f64>()
    );
    assert_eq!(bytes[16..32], build_id());
    assert_eq!(
        u64::from_le_bytes(bytes[32..40].try_into().unwrap()),
        (bytes.len() - HEADER_LEN) as u64
    );
}

#[test]
fn bytes_of_another_build_are_refused() {
    let bytes = compound().save_binary_state().unwrap();
    for bit in [
        FLAG_DOUBLE_PRECISION,
        FLAG_CROSS_PLATFORM_DETERMINISTIC,
        FLAG_LITTLE_ENDIAN,
    ] {
        let mut other = bytes.clone();
        other[12..16].copy_from_slice(&(build_flags() ^ bit).to_le_bytes());
        assert_eq!(
            error(&resealed(other)),
            BinaryStateError::OtherBuild,
            "flag {bit}"
        );
    }
    let mut other_version = bytes.clone();
    other_version[8] ^= 2;
    assert_eq!(
        error(&resealed(other_version)),
        BinaryStateError::OtherBuild
    );
    let mut other_jolt = bytes.clone();
    other_jolt[20] ^= 1;
    assert_eq!(error(&resealed(other_jolt)), BinaryStateError::OtherBuild);
}

#[test]
fn every_header_bit_flip_is_refused() {
    let bytes = compound().save_binary_state().unwrap();
    for bit in 0..HEADER_LEN * 8 {
        let mut flipped = bytes.clone();
        flipped[bit / 8] ^= 1 << (bit % 8);
        assert!(restore(&flipped).is_err(), "header bit {bit}");
    }
}

#[test]
fn payload_bit_flips_are_refused_as_corrupt() {
    let bytes = compound().save_binary_state().unwrap();
    let payload_bits = (bytes.len() - HEADER_LEN) as u64 * 8;
    let mut random = SplitMix(43);
    for _ in 0..4096 {
        let bit = HEADER_LEN * 8 + (random.next() % payload_bits) as usize;
        let mut flipped = bytes.clone();
        flipped[bit / 8] ^= 1 << (bit % 8);
        assert_eq!(error(&flipped), BinaryStateError::Corrupt, "bit {bit}");
    }
}

#[test]
fn every_truncation_is_refused_as_truncated() {
    let bytes = compound().save_binary_state().unwrap();
    for length in 0..bytes.len() {
        assert_eq!(
            error(&bytes[..length]),
            BinaryStateError::Truncated,
            "length {length}"
        );
    }
}

#[test]
fn bytes_after_the_payload_are_refused() {
    let bytes = compound().save_binary_state().unwrap();
    let twice = [bytes.clone(), bytes.clone()].concat();
    assert_eq!(
        error(&twice),
        BinaryStateError::Malformed("bytes follow the payload")
    );
    let mut longer = bytes;
    longer.push(0);
    assert!(matches!(error(&longer), BinaryStateError::Malformed(_)));
}

#[test]
fn bytes_without_the_header_are_not_binary_state() {
    assert_eq!(error(&[]), BinaryStateError::Truncated);
    assert_eq!(error(&[0; HEADER_LEN]), BinaryStateError::NotBinaryState);
    assert_eq!(error(&[0; 12]), BinaryStateError::NotBinaryState);
    assert_eq!(error(b"OXJSHAPE"), BinaryStateError::Truncated);
}

#[test]
fn a_record_joltc_refuses_behind_a_valid_checksum_is_rejected() {
    let mut bytes = compound().save_binary_state().unwrap();
    bytes.push(0);
    let length = (bytes.len() - HEADER_LEN) as u64;
    bytes[32..40].copy_from_slice(&length.to_le_bytes());
    let error = error(&resealed(bytes));
    let BinaryStateError::Rejected(message) = error else {
        panic!("expected a joltc refusal, got {error:?}");
    };
    assert!(
        message.as_str().contains("bytes after the last record"),
        "{message}"
    );
}

#[test]
fn the_shape_error_reports_its_source() {
    let error = ShapeError::BinaryState(BinaryStateError::Corrupt);
    assert_eq!(
        error.to_string(),
        "invalid shape binary state: the checksum does not match"
    );
    let source = error
        .source()
        .expect("the binary state error is the source");
    assert_eq!(source.to_string(), "the checksum does not match");
    assert!(ShapeError::AllocationFailed.source().is_none());
}

#[test]
fn a_restored_shape_beyond_the_extent_is_malformed() {
    let half_extent = Vec3::new(0.123, 0.234, 0.345);
    let mut bytes = Shape::new_box(half_extent)
        .unwrap()
        .save_binary_state()
        .unwrap();
    // The box record holds the half extent as three floats: make them far too large, as a
    // writer that skipped the constructor's checks would.
    let mut replaced = 0;
    for value in [half_extent.x, half_extent.y, half_extent.z] {
        let (from, to) = (value.to_le_bytes(), 1.0e9f32.to_le_bytes());
        let at = (HEADER_LEN..bytes.len() - 3)
            .find(|&i| bytes[i..i + 4] == from)
            .unwrap();
        bytes[at..at + 4].copy_from_slice(&to);
        replaced += 1;
    }
    assert_eq!(replaced, 3);
    assert_eq!(
        error(&resealed(bytes)),
        BinaryStateError::Malformed("the shape's bounds exceed limits::MAX_SHAPE_EXTENT")
    );
}
