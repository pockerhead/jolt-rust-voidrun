//! The tapered family: tapered capsules near Jolt's sphere boundary and tapered cylinders with
//! tiny and huge radius ratios.

use super::*;

const TAPERED_CASES: usize = 1000;

/// Half height, top and bottom radius of a tapered capsule; every other case sits close to the
/// sphere boundary `|bottom - top| = 2 * half_height`.
fn capsule(rng: &mut Rng, index: usize) -> (f32, f32, f32) {
    let half_height = rng.log_range(1.0e-3, 100.0);
    let smaller = rng.log_range(1.0e-4, 100.0);
    let taper = if index.is_multiple_of(2) {
        2.0 * half_height * (1.0 - rng.log_range(1.0e-8, 1.0e-2))
    } else {
        2.0 * half_height * rng.unit()
    };
    let (top, bottom) = if rng.below(0, 2) == 0 {
        (smaller, smaller + taper)
    } else {
        (smaller + taper, smaller)
    };
    (half_height as f32, top as f32, bottom as f32)
}

/// Half height, top and bottom radius of a tapered cylinder, down to cones and radii near the
/// `2^-63` m floor.
fn cylinder(rng: &mut Rng) -> (f32, f32, f32) {
    let half_height = rng.log_range(1.0e-3, 100.0);
    let larger = rng.log_range(1.0e-20, 100.0);
    let smaller = match rng.below(0, 3) {
        0 => 0.0,
        1 => larger * rng.log_range(1.0e-6, 1.0),
        _ => larger * rng.unit(),
    };
    let (top, bottom) = if rng.below(0, 2) == 0 {
        (smaller, larger)
    } else {
        (larger, smaller)
    };
    (half_height as f32, top as f32, bottom as f32)
}

pub fn tapered_family(arena: &mut Arena) {
    let mut rng = Rng::new(0x5EED_0004);
    let mut accepted = 0;
    for index in 0..TAPERED_CASES {
        announce("tapered", index);
        let (what, result, extent) = if index.is_multiple_of(2) {
            let (h, top, bottom) = capsule(&mut rng, index / 2);
            let larger = top.max(bottom);
            (
                format!("tapered {index} (capsule {h} {top} {bottom})"),
                Shape::new_tapered_capsule(h, top, bottom),
                Extent::of(&[
                    Vec3::new(-larger, -h - bottom, -larger),
                    Vec3::new(larger, h + top, larger),
                ]),
            )
        } else {
            let (h, top, bottom) = cylinder(&mut rng);
            let larger = top.max(bottom);
            let convex_radius = [0.0, 0.05][index / 2 % 2];
            (
                format!("tapered {index} (cylinder {h} {top} {bottom})"),
                Shape::new_tapered_cylinder_with_convex_radius(h, top, bottom, convex_radius),
                Extent::of(&[
                    Vec3::new(-larger, -h, -larger),
                    Vec3::new(larger, h, larger),
                ]),
            )
        };
        let shape = match result {
            Ok(shape) => shape,
            Err(error) => {
                eprintln!("{what}: {error}");
                continue;
            }
        };
        accepted += 1;
        if accepted % 4 == 0 && !arena.drop_dynamic(&shape, extent, 30, &what) {
            arena.drop_probes_on(&shape, extent, 30, &what);
        }
    }
    eprintln!("tapered: {accepted} of {TAPERED_CASES} accepted");
    assert!(
        accepted > TAPERED_CASES / 2,
        "only {accepted} tapered shapes were accepted"
    );
}
