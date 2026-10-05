use super::*;

const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

/// The axis-aligned vectors with `value` on one axis, in both directions.
fn on_axes(value: f32) -> Vec<Vec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(Vec3::from(v));
        }
    }
    vectors
}

fn real_on_axes(value: Real) -> Vec<RVec3> {
    let mut vectors = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut v = [0.0; 3];
            v[axis] = sign * value;
            vectors.push(RVec3::from(v));
        }
    }
    vectors
}

#[test]
fn max_position_follows_the_precision_of_real() {
    let expected = if core::mem::size_of::<Real>() == 8 {
        1.0e7
    } else {
        5.0e3
    };
    assert_eq!(MAX_POSITION, expected);
}

#[test]
fn position_checks_accept_their_bound_and_reject_beyond() {
    type Check = fn(RVec3) -> bool;
    let checks: [(Check, Real); 2] = [
        (is_in_frame, MAX_POSITION),
        (is_frame_displacement, 2.0 * MAX_POSITION),
    ];
    for (check, bound) in checks {
        for v in real_on_axes(bound) {
            assert!(check(v), "{v:?}");
        }
        for v in real_on_axes(bound.next_up()) {
            assert!(!check(v), "{v:?}");
        }
        for value in NON_FINITE {
            for v in real_on_axes(Real::from(value)) {
                assert!(!check(v), "{v:?}");
            }
        }
    }
}

#[test]
fn vector_checks_accept_their_bound_and_reject_beyond() {
    type Check = fn(Vec3) -> bool;
    // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
    #[allow(clippy::unnecessary_cast)]
    let span = (2.0 * MAX_POSITION) as f32;
    let checks: [(Check, f32); 5] = [
        (is_frame_span, span),
        (is_local_offset, MAX_SHAPE_EXTENT),
        (is_linear_velocity, MAX_LINEAR_VELOCITY),
        (is_angular_velocity, MAX_ANGULAR_VELOCITY),
        (is_acceleration, MAX_ACCELERATION),
    ];
    for (check, bound) in checks {
        for v in on_axes(bound) {
            assert!(check(v), "{v:?}");
        }
        for v in on_axes(bound.next_up()) {
            assert!(!check(v), "{v:?}");
        }
        for value in NON_FINITE {
            for v in on_axes(value) {
                assert!(!check(v), "{v:?}");
            }
        }
    }
}

#[test]
fn the_largest_vertex_inverse_mass_is_the_smallest_mass_for_jolt() {
    assert_eq!(1.0 / MAX_VERTEX_INVERSE_MASS, MIN_MASS);
    assert!(1.0 / MAX_VERTEX_INVERSE_MASS.next_up() < MIN_MASS);
}

#[test]
fn scalar_checks_accept_their_range_and_reject_beyond() {
    type Check = fn(f32) -> bool;
    let checks: [(Check, &[f32], &[f32]); 6] = [
        (
            is_friction,
            &[0.0, MAX_FRICTION],
            &[-f32::MIN_POSITIVE, MAX_FRICTION.next_up()],
        ),
        (
            is_local_distance,
            &[0.0, MAX_SHAPE_EXTENT],
            &[-f32::MIN_POSITIVE, MAX_SHAPE_EXTENT.next_up()],
        ),
        (
            is_gravity_factor,
            &[-MAX_GRAVITY_FACTOR, 0.0, MAX_GRAVITY_FACTOR],
            &[
                (-MAX_GRAVITY_FACTOR).next_down(),
                MAX_GRAVITY_FACTOR.next_up(),
            ],
        ),
        (
            is_mass,
            &[MIN_MASS, MAX_MASS],
            &[0.0, MIN_MASS.next_down(), MAX_MASS.next_up()],
        ),
        (
            is_vertex_inverse_mass,
            &[1.0 / MAX_MASS, 1.0 / MIN_MASS, MAX_VERTEX_INVERSE_MASS],
            &[
                0.0,
                (1.0 / MAX_MASS).next_down(),
                MAX_VERTEX_INVERSE_MASS.next_up(),
            ],
        ),
        (
            is_compliance,
            &[0.0, MAX_COMPLIANCE],
            &[-f32::MIN_POSITIVE, MAX_COMPLIANCE.next_up()],
        ),
    ];
    for (check, accepted, rejected) in checks {
        for &value in accepted {
            assert!(check(value), "{value}");
        }
        for &value in rejected.iter().chain(&NON_FINITE) {
            assert!(!check(value), "{value}");
        }
    }
}

/// The vector along `direction` (components 1 or 0) whose Jolt length is the largest at
/// most `bound`, found by stepping one component.
fn on_the_jolt_bound(direction: Vec3, bound: f32) -> Vec3 {
    let mut v = direction.scale(bound / jolt_length(direction));
    while jolt_length(v) > bound {
        v.x = v.x.next_down();
    }
    while jolt_length(Vec3::new(v.x.next_up(), v.y, v.z)) <= bound {
        v.x = v.x.next_up();
    }
    v
}

#[test]
fn velocity_checks_use_jolts_length_on_diagonals() {
    for (check, bound) in [
        (is_linear_velocity as fn(Vec3) -> bool, MAX_LINEAR_VELOCITY),
        (is_angular_velocity, MAX_ANGULAR_VELOCITY),
    ] {
        let v = on_the_jolt_bound(Vec3::new(1.0, 1.0, 1.0), bound);
        assert!(check(v), "{v:?}");
        assert!(!check(Vec3::new(v.x.next_up(), v.y, v.z)), "{v:?}");
    }
}

#[test]
fn weight_impulse_keeps_the_angular_speed_of_the_ground_body_finite() {
    let largest_kept_inverse_inertia = 3.0_f64.sqrt() * 1.0e6;
    let largest_lever = 3.0_f64.sqrt() * f64::from(MAX_SHAPE_EXTENT);
    let angular_speed =
        largest_kept_inverse_inertia * largest_lever * f64::from(MAX_WEIGHT_IMPULSE)
            + f64::from(MAX_ANGULAR_VELOCITY);
    assert!(angular_speed * angular_speed < f64::from(f32::MAX) / 4.0);
    let sphere_inverse_inertia = 2.5 / f64::from(MIN_MASS);
    assert!(sphere_inverse_inertia < largest_kept_inverse_inertia);
}

#[test]
fn weight_impulse_check_accepts_its_bound_and_rejects_beyond() {
    let gravity = MAX_WEIGHT_IMPULSE / MAX_MASS;
    for down in on_axes(gravity) {
        assert!(is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
        assert!(is_weight_impulse(0.0, down, 1.0), "{down:?}");
    }
    for down in on_axes(gravity.next_up()) {
        assert!(!is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
    }
    assert!(!is_weight_impulse(
        MAX_MASS,
        Vec3::new(0.0, -gravity, 0.0),
        1.0f32.next_up()
    ));
}

#[test]
fn lever_arm_ratios_measure_the_lever_against_the_radius_of_gyration() {
    // A 2 kg cube of side 1 m: inertia 2 / 6 about every axis, radius of gyration² 1 / 6.
    let inverse_inertia = Vec3::new(3.0, 3.0, 3.0);
    let ratio = lever_arm_ratio(0.5, inverse_inertia, Vec3::new(0.0, 2.0, 0.0));
    assert!((ratio - 2.0 * 4.0 * 6.0).abs() < 1e-9, "{ratio}");
    // A thin rod of 1 kg and 2 m along y held at its end: 6 whatever its thickness.
    let rod = Vec3::new(3.0, 1.0e4, 3.0);
    let ratio = lever_arm_ratio(1.0, rod, Vec3::new(0.0, 1.0, 0.0));
    assert!((ratio - 6.0).abs() < 1e-9, "{ratio}");
    // Anywhere within a distance: the two largest inverse inertias.
    let within = lever_arm_ratio_within(1.0, rod, 1.0);
    assert!((within - (1.0e4 + 3.0)).abs() < 1e-6, "{within}");
    let bound = f64::from(MAX_LEVER_ARM_RATIO);
    assert!(is_lever_arm_ratio(bound));
    assert!(!is_lever_arm_ratio(bound.next_up()));
    assert!(!is_lever_arm_ratio(f64::NAN));
    assert!(!is_lever_arm_ratio(f64::INFINITY));
}

#[test]
fn principal_levers_measure_the_lever_with_the_inverse_inertia() {
    let mass = PrincipalMass {
        inverse_mass: 0.5,
        inverse_inertia: [1.0, 2.0, 3.0],
    };
    let lever_along = |r: [f64; 3], d: [f64; 3]| {
        let c = cross(r, d);
        (0..3)
            .map(|k| mass.inverse_inertia[k] * c[k] * c[k])
            .sum::<f64>()
            .sqrt()
    };
    let (y, z) = ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
    // At (2, 0, 0): r × y = (0, 0, 2) weighted by 3, r × z = (0, −2, 0) weighted by 2.
    assert!((mass.lever_in_plane([2.0, 0.0, 0.0], y, z) - 12.0_f64.sqrt()).abs() < 1e-12);
    // At (0, 1, 1): r × y = (−1, 0, 0) and r × z = (1, 0, 0), largest along (y − z) / √2.
    assert!((mass.lever_in_plane([0.0, 1.0, 1.0], y, z) - 2.0_f64.sqrt()).abs() < 1e-12);
    // It is the largest lever over the directions of the plane.
    let tilted = normalized([0.0, 1.0, 1.0]);
    let across = cross(tilted, [1.0, 0.0, 0.0]);
    let r = [0.7, -1.3, 2.1];
    let largest = mass.lever_in_plane(r, tilted, across);
    let sampled = (0..3600)
        .map(|k| {
            let (sin, cos) = (f64::from(k) * std::f64::consts::PI / 1800.0).sin_cos();
            lever_along(r, [0, 1, 2].map(|i| cos * tilted[i] + sin * across[i]))
        })
        .fold(0.0, f64::max);
    assert!(sampled <= largest + 1e-12 && sampled >= largest * (1.0 - 1e-5));
    // Per metre: the largest inverse inertia, which bounds every unit lever in every direction.
    assert!((mass.lever_per_metre() - 3.0_f64.sqrt()).abs() < 1e-12);
    for k in 0..64 {
        let angle = f64::from(k) * 0.1;
        let u = normalized([angle.cos(), angle.sin(), 0.3 * angle]);
        let d = normalized([angle.sin(), 0.5, angle.cos()]);
        assert!(lever_along(u, d) <= mass.lever_per_metre() + 1e-12);
    }
    let bound = f64::from(MAX_TRACK_MASS_RATIO);
    assert!(is_track_mass_ratio(bound));
    assert!(!is_track_mass_ratio(bound.next_up()));
    assert!(!is_track_mass_ratio(f64::NAN));
}

#[test]
fn accelerations_are_bounded_by_the_speed_clamp_per_step() {
    let per_step = MAX_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
    assert!((per_step - MAX_LINEAR_VELOCITY).abs() <= MAX_LINEAR_VELOCITY * 1.0e-6);
    let per_step = MAX_ANGULAR_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
    assert!((per_step - MAX_ANGULAR_VELOCITY).abs() <= MAX_ANGULAR_VELOCITY * 1.0e-6);
}

/// The tetrahedron with a right corner at `origin`, unit edges along x and z and the fourth
/// vertex `height` above `(1, 0, 1)`, faces wound counter-clockwise seen from outside.
fn tetrahedron(origin: Vec3, height: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let positions = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, height, 1.0],
    ]
    .map(|[x, y, z]| Vec3::new(origin.x + x, origin.y + y, origin.z + z))
    .to_vec();
    (positions, vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
}

/// Six-volumes as Jolt might compute them in `f32` after rotating the positions by
/// `rotation` with Jolt's quaternion rotation: once with separate multiplications and once
/// with fused multiply-adds in the cross and dot products.
fn f32_six_volumes(positions: &[Vec3], faces: &[[u32; 3]], rotation: crate::Quat) -> [f32; 2] {
    let rotated: Vec<Vec3> = positions
        .iter()
        .map(|&p| crate::math::jolt_rotate(rotation, p))
        .collect();
    let mut plain = 0.0_f32;
    let mut fused = 0.0_f32;
    for &[a, b, c] in faces {
        let [x1, x2, x3] = [a, b, c].map(|i| rotated[i as usize]);
        let cross = Vec3::new(
            x1.y * x2.z - x1.z * x2.y,
            x1.z * x2.x - x1.x * x2.z,
            x1.x * x2.y - x1.y * x2.x,
        );
        plain += (cross.x * x3.x + cross.y * x3.y) + cross.z * x3.z;
        let cross = Vec3::new(
            x1.y.mul_add(x2.z, -(x1.z * x2.y)),
            x1.z.mul_add(x2.x, -(x1.x * x2.z)),
            x1.x.mul_add(x2.y, -(x1.y * x2.x)),
        );
        fused += cross.z.mul_add(x3.z, cross.x.mul_add(x3.x, cross.y * x3.y));
    }
    [plain, fused]
}

#[test]
fn pressure_volume_bound_is_below_f32_six_volumes() {
    // Seeded rotations, normalized within Jolt's tolerance.
    let mut seed = 0x2545_f491_u32;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32 * 2.0 - 1.0
    };
    let mut checked = 0;
    // Rounding grows with the distance from the origin, so heights from 1e-12 to 1 cover
    // the region where the bound becomes positive for each origin.
    for origin in [
        Vec3::ZERO,
        Vec3::new(3.0, -2.0, 4.0),
        Vec3::new(10.0, -7.0, 15.0),
    ] {
        for step in 0..=48 {
            let height = 10.0_f32.powf(-12.0 + step as f32 / 4.0);
            let (positions, faces) = tetrahedron(origin, height);
            let geometry = SoftBodyPressureGeometry::new(&positions, &faces);
            if geometry.six_volume_low <= 0.0 {
                continue;
            }
            for _ in 0..64 {
                let rotation = crate::Quat::from_xyzw(next(), next(), next(), next()).normalized();
                for volume in f32_six_volumes(&positions, &faces, rotation) {
                    assert!(
                        f64::from(volume) >= geometry.six_volume_low,
                        "origin {origin:?}, height {height}: {volume} < {}",
                        geometry.six_volume_low
                    );
                }
                checked += 1;
            }
        }
    }
    assert!(checked >= 64 * 30, "{checked}");
}

#[test]
fn pressure_needs_a_positive_volume_and_bounds_the_vertex_force() {
    // The thin tetrahedron of a review: 1 m edges, faces with area, six-volume 1e-36.
    let (positions, faces) = tetrahedron(Vec3::ZERO, 1.0e-36);
    let sliver = SoftBodyPressureGeometry::new(&positions, &faces);
    assert!(sliver.six_volume_low < 0.0, "{sliver:?}");
    assert!(is_soft_body_pressure(0.0, &sliver));
    assert!(!is_soft_body_pressure(f32::MIN_POSITIVE, &sliver));
    // No faces enclose nothing.
    let empty = SoftBodyPressureGeometry::new(&positions, &[]);
    assert!(!is_soft_body_pressure(f32::MIN_POSITIVE, &empty));
    // A unit tetrahedron: the vertex of the largest face sum is (1, 1, 1), whose faces have
    // the edge products sqrt(3) * 1, 1 * sqrt(3) and sqrt(2) * sqrt(2).
    let (positions, faces) = tetrahedron(Vec3::ZERO, 1.0);
    let unit = SoftBodyPressureGeometry::new(&positions, &faces);
    let area = 2.0 * 3.0_f64.sqrt() + 2.0;
    assert!(
        (unit.largest_vertex_face_area / area - 1.0).abs() < 1.0e-3,
        "{unit:?}"
    );
    assert!((unit.six_volume_low - 1.0).abs() < 1.0e-4, "{unit:?}");
    let bound = MAX_PRESSURE_VERTEX_FORCE * unit.six_volume_low / unit.largest_vertex_face_area;
    let bound = bound as f32;
    assert!(is_soft_body_pressure(bound.next_down(), &unit));
    assert!(!is_soft_body_pressure(bound.next_up(), &unit));
}

/// An 11 × 11 cloth of 1 kg vertices, 0.1 m apart, centred `distance` from the origin
/// along (1, 0, 1).
fn offset_cloth(distance: f32) -> SoftBodyMassDistribution {
    let offset = distance / 2.0_f32.sqrt();
    SoftBodyMassDistribution::new((0..121).map(|i| {
        let x = (i % 11) as f32 * 0.1 - 0.5 + offset;
        let z = (i / 11) as f32 * 0.1 - 0.5 + offset;
        (Vec3::new(x, 0.0, z), 1.0)
    }))
}

/// `scale · R diag(1, 1, moment) Rᵀ` for a fixed general rotation `R`.
fn rotated_needle(moment: f64, scale: f64) -> [[f64; 3]; 3] {
    let r = jolt_rotation_matrix(crate::Quat::from_xyzw(0.3, -0.2, 0.5, 0.78).normalized());
    let d = [1.0, 1.0, moment];
    std::array::from_fn(|i| {
        std::array::from_fn(|j| scale * (0..3).map(|k| r[i][k] * d[k] * r[j][k]).sum::<f64>())
    })
}

#[test]
fn rigid_body_inertia_floor_holds_at_its_boundary() {
    // For diag(1, 1, m), det / minors = m / (1 + 2m) and |I|_F = sqrt(2 + m²).
    let (mut low, mut high) = (0.0_f64, 1.0_f64);
    for _ in 0..200 {
        let m = 0.5 * (low + high);
        if m / (1.0 + 2.0 * m) >= MIN_INERTIA_RATIO * (2.0 + m * m).sqrt() {
            high = m;
        } else {
            low = m;
        }
    }
    assert!((6.7e-4..6.9e-4).contains(&high), "{high}");
    // The rule does not depend on the scale, also below Jolt's near-zero limit.
    for scale in [1.0e-30, 1.0e-9, 1.0, 1.0e9] {
        assert!(is_rigid_body_inertia(rotated_needle(
            high * (1.0 + 1e-6),
            scale
        )));
        assert!(!is_rigid_body_inertia(rotated_needle(
            high * (1.0 - 1e-6),
            scale
        )));
    }
    // Not positive definite.
    assert!(!is_rigid_body_inertia(rotated_needle(0.0, 1.0)));
    assert!(!is_rigid_body_inertia(rotated_needle(-0.1, 1.0)));
}

#[test]
fn soft_body_inertia_needs_vertices_around_the_origin() {
    let rotation = crate::Quat::from_xyzw(0.3, -0.2, 0.5, 0.78).normalized();
    for rotation in [None, Some(rotation)] {
        for distance in [0.0, 1.0, 5.0, 10.0] {
            assert!(is_soft_body_inertia(&offset_cloth(distance), rotation));
        }
        for distance in [15.0, 50.0, 1000.0] {
            assert!(!is_soft_body_inertia(&offset_cloth(distance), rotation));
        }
    }
    // The identity rotation is no rotation for Jolt.
    assert!(is_soft_body_inertia(
        &offset_cloth(10.0),
        Some(crate::Quat::IDENTITY)
    ));
    // A kinematic vertex skips the decomposition.
    let pinned = SoftBodyMassDistribution::new([
        (Vec3::new(1000.0, 0.0, 1000.0), 0.0),
        (Vec3::new(1000.1, 0.0, 1000.0), 1.0),
    ]);
    assert!(is_soft_body_inertia(&pinned, None));
    // Exactly diagonal tensors: near zero, or every moment invertible.
    let at = |positions: &[[f32; 3]]| {
        SoftBodyMassDistribution::new(positions.iter().map(|&p| (Vec3::from(p), 1.0)))
    };
    assert!(is_soft_body_inertia(&at(&[[0.0; 3]]), None));
    assert!(is_soft_body_inertia(
        &at(&[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, -3.0]]),
        None
    ));
    assert!(!is_soft_body_inertia(&at(&[[1.0, 0.0, 0.0]]), None));
    assert!(!is_soft_body_inertia(
        &at(&[[1.0, 0.0, 0.0], [-2.0, 0.0, 0.0]]),
        None
    ));
    // A line off the axes has the same zero moment.
    assert!(!is_soft_body_inertia(
        &at(&[[0.6, 0.8, 0.0], [1.2, 1.6, 0.0]]),
        None
    ));
    // A rotation makes the axis-aligned set general; its moments still pass.
    assert!(is_soft_body_inertia(
        &at(&[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, -3.0]]),
        Some(rotation)
    ));
}

#[test]
fn contact_scale_floor_is_the_mass_ratio() {
    let ratio = f64::from(MIN_MASS) / f64::from(MAX_MASS);
    assert!((f64::from(MIN_CONTACT_SCALE) - ratio).abs() <= ratio * f64::from(f32::EPSILON));
    assert!(is_contact_scale(1.0e-9) && !is_contact_scale(MIN_CONTACT_SCALE.next_down()));
}

#[test]
fn soft_body_force_is_bounded_whatever_the_inverse_masses() {
    let bound = f64::from(MAX_ACCELERATION) * f64::from(MAX_MASS);
    for w in [0.0, 1.0 / MAX_MASS] {
        assert!(is_soft_body_force([bound, 0.0, 0.0], w, 4));
        assert!(!is_soft_body_force([bound * (1.0 + 1e-12), 0.0, 0.0], w, 4));
    }
    let per_vertex = 4.0 * f64::from(MAX_ACCELERATION);
    assert!(is_soft_body_force([0.0, per_vertex, 0.0], 1.0, 4));
    assert!(!is_soft_body_force(
        [0.0, per_vertex, 0.0],
        1.0f32.next_up(),
        4
    ));
    assert!(!is_soft_body_force([f64::NAN, 0.0, 0.0], 0.0, 4));
}
