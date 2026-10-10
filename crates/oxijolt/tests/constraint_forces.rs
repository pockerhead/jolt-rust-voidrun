//! Constraint impulse readouts: every `total_lambda*` getter against the load its constraint
//! holds, worked out from mass, lever arm and step length, with signs in frames where Jolt's
//! constraint axes are known; channels that carry no load read zero. Also that the readout is
//! the impulse of the whole step, and that disabled and sleeping constraints keep their last
//! values.

mod common;

use std::f32::consts::PI;

use common::constraint_rigs::{anchor, cube, readout_bits, sleepy_cube, HALF};
use common::{length, world, CALM_SPEED, DT};
use oxijolt::*;

const G: f32 = 9.81;
const GRAVITY: Vec3 = Vec3::new(0.0, -G, 0.0);
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
/// Mass of a held cube, kg.
const MASS: f32 = 10.0;
/// Where a held cube starts.
const CENTRE: RVec3 = RVec3::new(0.0, 5.0, 0.0);
/// Ticks after which a held load has settled.
const SETTLE: usize = 120;
/// Relative tolerance of a settled readout.
const REL: f32 = 0.01;
/// A couple applied to a held cube, N·m.
const TORQUE: f32 = 5.0;
/// A force applied to a held cube, N.
const FORCE: f32 = 10.0;

/// The impulse in N·s that holds `mass` kg against gravity for one step of `dt` seconds.
fn weight(mass: f32, dt: f32) -> f32 {
    mass * G * dt
}

fn close(actual: f32, expected: f32, rel: f32) -> bool {
    (actual - expected).abs() <= rel * expected.abs()
}

#[track_caller]
fn assert_close(actual: f32, expected: f32, what: &str) {
    assert!(
        close(actual, expected, REL),
        "{what}: {actual}, expected {expected}"
    );
}

/// `actual` is zero next to a readout of size `scale`: within 0.1 % of it.
#[track_caller]
fn assert_small(actual: f32, scale: f32, what: &str) {
    assert!(
        actual.abs() <= 1e-3 * scale.abs(),
        "{what}: {actual}, expected about 0 next to {scale}"
    );
}

/// Each component of `actual` close to `expected`, or small next to `scale` where `expected` is
/// zero.
#[track_caller]
fn assert_components(actual: &[f32], expected: &[f32], scale: f32, what: &str) {
    assert_eq!(actual.len(), expected.len());
    for (i, (&a, &e)) in actual.iter().zip(expected).enumerate() {
        if e == 0.0 {
            assert_small(a, scale, &format!("{what}[{i}] of {actual:?}"));
        } else {
            assert_close(a, e, &format!("{what}[{i}] of {actual:?}"));
        }
    }
}

/// `v` scaled by `size`.
fn along(v: Vec3, size: f32) -> Vec3 {
    Vec3::new(v.x * size, v.y * size, v.z * size)
}

fn components(v: Vec3) -> [f32; 3] {
    [v.x, v.y, v.z]
}

fn at(offset: [f32; 3]) -> RVec3 {
    shifted(CENTRE, offset)
}

fn shifted(p: RVec3, d: [f32; 3]) -> RVec3 {
    RVec3::new(p.x + d[0] as Real, p.y + d[1] as Real, p.z + d[2] as Real)
}

/// The horizontal distance between `a` and `b`, metres.
// `Real` is `f32` without the `double-precision` feature.
#[allow(clippy::unnecessary_cast)]
fn horizontal(a: RVec3, b: RVec3) -> f32 {
    ((a.x - b.x) as f32).hypot((a.z - b.z) as f32)
}

/// A world with a [`MASS`] kg cube at `centre`, held by `settings` to a static anchor 1 m above
/// it. Nothing is stepped yet.
fn rig<S: ConstraintSettings>(
    gravity: Vec3,
    centre: RVec3,
    settings: &S,
) -> (PhysicsWorld, BodyId, ConstraintId<S::Kind>) {
    let mut world = world(gravity, 1);
    let anchor = anchor(&mut world, shifted(centre, [0.0, 1.0, 0.0]));
    let cube = cube(&mut world, centre, MASS);
    let id = world.create_constraint(anchor, cube, settings).unwrap();
    (world, cube, id)
}

/// Steps `ticks` times with `dt`, adding `force` and `torque` to `body` before each step.
fn run_loaded(
    world: &mut PhysicsWorld,
    dt: f32,
    ticks: usize,
    body: BodyId,
    force: Vec3,
    torque: Vec3,
) {
    for _ in 0..ticks {
        let mut cube = world.body_mut(body).unwrap();
        cube.add_force(force).unwrap();
        cube.add_torque(torque).unwrap();
        assert!(world.step(dt).unwrap().is_complete());
    }
}

fn run(world: &mut PhysicsWorld, dt: f32, ticks: usize) {
    for _ in 0..ticks {
        assert!(world.step(dt).unwrap().is_complete());
    }
}

/// The readout `read` of a cube at [`CENTRE`] held by `settings` under `gravity` and the couple
/// `torque`, after [`SETTLE`] ticks.
fn settled<S: ConstraintSettings, R>(
    gravity: Vec3,
    settings: &S,
    torque: Vec3,
    read: impl Fn(ConstraintRef<'_, S::Kind>) -> R,
) -> R {
    let (mut world, cube, id) = rig(gravity, CENTRE, settings);
    run_loaded(&mut world, DT, SETTLE, cube, Vec3::ZERO, torque);
    read(world.constraint(id).unwrap())
}

#[test]
fn a_weld_reads_the_weight_it_holds() {
    let weld = FixedConstraintSettings::new(at([0.0, HALF, 0.0]), X, Y);
    let (position, rotation) = settled(GRAVITY, &weld, Vec3::ZERO, |c| {
        (c.total_lambda_position(), c.total_lambda_rotation())
    });
    let w = weight(MASS, DT);
    assert_components(&components(position), &[0.0, w, 0.0], w, "position");
    // The weld is above the centre of mass: no torque, against the weight times 1 m.
    assert_components(&components(rotation), &[0.0; 3], w, "rotation");
}

#[test]
fn a_weld_off_the_centre_reads_the_torque_it_holds() {
    // Body 2 balances: (p - c) × λ_position + λ_rotation = 0, so λ_rotation = (c - p) × λ_position
    // with c - p = (-0.4, -0.25, 0) and λ_position = (0, w, 0).
    let weld = FixedConstraintSettings::new(at([0.4, HALF, 0.0]), X, Y);
    let (position, rotation) = settled(GRAVITY, &weld, Vec3::ZERO, |c| {
        (c.total_lambda_position(), c.total_lambda_rotation())
    });
    let w = weight(MASS, DT);
    assert_components(&components(position), &[0.0, w, 0.0], w, "position");
    assert_components(&components(rotation), &[0.0, 0.0, -0.4 * w], w, "rotation");
}

#[test]
fn a_weld_reads_a_couple_in_its_rotation_part() {
    let weld = FixedConstraintSettings::new(CENTRE, X, Y);
    let (position, rotation) = settled(Vec3::ZERO, &weld, along(Z, TORQUE), |c| {
        (c.total_lambda_position(), c.total_lambda_rotation())
    });
    let couple = TORQUE * DT;
    assert_components(
        &components(rotation),
        &[0.0, 0.0, -couple],
        couple,
        "rotation",
    );
    // At the centre of mass the couple needs no force; scale: the couple over 1 m.
    assert_components(&components(position), &[0.0; 3], couple, "position");
}

/// The weld readout after running each `(dt, ticks)` segment in turn.
fn weld_readout_after(segments: &[(f32, usize)]) -> f32 {
    let weld = FixedConstraintSettings::new(at([0.0, HALF, 0.0]), X, Y);
    let (mut world, _, id) = rig(GRAVITY, CENTRE, &weld);
    for &(dt, ticks) in segments {
        run(&mut world, dt, ticks);
    }
    world.constraint(id).unwrap().total_lambda_position().y
}

#[test]
fn the_readout_is_the_impulse_of_the_whole_step() {
    let at_60_hz = weld_readout_after(&[(DT, SETTLE)]);
    let at_30_hz = weld_readout_after(&[(2.0 * DT, SETTLE)]);
    let switched = weld_readout_after(&[(DT, SETTLE), (2.0 * DT, 60)]);
    assert_close(at_60_hz, weight(MASS, DT), "60 Hz");
    assert_close(at_30_hz, 2.0 * at_60_hz, "30 Hz");
    assert_close(switched, at_30_hz, "60 Hz, then 30 Hz");
}

#[test]
fn point_cone_swing_twist_and_six_dof_read_the_hanging_weight() {
    let w = weight(MASS, DT);
    // Hanging from the top face, then turned 90° about Z: gravity along -X, the pivot on the
    // +X face. The impulse follows the world.
    for (gravity, pivot, expected) in [
        (GRAVITY, [0.0, HALF, 0.0], [0.0, w, 0.0]),
        (Vec3::new(-G, 0.0, 0.0), [HALF, 0.0, 0.0], [w, 0.0, 0.0]),
    ] {
        let p = at(pivot);
        let fixed_translation = [
            SixDofConstraintAxis::TranslationX,
            SixDofConstraintAxis::TranslationY,
            SixDofConstraintAxis::TranslationZ,
        ]
        .into_iter()
        .fold(SixDofConstraintSettings::new(p, X, Y), |s, axis| {
            s.axis(axis, SixDofAxis::Fixed)
        });
        let readings = [
            settled(gravity, &PointConstraintSettings::new(p), Vec3::ZERO, |c| {
                c.total_lambda_position()
            }),
            settled(
                gravity,
                &ConeConstraintSettings::new(p, X, 0.0),
                Vec3::ZERO,
                |c| c.total_lambda_position(),
            ),
            settled(
                gravity,
                &SwingTwistConstraintSettings::new(p, X, Z),
                Vec3::ZERO,
                |c| c.total_lambda_position(),
            ),
            settled(gravity, &fixed_translation, Vec3::ZERO, |c| {
                c.total_lambda_position()
            }),
        ];
        for (kind, reading) in ["point", "cone", "swing-twist", "six-DOF"]
            .into_iter()
            .zip(readings)
        {
            assert_components(&components(reading), &expected, w, kind);
        }
    }
}

#[test]
fn distance_and_pulley_read_the_rope_tension() {
    let w = weight(MASS, DT);
    let top = at([0.0, HALF, 0.0]);
    let rope = DistanceConstraintSettings::new(at([0.0, 1.0, 0.0]), top);
    let taut = settled(GRAVITY, &rope, Vec3::ZERO, |c| c.total_lambda_position());
    assert_close(taut.abs(), w, "taut rope");

    // A rope of up to 2 m with 0.75 m out is slack while the cube falls the first 1.25 m (about
    // 30 ticks), then catches and holds it.
    let rope = rope.range(DistanceRange::Range { min: 0.0, max: 2.0 });
    let (mut fall, _, id) = rig(GRAVITY, CENTRE, &rope);
    for _ in 0..10 {
        run(&mut fall, DT, 1);
        assert_eq!(fall.constraint(id).unwrap().total_lambda_position(), 0.0);
    }
    run(&mut fall, DT, 3 * SETTLE);
    let caught = fall.constraint(id).unwrap().total_lambda_position();
    assert_close(caught.abs(), w, "caught rope");

    // Body 2's rope carries `ratio` times body 1's impulse: 2 kg on rope 1 balance 2 kg at
    // ratio 1 and 4 kg at ratio 2.
    for (ratio, mass2) in [(1.0, 2.0), (2.0, 4.0)] {
        let mut world = world(GRAVITY, 1);
        let crate1 = cube(&mut world, RVec3::new(-1.0, 3.0, 0.0), 2.0);
        let crate2 = cube(&mut world, RVec3::new(1.0, 3.0, 0.0), mass2);
        let pulley = PulleyConstraintSettings::new(
            RVec3::new(-1.0, 3.0 + HALF as Real, 0.0),
            RVec3::new(-1.0, 5.0, 0.0),
            RVec3::new(1.0, 3.0 + HALF as Real, 0.0),
            RVec3::new(1.0, 5.0, 0.0),
        )
        .ratio(ratio);
        let id = world.create_constraint(crate1, crate2, &pulley).unwrap();
        run(&mut world, DT, SETTLE);
        // The rope pulls: the impulse is negative.
        let tension = world.constraint(id).unwrap().total_lambda_position();
        assert_close(tension, -weight(2.0, DT), &format!("pulley ratio {ratio}"));
        for crate_ in [crate1, crate2] {
            let speed = length(world.body(crate_).unwrap().linear_velocity());
            assert!(
                speed < CALM_SPEED,
                "ratio {ratio}: a crate moves at {speed} m/s"
            );
        }
    }
}

#[test]
fn hinge_and_slider_read_their_new_parts() {
    let w = weight(MASS, DT);
    // Hinge axis X with its pin 0.5 m along X from the centre. The position part holds the weight
    // at the pin, a torque (0.5, 0, 0) × (0, w, 0) = 0.5 w about +Z on body 2, which the rotation
    // part cancels. Jolt's rotation axes for hinge axis X are b2 × a1 = -Y and c2 × a1 = -Z
    // (`HingeRotationConstraintPart`, b2 = X.GetNormalizedPerpendicular() = -Z, c2 = X × b2 = Y),
    // so the impulse -0.5 w Z is the second component, +0.5 w.
    let hinge = HingeConstraintSettings::new(at([0.5, 0.0, 0.0]), X, Y);
    let (position, rotation) = settled(GRAVITY, &hinge, Vec3::ZERO, |c| {
        (c.total_lambda_position(), c.total_lambda_rotation())
    });
    assert_components(&components(position), &[0.0, w, 0.0], w, "hinge position");
    assert_components(&rotation, &[0.0, 0.5 * w], w, "hinge rotation");

    // Slider axis X at the centre, with a couple about Z. The position axes are the normal n1
    // and n2 = slider × n1: with normal Y the weight is along n1, with normal Z along n2 = -Y.
    let couple = TORQUE * DT;
    for (normal, expected) in [(Y, [w, 0.0]), (Z, [0.0, -w])] {
        let slider = SliderConstraintSettings::new(CENTRE, X, normal);
        let (position, rotation) = settled(GRAVITY, &slider, along(Z, TORQUE), |c| {
            (c.total_lambda_position(), c.total_lambda_rotation())
        });
        assert_components(&position, &expected, w, "slider position");
        assert_components(
            &components(rotation),
            &[0.0, 0.0, -couple],
            couple,
            "slider rotation",
        );
    }
}

/// The angular impulse in N·m·s that holds a [`MASS`] kg cube centred at `centre` against
/// gravity about a horizontal axis through `pivot`, for one step.
fn weight_moment(pivot: RVec3, centre: RVec3) -> f32 {
    weight(MASS, DT) * horizontal(pivot, centre)
}

#[test]
fn motors_and_limits_read_the_load_they_hold() {
    let w = weight(MASS, DT);
    // A hinge about Z with the cube 0.5 m out along +X: velocity motor at 0, limits it rests on,
    // friction above the load.
    let pin = CENTRE;
    let arm = at([0.5, 0.0, 0.0]);
    let hinge = HingeConstraintSettings::new(pin, Z, X);
    let cases = [
        ("velocity motor", hinge.clone(), MotorState::Velocity),
        (
            "friction",
            hinge.clone().max_friction_torque(100.0),
            MotorState::Off,
        ),
        ("limits", hinge.limits(0.0, 0.5), MotorState::Off),
    ];
    for (what, settings, motor) in cases {
        let (mut world, cube, id) = rig(GRAVITY, arm, &settings);
        world.constraint_mut(id).unwrap().set_motor_state(motor);
        run(&mut world, DT, SETTLE);
        let expected = weight_moment(pin, world.body(cube).unwrap().position());
        let c = world.constraint(id).unwrap();
        let (motor, limits) = (c.total_lambda_motor(), c.total_lambda_rotation_limits());
        let (held, idle) = if what == "limits" {
            (limits, motor)
        } else {
            (motor, limits)
        };
        assert_close(held.abs(), expected, what);
        assert_eq!(idle, 0.0, "{what}");
    }

    // A vertical slider: a velocity motor at 0, then limits it rests on.
    let slider = SliderConstraintSettings::new(CENTRE, Y, X);
    let (mut world, _, id) = rig(GRAVITY, CENTRE, &slider);
    world
        .constraint_mut(id)
        .unwrap()
        .set_motor_state(MotorState::Velocity);
    run(&mut world, DT, SETTLE);
    let c = world.constraint(id).unwrap();
    assert_close(c.total_lambda_motor().abs(), w, "slider motor");
    assert_eq!(c.total_lambda_position_limits(), 0.0);
    let limited = settled(GRAVITY, &slider.limits(-0.1, 0.1), Vec3::ZERO, |c| {
        (c.total_lambda_position_limits(), c.total_lambda_motor())
    });
    assert_close(limited.0.abs(), w, "slider limits");
    assert_eq!(limited.1, 0.0);

    // A cone of half angle 0.3 around X with the cube on a 0.5 m arm: it swings down onto the
    // cone and rests there.
    let pivot = at([-0.5, 0.0, 0.0]);
    let cone = ConeConstraintSettings::new(pivot, X, 0.3);
    let (mut world, cube, id) = rig(GRAVITY, CENTRE, &cone);
    run(&mut world, DT, SETTLE);
    let expected = weight_moment(pivot, world.body(cube).unwrap().position());
    let rotation = world.constraint(id).unwrap().total_lambda_rotation();
    assert_close(rotation.abs(), expected, "cone");
}

#[test]
fn swing_twist_limits_and_motors_read_the_couple_they_hold() {
    // The frame is the world's axes (twist X, plane Z, so the normal is Y): a couple about one
    // axis turns the cube about its centre onto that axis' limit only. Swing Y is limited by the
    // plane half angle, swing Z by the normal half angle; the other swing is locked.
    let couple = TORQUE * DT;
    let joint = SwingTwistConstraintSettings::new(CENTRE, X, Z);
    let cases = [
        (joint.clone().twist_limits(-0.3, 0.3), X, 0),
        (joint.clone().half_cone_angles(0.0, 0.3), Y, 1),
        (joint.clone().half_cone_angles(0.3, 0.0), Z, 2),
    ];
    for (settings, axis, index) in cases {
        let limits = settled(Vec3::ZERO, &settings, along(axis, TORQUE), |c| {
            c.total_lambda_limits()
        });
        let mut expected = [0.0; 3];
        expected[index] = couple;
        let limits = limits.map(f32::abs);
        assert_components(&limits, &expected, couple, &format!("limit {index}"));
    }

    // Velocity motors at 0 with every limit off hold a couple per constraint axis.
    let free = joint.twist_limits(-PI, PI).half_cone_angles(PI, PI);
    let (mut world, cube, id) = rig(Vec3::ZERO, CENTRE, &free);
    let mut joint = world.constraint_mut(id).unwrap();
    joint.set_swing_motor_state(MotorState::Velocity);
    joint.set_twist_motor_state(MotorState::Velocity);
    let torque = Vec3::new(1.0, 2.0, 3.0);
    run_loaded(&mut world, DT, SETTLE, cube, Vec3::ZERO, torque);
    let c = world.constraint(id).unwrap();
    let expected = components(torque).map(|t| -t * DT);
    assert_components(&components(c.total_lambda_motor()), &expected, DT, "motors");
    assert_components(&c.total_lambda_limits(), &[0.0; 3], DT, "limits off");
}

#[test]
fn two_limited_swing_axes_read_one_limit_in_the_swing_y_part() {
    // With both swing axes limited Jolt solves a single swing limit, about the axis from the
    // clamped swing to the current one, in its swing Y part; the swing Z part reads 0. A limit
    // pushes back only, so it reads -couple whichever way the couple turns the cube. Six-DOF with
    // free or limited rotation axes reads the same parts.
    use SixDofConstraintAxis::*;
    let couple = TORQUE * DT;
    let swing_twist = SwingTwistConstraintSettings::new(CENTRE, X, Z)
        .twist_limits(-PI, PI)
        .half_cone_angles(0.3, 0.3);
    let limited = SixDofAxis::Limited {
        min: -0.3,
        max: 0.3,
    };
    let six_dof = [TranslationX, TranslationY, TranslationZ]
        .into_iter()
        .fold(SixDofConstraintSettings::new(CENTRE, X, Y), |s, axis| {
            s.axis(axis, SixDofAxis::Fixed)
        })
        .axis(RotationY, limited)
        .axis(RotationZ, limited);
    for torque in [Z, along(Z, -1.0), Y, along(Y, -1.0)].map(|t| along(t, TORQUE)) {
        let what = format!("{torque:?}");
        let limits = settled(Vec3::ZERO, &swing_twist, torque, |c| {
            c.total_lambda_limits()
        });
        let expected = [0.0, -couple, 0.0];
        assert_components(&limits, &expected, couple, &format!("swing-twist, {what}"));
        let rotation = settled(Vec3::ZERO, &six_dof, torque, |c| c.total_lambda_rotation());
        assert_components(
            &components(rotation),
            &expected,
            couple,
            &format!("six-DOF, {what}"),
        );
    }

    // At its minimum a limit's axis is reversed, so a twist held at either end reads -couple.
    let twist = SwingTwistConstraintSettings::new(CENTRE, X, Z).twist_limits(-0.3, 0.3);
    for torque in [TORQUE, -TORQUE] {
        let limits = settled(Vec3::ZERO, &twist, along(X, torque), |c| {
            c.total_lambda_limits()
        });
        let what = format!("twist under {torque} N·m");
        assert_components(&limits, &[-couple, 0.0, 0.0], couple, &what);
    }
}

#[test]
fn world_vectors_follow_the_world_and_axis_parts_the_frame() {
    // The frame turned 0.5 rad about X: n1 = (0, cos, sin) and n2 = X × n1 = (0, -sin, cos). A
    // weld's position part stays the world vector (0, w, 0); the slider's two position parts and
    // a six-DOF joint's per-axis translation parts split it over the turned axes.
    use SixDofConstraintAxis::*;
    let w = weight(MASS, DT);
    let (sin, cos) = 0.5f32.sin_cos();
    let n1 = Vec3::new(0.0, cos, sin);
    let weld = FixedConstraintSettings::new(at([0.0, HALF, 0.0]), X, n1);
    let position = settled(GRAVITY, &weld, Vec3::ZERO, |c| c.total_lambda_position());
    assert_components(&components(position), &[0.0, w, 0.0], w, "weld");

    let slider = SliderConstraintSettings::new(CENTRE, X, n1);
    let position = settled(GRAVITY, &slider, Vec3::ZERO, |c| c.total_lambda_position());
    assert_components(&position, &[w * cos, -w * sin], w, "slider");

    let six_dof = [TranslationX, TranslationZ, RotationX, RotationY, RotationZ]
        .into_iter()
        .fold(SixDofConstraintSettings::new(CENTRE, X, n1), |s, axis| {
            s.axis(axis, SixDofAxis::Fixed)
        })
        .axis(
            TranslationY,
            SixDofAxis::Limited {
                min: -0.1,
                max: 0.1,
            },
        );
    let position = settled(GRAVITY, &six_dof, Vec3::ZERO, |c| c.total_lambda_position());
    assert_components(
        &components(position),
        &[0.0, w * cos, -w * sin],
        w,
        "six-DOF",
    );
}

#[test]
fn a_soft_limit_spring_makes_six_dof_translation_per_axis() {
    // All six axes fixed in the frame turned 0.5 rad about X of the test above. Rigid, the
    // translation reads the world vector (0, w, 0); with a soft limit spring on one translation
    // axis Jolt solves the three axes apart and reads (0, w cos, -w sin), of the same length.
    use SixDofConstraintAxis::*;
    let w = weight(MASS, DT);
    let (sin, cos) = 0.5f32.sin_cos();
    let n1 = Vec3::new(0.0, cos, sin);
    let rigid = [
        TranslationX,
        TranslationY,
        TranslationZ,
        RotationX,
        RotationY,
        RotationZ,
    ]
    .into_iter()
    .fold(SixDofConstraintSettings::new(CENTRE, X, n1), |s, axis| {
        s.axis(axis, SixDofAxis::Fixed)
    });
    let soft = rigid.clone().limits_spring(
        TranslationY,
        SpringSettings::FrequencyAndDamping {
            frequency: 20.0,
            damping: 1.0,
        },
    );
    for (settings, expected, what) in [
        (rigid, [0.0, w, 0.0], "rigid"),
        (soft, [0.0, w * cos, -w * sin], "soft"),
    ] {
        let position = settled(GRAVITY, &settings, Vec3::ZERO, |c| {
            c.total_lambda_position()
        });
        assert_components(&components(position), &expected, w, what);
        assert_close(length(position), w, &format!("{what} length"));
    }
}

#[test]
fn six_dof_reads_per_axis_parts_and_motors() {
    use SixDofConstraintAxis::*;
    let w = weight(MASS, DT);
    let couple = TORQUE * DT;
    let joint = SixDofConstraintSettings::new(CENTRE, X, Y);
    let fixed = |s: SixDofConstraintSettings, axes: &[SixDofConstraintAxis]| {
        axes.iter()
            .fold(s, |s, &axis| s.axis(axis, SixDofAxis::Fixed))
    };

    // X and Z fixed, Y limited: three translation parts in constraint-axis order.
    let limited_y = fixed(joint.clone(), &[TranslationX, TranslationZ]).axis(
        TranslationY,
        SixDofAxis::Limited {
            min: -0.1,
            max: 0.1,
        },
    );
    let position = settled(GRAVITY, &limited_y, Vec3::ZERO, |c| {
        c.total_lambda_position()
    });
    assert_components(
        &components(position).map(f32::abs),
        &[0.0, w, 0.0],
        w,
        "translation",
    );

    // Rotation X limited, Y and Z fixed: twist, swing Y and swing Z parts.
    let translation = [TranslationX, TranslationY, TranslationZ];
    let limited_x = fixed(joint.clone(), &translation).axis(
        RotationX,
        SixDofAxis::Limited {
            min: -0.3,
            max: 0.3,
        },
    );
    let limited_x = fixed(limited_x, &[RotationY, RotationZ]);
    let rotation = settled(Vec3::ZERO, &limited_x, along(X, TORQUE), |c| {
        c.total_lambda_rotation()
    });
    assert_components(
        &components(rotation).map(f32::abs),
        &[couple, 0.0, 0.0],
        couple,
        "twist",
    );

    // Everything fixed: a world-space angular impulse on body 2.
    let welded = fixed(joint.clone(), &translation);
    let welded = fixed(welded, &[RotationX, RotationY, RotationZ]);
    let rotation = settled(Vec3::ZERO, &welded, along(Z, TORQUE), |c| {
        c.total_lambda_rotation()
    });
    assert_components(
        &components(rotation),
        &[0.0, 0.0, -couple],
        couple,
        "fixed rotation",
    );

    // Every axis free with velocity motors at 0 holding a force and a couple.
    let (mut world, cube, id) = rig(Vec3::ZERO, CENTRE, &joint);
    let mut motors = world.constraint_mut(id).unwrap();
    for axis in [
        TranslationX,
        TranslationY,
        TranslationZ,
        RotationX,
        RotationY,
        RotationZ,
    ] {
        motors.set_motor_state(axis, MotorState::Velocity);
    }
    let force = Vec3::new(10.0, 20.0, 30.0);
    let torque = Vec3::new(1.0, 2.0, 3.0);
    run_loaded(&mut world, DT, SETTLE, cube, force, torque);
    let c = world.constraint(id).unwrap();
    let held = |v: Vec3| components(v).map(|f| -f * DT);
    assert_components(
        &components(c.total_lambda_motor_translation()),
        &held(force),
        DT,
        "translation motors",
    );
    assert_components(
        &components(c.total_lambda_motor_rotation()),
        &held(torque),
        DT,
        "rotation motors",
    );
}

/// Two 1 kg cubes 2 m apart along X, each on a hinge about Z to its own anchor, and the anchors.
fn hinged_pair(world: &mut PhysicsWorld) -> ([BodyId; 2], [ConstraintId<HingeConstraint>; 2]) {
    let centres = [CENTRE, at([2.0, 0.0, 0.0])];
    let bodies = centres.map(|c| cube(world, c, 1.0));
    let hinges = [0, 1].map(|i| {
        let anchor = anchor(world, shifted(centres[i], [0.0, 1.0, 0.0]));
        let hinge = HingeConstraintSettings::new(centres[i], Z, X);
        world.create_constraint(anchor, bodies[i], &hinge).unwrap()
    });
    (bodies, hinges)
}

#[test]
fn couplings_read_the_load_they_carry() {
    // Gear: body 1 held by its hinge's velocity motor, a couple of 1 N·m on body 2. Jolt applies
    // the gear impulse to body 2 as is (not `ratio` times it), so it reads the couple at both
    // ratios.
    let couple = 1.0 * DT;
    for ratio in [1.0, 2.0] {
        let mut world = world(Vec3::ZERO, 1);
        let ([disc1, disc2], [hinge1, _]) = hinged_pair(&mut world);
        world
            .constraint_mut(hinge1)
            .unwrap()
            .set_motor_state(MotorState::Velocity);
        let gear = GearConstraintSettings::new(Z, Z, ratio);
        let id = world.create_constraint(disc1, disc2, &gear).unwrap();
        run_loaded(&mut world, DT, SETTLE, disc2, Vec3::ZERO, Z);
        let lambda = world.constraint(id).unwrap().total_lambda();
        assert_close(lambda.abs(), couple, &format!("gear ratio {ratio}"));
    }

    // Rack and pinion: the pinion held by its hinge's velocity motor, a force F along +X on the
    // rack. The rack gets -ratio times the readout and needs -F dt, so it reads F dt / ratio.
    for ratio in [2.0, -2.0] {
        let mut world = world(Vec3::ZERO, 1);
        let pinion = cube(&mut world, CENTRE, 1.0);
        let rack_at = at([2.0, 0.0, 0.0]);
        let rack = cube(&mut world, rack_at, 1.0);
        let anchor = anchor(&mut world, at([1.0, 1.0, 0.0]));
        let hinge = world
            .create_constraint(anchor, pinion, &HingeConstraintSettings::new(CENTRE, Z, X))
            .unwrap();
        world
            .constraint_mut(hinge)
            .unwrap()
            .set_motor_state(MotorState::Velocity);
        world
            .create_constraint(anchor, rack, &SliderConstraintSettings::new(rack_at, X, Y))
            .unwrap();
        let coupling = RackAndPinionConstraintSettings::new(Z, X, ratio);
        let id = world.create_constraint(pinion, rack, &coupling).unwrap();
        run_loaded(&mut world, DT, SETTLE, rack, along(X, FORCE), Vec3::ZERO);
        let lambda = world.constraint(id).unwrap().total_lambda();
        assert_close(lambda, FORCE * DT / ratio, &format!("rack ratio {ratio}"));
    }
}

/// A straight path along X through `CENTRE` at fraction 1, from 1 m before it to 1 m after it,
/// in the plane of `normal`, fixed to an anchor 1 m above the centre.
fn straight_path(normal: Vec3) -> PathConstraintSettings {
    let point = |x: f32| HermitePathPoint {
        position: Vec3::new(x, 0.0, 0.0),
        tangent: X,
    };
    let path = HermitePath::new(normal, vec![point(0.0), point(1.0), point(2.0)], false).unwrap();
    PathConstraintSettings::new(path)
        .path_position(Vec3::new(-1.0, -1.0, 0.0))
        .path_fraction(1.0)
}

#[test]
fn path_reads_its_parts() {
    let w = weight(MASS, DT);
    // Jolt's path binormal is normal × tangent and its normal tangent × binormal: with normal Z
    // the weight is along the binormal (+Y), with normal Y along the normal (+Y).
    for (normal, expected) in [(Z, [0.0, w]), (Y, [w, 0.0])] {
        let position = settled(GRAVITY, &straight_path(normal), Vec3::ZERO, |c| {
            c.total_lambda_position()
        });
        assert_components(&position, &expected, w, "position");
    }

    // A force along the tangent, held by the far end (limits allow only impulses against +X)
    // and then by a velocity motor at 0.
    let pushed = FORCE * DT;
    let (mut world, cube, id) = rig(Vec3::ZERO, CENTRE, &straight_path(Z));
    run_loaded(
        &mut world,
        DT,
        2 * SETTLE,
        cube,
        along(X, FORCE),
        Vec3::ZERO,
    );
    let c = world.constraint(id).unwrap();
    assert!(
        (c.path_fraction() - 2.0).abs() < 1e-2,
        "{}",
        c.path_fraction()
    );
    assert_close(c.total_lambda_position_limits(), -pushed, "end of the path");
    assert_eq!(c.total_lambda_motor(), 0.0);
    let (mut world, cube, id) = rig(Vec3::ZERO, CENTRE, &straight_path(Z));
    world
        .constraint_mut(id)
        .unwrap()
        .set_motor_state(MotorState::Velocity);
    run_loaded(&mut world, DT, SETTLE, cube, along(X, FORCE), Vec3::ZERO);
    let c = world.constraint(id).unwrap();
    assert_close(c.total_lambda_motor(), -pushed, "motor");
    assert_eq!(c.total_lambda_position_limits(), 0.0);

    // Turning only about the normal Z: Jolt's hinge axes are b2 × a1 = X and c2 × a1 = Y
    // (b2 = Z.GetNormalizedPerpendicular() = Y, c2 = Z × Y = -X), so a couple about X reads in
    // the first component. Fully constrained: a world-space angular impulse.
    let couple = TORQUE * DT;
    let hinged =
        straight_path(Z).rotation_constraint(PathRotationConstraint::ConstrainAroundNormal);
    let (hinge, full) = settled(Vec3::ZERO, &hinged, along(X, TORQUE), |c| {
        (c.total_lambda_rotation_hinge(), c.total_lambda_rotation())
    });
    assert_components(&hinge, &[-couple, 0.0], couple, "hinge mode");
    assert_eq!(full, Vec3::ZERO);
    let welded = straight_path(Z).rotation_constraint(PathRotationConstraint::FullyConstrained);
    let (hinge, full) = settled(Vec3::ZERO, &welded, along(Z, TORQUE), |c| {
        (c.total_lambda_rotation_hinge(), c.total_lambda_rotation())
    });
    assert_components(
        &components(full),
        &[0.0, 0.0, -couple],
        couple,
        "fully constrained",
    );
    assert_eq!(hinge, [0.0; 2]);
}

#[test]
fn a_disabled_constraint_keeps_its_last_readout() {
    let weld = FixedConstraintSettings::new(at([0.0, HALF, 0.0]), X, Y);
    let (mut world, cube, id) = rig(GRAVITY, CENTRE, &weld);
    run(&mut world, DT, SETTLE);
    let loaded = readout_bits(&world, id.into());
    let height = world.body(cube).unwrap().position().y;
    world.constraint_mut(id).unwrap().set_enabled(false);
    assert!(!world.constraint(id).unwrap().is_enabled());
    assert_eq!(readout_bits(&world, id.into()), loaded);
    run(&mut world, DT, 1);
    assert_eq!(readout_bits(&world, id.into()), loaded);
    assert!(
        world.body(cube).unwrap().position().y < height,
        "the cube falls"
    );

    world.constraint_mut(id).unwrap().set_enabled(true);
    run(&mut world, DT, SETTLE);
    let position = world.constraint(id).unwrap().total_lambda_position();
    assert_close(position.y, weight(MASS, DT), "held again");
}

#[test]
fn a_sleeping_constraint_keeps_its_last_readout() {
    let mut world = world(GRAVITY, 1);
    let anchor = anchor(&mut world, at([0.0, 1.0, 0.0]));
    let cube = sleepy_cube(&mut world, CENTRE, MASS);
    let weld = FixedConstraintSettings::new(at([0.0, HALF, 0.0]), X, Y);
    let id = world.create_constraint(anchor, cube, &weld).unwrap();
    let tick = (0..600)
        .find(|_| {
            run(&mut world, DT, 1);
            world.body(cube).unwrap().is_sleeping()
        })
        .expect("the welded cube falls asleep within 600 ticks");
    // The step that put the cube to sleep still solved the weld.
    let asleep = readout_bits(&world, id.into());
    assert_close(f32::from_bits(asleep[1]), weight(MASS, DT), "asleep");
    run(&mut world, DT, 30);
    assert!(world.body(cube).unwrap().is_sleeping());
    assert_eq!(
        readout_bits(&world, id.into()),
        asleep,
        "after falling asleep at tick {tick}"
    );
}
