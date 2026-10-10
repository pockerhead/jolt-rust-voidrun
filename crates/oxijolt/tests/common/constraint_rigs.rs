//! Bodies for constraint impulse tests, a rig with a loaded constraint of every kind, and every
//! impulse readout of a constraint in a fixed order.

use std::collections::BTreeSet;

use oxijolt::*;

/// Half extent of a [`cube`], metres.
pub const HALF: f32 = 0.25;

/// A small static box at `position`, the world end of a constraint.
pub fn anchor(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// A dynamic cube of half extent [`HALF`] and `mass` kg centred at `centre`, without damping,
/// so a held cube carries exactly its load, and kept awake.
pub fn cube(world: &mut PhysicsWorld, centre: RVec3, mass: f32) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &cube_settings(centre, mass).allow_sleeping(false),
        )
        .unwrap()
}

/// A [`cube`] that may fall asleep.
pub fn sleepy_cube(world: &mut PhysicsWorld, centre: RVec3, mass: f32) -> BodyId {
    world
        .create_body(&cube_shape(), &cube_settings(centre, mass))
        .unwrap()
}

fn cube_shape() -> Shape {
    Shape::new_box(Vec3::new(HALF, HALF, HALF)).unwrap()
}

fn cube_settings(centre: RVec3, mass: f32) -> BodySettings {
    BodySettings::new_dynamic()
        .position(centre)
        .mass(mass)
        .linear_damping(0.0)
        .angular_damping(0.0)
}

/// Every `total_lambda*` readout of the constraint `id`, in the order its kind declares them;
/// vectors and arrays component by component.
pub fn readouts(world: &PhysicsWorld, id: AnyConstraintId) -> Vec<f32> {
    readout_parts(world, id)
        .into_iter()
        .flat_map(|(_, values)| values)
        .collect()
}

/// The readouts of the constraint `id` getter by getter, each with its name. A six-DOF getter
/// whose value is a world-space vector while all three of its axes are fixed is named with its
/// representation.
pub fn readout_parts(world: &PhysicsWorld, id: AnyConstraintId) -> Vec<(&'static str, Vec<f32>)> {
    fn typed<K: ConstraintKind>(world: &PhysicsWorld, id: AnyConstraintId) -> ConstraintRef<'_, K> {
        world.constraint(id.downcast::<K>().unwrap()).unwrap()
    }
    let v = |v: Vec3| vec![v.x, v.y, v.z];
    match id.kind() {
        ConstraintType::Fixed => {
            let c = typed::<FixedConstraint>(world, id);
            vec![
                ("position", v(c.total_lambda_position())),
                ("rotation", v(c.total_lambda_rotation())),
            ]
        }
        ConstraintType::Point => {
            let c = typed::<PointConstraint>(world, id);
            vec![("position", v(c.total_lambda_position()))]
        }
        ConstraintType::Distance => {
            let c = typed::<DistanceConstraint>(world, id);
            vec![("position", vec![c.total_lambda_position()])]
        }
        ConstraintType::Hinge => {
            let c = typed::<HingeConstraint>(world, id);
            vec![
                ("position", v(c.total_lambda_position())),
                ("rotation", c.total_lambda_rotation().to_vec()),
                ("motor", vec![c.total_lambda_motor()]),
                ("rotation_limits", vec![c.total_lambda_rotation_limits()]),
            ]
        }
        ConstraintType::Slider => {
            let c = typed::<SliderConstraint>(world, id);
            vec![
                ("position", c.total_lambda_position().to_vec()),
                ("motor", vec![c.total_lambda_motor()]),
                ("position_limits", vec![c.total_lambda_position_limits()]),
                ("rotation", v(c.total_lambda_rotation())),
            ]
        }
        ConstraintType::Cone => {
            let c = typed::<ConeConstraint>(world, id);
            vec![
                ("position", v(c.total_lambda_position())),
                ("rotation", vec![c.total_lambda_rotation()]),
            ]
        }
        ConstraintType::SwingTwist => {
            let c = typed::<SwingTwistConstraint>(world, id);
            vec![
                ("position", v(c.total_lambda_position())),
                ("limits", c.total_lambda_limits().to_vec()),
                ("motor", v(c.total_lambda_motor())),
            ]
        }
        ConstraintType::SixDof => {
            use SixDofConstraintAxis::*;
            let c = typed::<SixDofConstraint>(world, id);
            let fixed = |axes: [SixDofConstraintAxis; 3]| {
                axes.into_iter()
                    .all(|axis| matches!(c.limits(axis), Some((min, max)) if min >= max))
            };
            let position = if fixed([TranslationX, TranslationY, TranslationZ]) {
                "position (world vector)"
            } else {
                "position (per axis)"
            };
            let rotation = if fixed([RotationX, RotationY, RotationZ]) {
                "rotation (world vector)"
            } else {
                "rotation (limit parts)"
            };
            vec![
                (position, v(c.total_lambda_position())),
                (rotation, v(c.total_lambda_rotation())),
                ("motor_translation", v(c.total_lambda_motor_translation())),
                ("motor_rotation", v(c.total_lambda_motor_rotation())),
            ]
        }
        ConstraintType::Gear => {
            vec![(
                "gear",
                vec![typed::<GearConstraint>(world, id).total_lambda()],
            )]
        }
        ConstraintType::RackAndPinion => {
            let c = typed::<RackAndPinionConstraint>(world, id);
            vec![("rack_and_pinion", vec![c.total_lambda()])]
        }
        ConstraintType::Pulley => {
            let c = typed::<PulleyConstraint>(world, id);
            vec![("position", vec![c.total_lambda_position()])]
        }
        ConstraintType::Path => {
            let c = typed::<PathConstraint>(world, id);
            vec![
                ("position", c.total_lambda_position().to_vec()),
                ("position_limits", vec![c.total_lambda_position_limits()]),
                ("motor", vec![c.total_lambda_motor()]),
                ("rotation_hinge", c.total_lambda_rotation_hinge().to_vec()),
                ("rotation", v(c.total_lambda_rotation())),
            ]
        }
        kind => panic!("no readouts listed for {kind:?}"),
    }
}

/// The number of distinct getters, six-DOF representations counted apart, of the constraint
/// kinds [`all_kinds_rig`] makes.
pub const READOUT_GETTERS: usize = 31;

/// Every getter, as "kind getter", that some constraint of `ids` has, and those of them that no
/// constraint of `ids` reads a nonzero value from.
pub fn readout_coverage(
    world: &PhysicsWorld,
    ids: &[AnyConstraintId],
) -> (BTreeSet<String>, BTreeSet<String>) {
    let mut all = BTreeSet::new();
    let mut loaded = BTreeSet::new();
    for &id in ids {
        for (getter, values) in readout_parts(world, id) {
            let name = format!("{:?} {getter}", id.kind());
            if values.iter().any(|&v| v != 0.0) {
                loaded.insert(name.clone());
            }
            all.insert(name);
        }
    }
    let unloaded = all.difference(&loaded).cloned().collect();
    (all, unloaded)
}

/// Panics unless `ids` has every getter of [`READOUT_GETTERS`] and each reads a load.
#[track_caller]
pub fn assert_every_getter_loaded(world: &PhysicsWorld, ids: &[AnyConstraintId], when: &str) {
    let (all, unloaded) = readout_coverage(world, ids);
    assert_eq!(all.len(), READOUT_GETTERS, "{when}: getters {all:?}");
    assert!(unloaded.is_empty(), "{when}: no load in {unloaded:?}");
}

/// Whether the constraint `id` is enabled.
pub fn is_enabled(world: &PhysicsWorld, id: AnyConstraintId) -> bool {
    fn typed<K: ConstraintKind>(world: &PhysicsWorld, id: AnyConstraintId) -> bool {
        world
            .constraint(id.downcast::<K>().unwrap())
            .unwrap()
            .is_enabled()
    }
    match id.kind() {
        ConstraintType::Fixed => typed::<FixedConstraint>(world, id),
        ConstraintType::Point => typed::<PointConstraint>(world, id),
        ConstraintType::Distance => typed::<DistanceConstraint>(world, id),
        ConstraintType::Hinge => typed::<HingeConstraint>(world, id),
        ConstraintType::Slider => typed::<SliderConstraint>(world, id),
        ConstraintType::Cone => typed::<ConeConstraint>(world, id),
        ConstraintType::SwingTwist => typed::<SwingTwistConstraint>(world, id),
        ConstraintType::SixDof => typed::<SixDofConstraint>(world, id),
        ConstraintType::Gear => typed::<GearConstraint>(world, id),
        ConstraintType::RackAndPinion => typed::<RackAndPinionConstraint>(world, id),
        ConstraintType::Pulley => typed::<PulleyConstraint>(world, id),
        ConstraintType::Path => typed::<PathConstraint>(world, id),
        kind => panic!("no constraint view listed for {kind:?}"),
    }
}

/// The bits of [`readouts`], for exact comparisons.
pub fn readout_bits(world: &PhysicsWorld, id: AnyConstraintId) -> Vec<u32> {
    readouts(world, id).into_iter().map(f32::to_bits).collect()
}

/// Where [`all_kinds_rig`] builds its first rig; the others follow every [`RIG_SPACING`] metres
/// along +X.
pub const RIG_ORIGIN: RVec3 = RVec3::new(0.0, 10.0, -20.0);
/// Distance between the rigs of [`all_kinds_rig`], metres: more than any of their bodies moves.
pub const RIG_SPACING: Real = 8.0;

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);
/// Mass of a rig's cube, kg.
const RIG_MASS: f32 = 2.0;
/// Length from a pendulum's pivot to its cube's centre, metres.
const PENDULUM: f32 = 0.75;

/// `p` moved by `d`.
pub fn shifted(p: RVec3, d: [f32; 3]) -> RVec3 {
    RVec3::new(p.x + d[0] as Real, p.y + d[1] as Real, p.z + d[2] as Real)
}

/// The unit vector `angle` radians from straight down, turned towards +X in the XY plane.
fn down_at(angle: f32) -> Vec3 {
    Vec3::new(angle.sin(), -angle.cos(), 0.0)
}

/// Angle from vertical, radians, at which a cone, swing-twist or six-DOF pendulum starts. Its
/// limit lets the cube turn 0.2 rad from where it starts, so it swings onto the limit 0.3 rad off
/// vertical and gravity presses it there.
const PRESSED: f32 = 0.5;

/// A cube on a pendulum of length [`PENDULUM`] below `pivot`, `angle` radians off vertical.
fn bob(world: &mut PhysicsWorld, pivot: RVec3, angle: f32) -> BodyId {
    let d = down_at(angle);
    cube(
        world,
        shifted(pivot, [d.x, d.y, d.z].map(|c| c * PENDULUM)),
        RIG_MASS,
    )
}

/// A static anchor 1 m behind `at` along +Z, out of reach of the rig's bodies.
fn anchor_behind(world: &mut PhysicsWorld, at: RVec3) -> BodyId {
    anchor(world, shifted(at, [0.0, 0.0, 1.0]))
}

/// The origin of rig `i` of [`all_kinds_rig`].
fn rig_origin(i: usize) -> RVec3 {
    RVec3::new(
        RIG_ORIGIN.x + RIG_SPACING * i as Real,
        RIG_ORIGIN.y,
        RIG_ORIGIN.z,
    )
}

/// One loaded constraint of every world constraint kind, each in its own rig at
/// [`RIG_ORIGIN`] plus a multiple of [`RIG_SPACING`] along X, for gravity along -Y. Returns
/// every constraint made in creation order, the hinges and slider that a gear and a rack and
/// pinion couple included.
///
/// Every getter of every kind, six-DOF in both of its representations, reads a load from tick 30
/// on: a weld beside a cube's centre holding its weight and moment; a point pendulum started
/// 0.3 rad off vertical; a rope pendulum; cone, swing-twist and six-DOF pendulums about an axis
/// 0.4 rad off vertical that swing onto their 0.2 rad limit and rest pressed onto it, the
/// swing-twist and six-DOF ones with friction; a six-DOF joint with its rotation fixed and one
/// translation axis limited with friction, holding a cube beside it; a hinge arm, 0.3 m along
/// the hinge axis from its pin, driven by a torque-limited velocity motor into its upper limit;
/// a cube beside a slider driven by a force-limited velocity motor into its limit; a gear
/// (ratio 2) from a motor-driven hinge to a hinge with friction; a rack and pinion (ratio 2) from
/// a motor-driven pinion to a rack on a slider with friction; a pulley whose 2 kg crate hangs
/// while the 3 kg one rests on a ledge; two cubes held beside the lower end of a sloped path with friction, one
/// turning only about the tangent, one with its rotation fixed.
pub fn all_kinds_rig(world: &mut PhysicsWorld) -> Vec<AnyConstraintId> {
    let mut ids: Vec<AnyConstraintId> = Vec::new();

    let o = rig_origin(0);
    let (a, c) = (anchor_behind(world, o), cube(world, o, RIG_MASS));
    let weld = FixedConstraintSettings::new(shifted(o, [0.4, HALF, 0.0]), X, Y);
    ids.push(world.create_constraint(a, c, &weld).unwrap().into());

    let o = rig_origin(1);
    let (a, c) = (anchor_behind(world, o), bob(world, o, 0.3));
    let point = PointConstraintSettings::new(o);
    ids.push(world.create_constraint(a, c, &point).unwrap().into());

    let o = rig_origin(2);
    let (a, c) = (anchor_behind(world, o), bob(world, o, 0.3));
    let rope = DistanceConstraintSettings::new(o, world.body(c).unwrap().position());
    ids.push(world.create_constraint(a, c, &rope).unwrap().into());

    let o = rig_origin(3);
    let a = anchor_behind(world, o);
    let c = cube(world, shifted(o, [0.5, 0.0, -0.3]), RIG_MASS);
    let hinge = HingeConstraintSettings::new(o, Z, X)
        .limits(-1.0, 0.3)
        .motor(MotorSettings::default().torque_limits(-20.0, 20.0));
    let hinge = world.create_constraint(a, c, &hinge).unwrap();
    drive_hinge(world, hinge, 1.0);
    ids.push(hinge.into());

    let o = rig_origin(4);
    let a = anchor_behind(world, o);
    let c = cube(world, shifted(o, [0.0, 0.0, -0.3]), RIG_MASS);
    let slider = SliderConstraintSettings::new(o, X, Y)
        .limits(-0.2, 0.2)
        .motor(MotorSettings::default().force_limits(-20.0, 20.0));
    let slider = world.create_constraint(a, c, &slider).unwrap();
    let mut motor = world.constraint_mut(slider).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    motor.set_target_velocity(1.0).unwrap();
    ids.push(slider.into());

    let axis = down_at(0.4);
    let across = Vec3::new(-axis.y, axis.x, 0.0);
    let o = rig_origin(5);
    let (a, c) = (anchor_behind(world, o), bob(world, o, PRESSED));
    let cone = ConeConstraintSettings::new(o, axis, 0.2);
    ids.push(world.create_constraint(a, c, &cone).unwrap().into());

    let o = rig_origin(6);
    let (a, c) = (anchor_behind(world, o), bob(world, o, PRESSED));
    let joint = SwingTwistConstraintSettings::new(o, axis, Z)
        .half_cone_angles(0.2, 0.2)
        .max_friction_torque(1.0);
    ids.push(world.create_constraint(a, c, &joint).unwrap().into());

    let o = rig_origin(7);
    let (a, c) = (anchor_behind(world, o), bob(world, o, PRESSED));
    let limited = SixDofAxis::Limited {
        min: -0.2,
        max: 0.2,
    };
    let joint = [
        (SixDofConstraintAxis::TranslationX, SixDofAxis::Fixed),
        (SixDofConstraintAxis::TranslationY, SixDofAxis::Fixed),
        (SixDofConstraintAxis::TranslationZ, SixDofAxis::Fixed),
        (SixDofConstraintAxis::RotationY, limited),
        (SixDofConstraintAxis::RotationZ, limited),
    ]
    .into_iter()
    .fold(
        SixDofConstraintSettings::new(o, axis, across),
        |s, (which, value)| s.axis(which, value),
    )
    .max_friction(SixDofConstraintAxis::RotationY, 1.0)
    .max_friction(SixDofConstraintAxis::RotationZ, 1.0);
    ids.push(world.create_constraint(a, c, &joint).unwrap().into());

    let o = rig_origin(8);
    let a = anchor_behind(world, o);
    let c = cube(world, shifted(o, [0.4, 0.0, 0.0]), RIG_MASS);
    let joint = [
        (SixDofConstraintAxis::TranslationX, SixDofAxis::Fixed),
        (SixDofConstraintAxis::TranslationY, limited_y()),
        (SixDofConstraintAxis::TranslationZ, SixDofAxis::Fixed),
        (SixDofConstraintAxis::RotationX, SixDofAxis::Fixed),
        (SixDofConstraintAxis::RotationY, SixDofAxis::Fixed),
        (SixDofConstraintAxis::RotationZ, SixDofAxis::Fixed),
    ]
    .into_iter()
    .fold(
        SixDofConstraintSettings::new(o, X, Y),
        |s, (which, value)| s.axis(which, value),
    )
    .max_friction(SixDofConstraintAxis::TranslationY, 5.0);
    ids.push(world.create_constraint(a, c, &joint).unwrap().into());

    let o = rig_origin(9);
    let centres = [o, shifted(o, [2.0, 0.0, 0.0])];
    let discs = centres.map(|centre| cube(world, centre, RIG_MASS));
    let hinges = [0, 1].map(|i| {
        let a = anchor_behind(world, centres[i]);
        let hinge = HingeConstraintSettings::new(centres[i], Z, X).max_friction_torque(1.0);
        world.create_constraint(a, discs[i], &hinge).unwrap()
    });
    drive_hinge(world, hinges[0], 1.0);
    ids.extend(hinges.map(AnyConstraintId::from));
    let gear = GearConstraintSettings::new(Z, Z, 2.0);
    ids.push(
        world
            .create_constraint(discs[0], discs[1], &gear)
            .unwrap()
            .into(),
    );

    let o = rig_origin(10);
    let rack_at = shifted(o, [3.0, 0.0, 0.0]);
    let (pinion, rack) = (cube(world, o, RIG_MASS), cube(world, rack_at, RIG_MASS));
    let a = anchor_behind(world, o);
    let hinge = HingeConstraintSettings::new(o, Z, X);
    let hinge = world.create_constraint(a, pinion, &hinge).unwrap();
    drive_hinge(world, hinge, 0.2);
    let a = anchor_behind(world, rack_at);
    let rail = SliderConstraintSettings::new(rack_at, X, Y).max_friction_force(5.0);
    let rail = world.create_constraint(a, rack, &rail).unwrap();
    ids.extend([AnyConstraintId::from(hinge), rail.into()]);
    let coupling = RackAndPinionConstraintSettings::new(Z, X, 2.0);
    ids.push(
        world
            .create_constraint(pinion, rack, &coupling)
            .unwrap()
            .into(),
    );

    let o = rig_origin(11);
    let other = shifted(o, [2.0, 0.0, 0.0]);
    let (crate1, crate2) = (cube(world, o, 2.0), cube(world, other, 3.0));
    let ledge = Shape::new_box(Vec3::new(0.4, 0.1, 0.4)).unwrap();
    let under = shifted(other, [0.0, -HALF - 0.1, 0.0]);
    world
        .create_body(&ledge, &BodySettings::new_static().position(under))
        .unwrap();
    let pulley = PulleyConstraintSettings::new(
        shifted(o, [0.0, HALF, 0.0]),
        shifted(o, [0.0, 2.0, 0.0]),
        shifted(other, [0.0, HALF, 0.0]),
        shifted(other, [0.0, 2.0, 0.0]),
    );
    ids.push(
        world
            .create_constraint(crate1, crate2, &pulley)
            .unwrap()
            .into(),
    );

    for (i, rotation) in [
        (12, PathRotationConstraint::ConstrainAroundTangent),
        (13, PathRotationConstraint::FullyConstrained),
    ] {
        let o = rig_origin(i);
        let (a, c) = (anchor_behind(world, o), cube(world, o, RIG_MASS));
        let path = sloped_path().rotation_constraint(rotation);
        ids.push(world.create_constraint(a, c, &path).unwrap().into());
    }
    ids
}

/// A six-DOF translation limit of 0.1 m either way.
fn limited_y() -> SixDofAxis {
    SixDofAxis::Limited {
        min: -0.1,
        max: 0.1,
    }
}

/// Turns on the velocity motor of `hinge` with the target `velocity`, rad/s.
fn drive_hinge(world: &mut PhysicsWorld, hinge: ConstraintId<HingeConstraint>, velocity: f32) {
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    motor.set_target_angular_velocity(velocity).unwrap();
}

/// A straight path 3 m long, sloping down 0.3 m per metre along +X, with a friction of 2 N, in
/// body 1's frame placed so that its lower end, fraction 3, is 0.4 m along +X from the point 1 m
/// in front of body 1 along -Z, where [`anchor_behind`] puts the body it carries: that body is
/// attached at the end, 0.4 m beside its centre.
fn sloped_path() -> PathConstraintSettings {
    let point = |t: f32| HermitePathPoint {
        position: Vec3::new(t - 2.6, 0.9 - 0.3 * t, 0.0),
        tangent: Vec3::new(1.0, -0.3, 0.0),
    };
    let path = HermitePath::new(Z, (0..4).map(|t| point(t as f32)).collect(), false).unwrap();
    PathConstraintSettings::new(path)
        .path_position(Vec3::new(0.0, 0.0, -1.0))
        .path_fraction(3.0)
        .max_friction_force(2.0)
}
