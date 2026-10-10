//! Bodies for constraint impulse tests, a rig with a loaded constraint of every kind, and every
//! impulse readout of a constraint in a fixed order.

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
    fn typed<K: ConstraintKind>(world: &PhysicsWorld, id: AnyConstraintId) -> ConstraintRef<'_, K> {
        world.constraint(id.downcast::<K>().unwrap()).unwrap()
    }
    let v = |v: Vec3| [v.x, v.y, v.z];
    let mut values = Vec::new();
    match id.kind() {
        ConstraintType::Fixed => {
            let c = typed::<FixedConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(v(c.total_lambda_rotation()));
        }
        ConstraintType::Point => {
            let c = typed::<PointConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
        }
        ConstraintType::Distance => {
            let c = typed::<DistanceConstraint>(world, id);
            values.push(c.total_lambda_position());
        }
        ConstraintType::Hinge => {
            let c = typed::<HingeConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(c.total_lambda_rotation());
            values.push(c.total_lambda_motor());
            values.push(c.total_lambda_rotation_limits());
        }
        ConstraintType::Slider => {
            let c = typed::<SliderConstraint>(world, id);
            values.extend(c.total_lambda_position());
            values.push(c.total_lambda_motor());
            values.push(c.total_lambda_position_limits());
            values.extend(v(c.total_lambda_rotation()));
        }
        ConstraintType::Cone => {
            let c = typed::<ConeConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.push(c.total_lambda_rotation());
        }
        ConstraintType::SwingTwist => {
            let c = typed::<SwingTwistConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(c.total_lambda_limits());
            values.extend(v(c.total_lambda_motor()));
        }
        ConstraintType::SixDof => {
            let c = typed::<SixDofConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(v(c.total_lambda_rotation()));
            values.extend(v(c.total_lambda_motor_translation()));
            values.extend(v(c.total_lambda_motor_rotation()));
        }
        ConstraintType::Gear => values.push(typed::<GearConstraint>(world, id).total_lambda()),
        ConstraintType::RackAndPinion => {
            values.push(typed::<RackAndPinionConstraint>(world, id).total_lambda());
        }
        ConstraintType::Pulley => {
            values.push(typed::<PulleyConstraint>(world, id).total_lambda_position());
        }
        ConstraintType::Path => {
            let c = typed::<PathConstraint>(world, id);
            values.extend(c.total_lambda_position());
            values.push(c.total_lambda_position_limits());
            values.push(c.total_lambda_motor());
            values.extend(c.total_lambda_rotation_hinge());
            values.extend(v(c.total_lambda_rotation()));
        }
        kind => panic!("no readouts listed for {kind:?}"),
    }
    values
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

/// A cube on a pendulum of length [`PENDULUM`] below `pivot`, 0.3 rad off vertical.
fn bob(world: &mut PhysicsWorld, pivot: RVec3) -> BodyId {
    let d = down_at(0.3);
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
/// The loads: a weld holding a cube; a point pendulum started 0.3 rad off vertical; a rope
/// pendulum; cone, swing-twist and six-DOF pendulums that swing onto a 0.2 rad limit around an
/// axis 0.4 rad off vertical and rest there; a hinge arm turned by a velocity motor at 1 rad/s;
/// a slider driven by a force-limited velocity motor into its limit; a gear (ratio 2) from a
/// motor-driven hinge to a hinge with friction; a rack and pinion (ratio 2) from a motor-driven
/// pinion to a rack on a slider with friction; a pulley whose 2 kg crate hangs while the 3 kg one
/// rests on a ledge; a cube sliding down a sloped path into its end.
pub fn all_kinds_rig(world: &mut PhysicsWorld) -> Vec<AnyConstraintId> {
    let mut ids: Vec<AnyConstraintId> = Vec::new();

    let o = rig_origin(0);
    let (a, c) = (anchor_behind(world, o), cube(world, o, RIG_MASS));
    let weld = FixedConstraintSettings::new(shifted(o, [0.0, HALF, 0.0]), X, Y);
    ids.push(world.create_constraint(a, c, &weld).unwrap().into());

    let o = rig_origin(1);
    let (a, c) = (anchor_behind(world, o), bob(world, o));
    let point = PointConstraintSettings::new(o);
    ids.push(world.create_constraint(a, c, &point).unwrap().into());

    let o = rig_origin(2);
    let (a, c) = (anchor_behind(world, o), bob(world, o));
    let rope = DistanceConstraintSettings::new(o, world.body(c).unwrap().position());
    ids.push(world.create_constraint(a, c, &rope).unwrap().into());

    let o = rig_origin(3);
    let a = anchor_behind(world, o);
    let c = cube(world, shifted(o, [0.5, 0.0, 0.0]), RIG_MASS);
    let hinge = HingeConstraintSettings::new(o, Z, X);
    let hinge = world.create_constraint(a, c, &hinge).unwrap();
    drive_hinge(world, hinge, 1.0);
    ids.push(hinge.into());

    let o = rig_origin(4);
    let (a, c) = (anchor_behind(world, o), cube(world, o, RIG_MASS));
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
    let (a, c) = (anchor_behind(world, o), bob(world, o));
    let cone = ConeConstraintSettings::new(o, axis, 0.2);
    ids.push(world.create_constraint(a, c, &cone).unwrap().into());

    let o = rig_origin(6);
    let (a, c) = (anchor_behind(world, o), bob(world, o));
    let joint = SwingTwistConstraintSettings::new(o, axis, Z).half_cone_angles(0.2, 0.2);
    ids.push(world.create_constraint(a, c, &joint).unwrap().into());

    let o = rig_origin(7);
    let (a, c) = (anchor_behind(world, o), bob(world, o));
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
    );
    ids.push(world.create_constraint(a, c, &joint).unwrap().into());

    let o = rig_origin(8);
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

    let o = rig_origin(9);
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

    let o = rig_origin(10);
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

    let o = rig_origin(11);
    let (a, c) = (anchor_behind(world, o), cube(world, o, RIG_MASS));
    ids.push(
        world
            .create_constraint(a, c, &sloped_path())
            .unwrap()
            .into(),
    );
    ids
}

/// Turns on the velocity motor of `hinge` with the target `velocity`, rad/s.
fn drive_hinge(world: &mut PhysicsWorld, hinge: ConstraintId<HingeConstraint>, velocity: f32) {
    let mut motor = world.constraint_mut(hinge).unwrap();
    motor.set_motor_state(MotorState::Velocity);
    motor.set_target_angular_velocity(velocity).unwrap();
}

/// A straight path 3 m long, sloping down 0.3 m per metre along +X, in body 1's frame placed so
/// that its second point, fraction 1, is 1 m in front of body 1 along -Z, where
/// [`anchor_behind`] puts the body it carries.
fn sloped_path() -> PathConstraintSettings {
    let point = |t: f32| HermitePathPoint {
        position: Vec3::new(t - 1.0, 0.3 - 0.3 * t, 0.0),
        tangent: Vec3::new(1.0, -0.3, 0.0),
    };
    let path = HermitePath::new(Z, (0..4).map(|t| point(t as f32)).collect(), false).unwrap();
    PathConstraintSettings::new(path)
        .path_position(Vec3::new(0.0, 0.0, -1.0))
        .path_fraction(1.0)
}
