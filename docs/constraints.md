# Constraints

A constraint joins two bodies of a world: a hinge for a door, a slider for a piston, a gear between
two wheels. `PhysicsWorld::create_constraint(body1, body2, &settings)` creates one from a settings
value and returns a `ConstraintId<K>` typed by its kind, which selects the motor, target, limit and
readout methods of `ConstraintRef<K>` (`world.constraint(id)`) and `ConstraintMut<K>`
(`world.constraint_mut(id)`). The world owns the constraint until `remove_constraint` or until the
world is dropped.

## Kinds

| Settings | Jolt constraint | What it does |
|---|---|---|
| `FixedConstraintSettings` | `FixedConstraint` | welds two bodies |
| `PointConstraintSettings` | `PointConstraint` | a ball joint |
| `DistanceConstraintSettings` | `DistanceConstraint` | keeps two points at a distance or within a range, optionally as a spring |
| `HingeConstraintSettings` | `HingeConstraint` | one rotation axis, with limits, a motor and friction |
| `SliderConstraintSettings` | `SliderConstraint` | one translation axis, with limits, a motor and friction |
| `ConeConstraintSettings` | `ConeConstraint` | a ball joint whose twist axis stays in a cone |
| `SwingTwistConstraintSettings` | `SwingTwistConstraint` | swing and twist limits and motors, for shoulders and necks |
| `SixDofConstraintSettings` | `SixDOFConstraint` | each of the six axes free, limited or fixed, with motors and springs |
| `GearConstraintSettings` | `GearConstraint` | couples the rotation of two hinged bodies |
| `RackAndPinionConstraintSettings` | `RackAndPinionConstraint` | couples a hinged body's rotation to a sliding body's translation |
| `PulleyConstraintSettings` | `PulleyConstraint` | a rope over two fixed points between two bodies |
| `PathConstraintSettings` | `PathConstraint` | keeps a body on a `HermitePath`, with a motor along it |

The swing-twist, hinge and six-DOF settings also serve as ragdoll joints (`RagdollJoint`). Angles
are in radians, torques in N·m, forces in N; the defaults are Jolt's.

## Two gears

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);

    let anchor = Shape::new_box(Vec3::new(0.1, 0.1, 0.1))?;
    let base = world.create_body(
        &anchor,
        &BodySettings::new_static().position(RVec3::new(0.0, -10.0, 0.0)),
    )?;

    // Two discs on hinges about z, held by the static base.
    let disc = Shape::new_box(Vec3::new(0.5, 0.5, 0.1))?;
    let mut hinged_disc = |at: Real| -> Result<_, Box<dyn std::error::Error>> {
        let centre = RVec3::new(at, 0.0, 0.0);
        let body = world.create_body(&disc, &BodySettings::new_dynamic().position(centre))?;
        let hinge_settings = HingeConstraintSettings::new(centre, z, x);
        let hinge = world.create_constraint(base, body, &hinge_settings)?;
        Ok((body, hinge))
    };
    let (disc1, hinge1) = hinged_disc(0.0)?;
    let (disc2, hinge2) = hinged_disc(3.0)?;

    // Gear 2 turns half as fast as gear 1, the other way. The hinges let Jolt correct drift.
    let gear = GearConstraintSettings::new(z, z, 2.0).hinges(hinge1, hinge2);
    world.create_constraint(disc1, disc2, &gear)?;

    // Drive gear 1 at 2 rad/s with the hinge's velocity motor.
    let mut motor = world.constraint_mut(hinge1)?;
    motor.set_target_angular_velocity(2.0)?;
    motor.set_motor_state(MotorState::Velocity);
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let spin = world.body(disc2)?.angular_velocity().z;
    assert!((spin + 1.0).abs() < 0.02);
    Ok(())
}
```

## Frames

Each constraint attaches a frame to each body: a point and two perpendicular unit axes. With
`ConstraintSpace::WorldSpace` (the default) the frames are given in world space at the moment the
constraint is created, and Jolt turns them into each body's own frame then. With
`ConstraintSpace::LocalToBodyCom` they are relative to each body's centre of mass. In a constraint's
frame X is the twist or hinge axis, and Y and Z are the swing axes.

## Bodies

- Both bodies belong to the world and are different. Neither may be a character's inner body, a
  ragdoll part or a soft body (`ConstraintError::Body`).
- Except for a gear, a rack and pinion and a pulley, one body may be static or kinematic, which
  anchors the constraint to the world or to the kinematic body. Those three need two dynamic bodies
  (`ConstraintError::NotDynamic`): Jolt 5.6's solver parts for them read both bodies' motion
  properties without checking, so a static body breaks them and a kinematic one gets pushed. A
  pulley's fixed points already anchor its rope.
- Jolt still lets the two bodies collide where their shapes touch; leave a gap, or put the bodies in
  object layers that do not collide.
- While a constraint exists, `remove_body` refuses its bodies (`BodyError::UsedByConstraint`).
  `constraints_of_body` lists the constraints that hold a body.
- A new constraint, every setter and `remove_constraint` wake the constraint's bodies that can move,
  so a sleeping body follows the change at the next step.

## Limits on what is accepted

The settings are checked before anything reaches Jolt, and a failing call changes nothing.

- **Lever arm.** Each point where a constraint holds a dynamic body must have a lever-arm ratio of
  at most `limits::MAX_LEVER_ARM_RATIO` (1000): its distance from the body's centre of mass,
  measured against the body's size. A door on a hinge at its edge, a weld at the surface of a part
  and a pendulum bob of radius `a` up to about `14 · a` from its pivot pass; a longer pendulum is a
  distance constraint whose points lie on the bodies ([limits.md](limits.md#lever-arm-ratio)).
- **Springs.** A spring given as frequency and damping becomes a stiffness and damping that grow
  with the bodies' effective mass; both are bounded ([limits.md](limits.md#springs)).
- **Gear ratios** lie within `1..=limits::MAX_GEAR_RATIO` (10), body 2 being the gear that turns
  slower; swap the bodies for a gear that speeds up, and chain gears for a larger reduction. Both
  bounds come from a defect in Jolt 5.6's `GearConstraintPart`, which applies the impulse to body 2
  without the ratio: below 1 the velocity error can grow without bound, and above 10 the gear needs
  ever more steps to restore its relation after a disturbance
  ([limits.md](limits.md#coupling-ratios)). For the same reason the torque passed to body 2 is not
  `ratio` times the torque on body 1; the rotation rates follow the ratio.
- **Rack and pinion ratios**, in radians per metre, have a magnitude within
  `1 / limits::MAX_RATIO..=limits::MAX_RATIO`. A negative ratio reverses the direction of the
  coupling.
- **Pulley ratios** are positive and within `1 / limits::MAX_RATIO..=limits::MAX_RATIO`.

## References between constraints

A gear can name the hinges that hold its two bodies (`GearConstraintSettings::hinges`), and a rack
and pinion the hinge and slider that hold its bodies
(`RackAndPinionConstraintSettings::constraints`). Jolt then corrects drift from their angle and
position; without them it couples the velocities only and the bodies slowly drift apart. Each
referenced constraint must hold the coupled body as its body 2, about the same axis direction. A
referenced hinge or slider cannot be removed while the coupling exists
(`ConstraintError::UsedByConstraint`); remove the coupling first.

A gear's drift correction uses hinge angles in `[-π, π]`, which is consistent across the wrap at ±π
only for an integer ratio; with another ratio Jolt corrects towards a wrong angle once gear 2 has
wrapped.

## Rebase, state and ids

- Constraint frames are relative to the bodies, so `rebase` needs to change nothing, except for
  pulleys, whose fixed points are world points: a rebase recreates each pulley in the new frame
  ([guide.md](guide.md#floating-origin)).
- A `WorldState` holds each constraint's enabled flag, warm start, motor states and targets (for a
  path also its motor settings and friction), but not the rest of its configuration: limits, motor
  settings, friction, distances, pulley lengths and the cone angle ([state.md](state.md)).
- Each world numbers its constraints from 1 in creation order and never reuses an id. Dropping a
  world removes its constraints first.

## Known instabilities

Jolt's solver can diverge for reasons the lever-arm ratio does not measure: a hinge whose pin lies
off the axis of a slender body, chains of hinges with non-parallel axes, a light body held by
several constraints, and a slider whose body 1 travels far along its axis.
[coverage.md](coverage.md#not-covered) describes the measured cases; no bound in `oxijolt::limits`
excludes them.
