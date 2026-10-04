# Saving and restoring a world

`PhysicsWorld::save_state` takes a snapshot of a world's simulation state as a `WorldState`, and
`PhysicsWorld::restore_state` returns the world to it. That is what rollback netcode and replays
need: save at tick N, run on, restore, and the same calls give the same results bit for bit.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    let ball_shape = Shape::new_sphere(0.5)?;
    let ball = world.create_body(
        &ball_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 2.0, 0.0)),
    )?;

    let saved = world.save_state();
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let first_run = world.body(ball)?.position();

    world.restore_state(&saved)?;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert_eq!(world.body(ball)?.position(), first_run);
    Ok(())
}
```

## What a state holds

Jolt's saved state of the physics system (`PhysicsSystem::SaveState`):
- the previous step's delta time and the world's gravity;
- per body its pose, velocities, accumulated force and torque, sleep test data, whether it may sleep
  and whether it is awake; for a soft body its vertex positions and velocities and its bounds;
- the contact cache;
- every constraint's own state: its enabled flag, the solver parts' warm start, motor states and
  targets, and for path constraints also the motor settings and maximum friction;
- for vehicles the driver input and the engine, transmission and wheel state.

On top of that it holds every character's `CharacterState`.

`save_state_of(&bodies)` saves only the listed bodies; global state, contacts, constraints and
characters are saved whole. Restoring it leaves every other body as it is (except that a character's
inner body moves to the restored character's pose), so a replay from it is exact only when those
bodies did not change since the save, for example static bodies the caller never moved.

## Which world it restores into

A state restores only into the world that saved it (`StateError::WrongWorld`), and only while that
world's structure is unchanged (`StateError::WorldChanged`). The structure changes when a body,
character, vehicle, ragdoll or constraint is created or removed, when a ragdoll's motion type is
set, and when a rebase moves anything. Calls that fail their checks change nothing and keep states
restorable. In both refusals the world is unchanged.

Jolt saves neither which objects exist nor its body id allocator, so rolling back across a creation
or removal is not supported: even a body created and removed again leaves the next id different.
Within one structure, the same calls after a restore create bodies with the same ids as in the
original run.

`StateError::RestoreFailed` means Jolt or a character could not read the state. A state the world
accepted is not expected to cause it; if it happens, the world may be partly restored and should be
discarded.

## What is not saved

Configuration Jolt does not save stays as it is when a state is restored. A caller that changes it
during a run and rolls back must set it back and replay those calls, as Jolt's rollback
documentation asks for body friction. These setters change such configuration:
- hinge: `set_motor_settings`, `set_limits`, `set_limits_spring`, `set_max_friction_torque`;
- slider: `set_motor_settings`, `set_limits`, `set_limits_spring`, `set_max_friction_force`;
- distance: `set_distance`, `set_limits_spring`;
- pulley: `set_length`;
- cone: `set_half_cone_angle`;
- swing-twist: `set_swing_motor_settings`, `set_twist_motor_settings`, `set_max_friction_torque`;
- six-DOF: `set_motor_settings`;
- vehicle: `set_gravity`, `set_max_pitch_roll_angle`, `set_collision_tester`;
- soft body: `set_vertex_inverse_mass` (a restore keeps the inverse masses set last).

Body properties set at creation (shape, mass, friction, layers) are not saved either; the body
setters change only poses and velocities, which are saved. A target changed after a save, such as a
hinge's target angle, is part of the saved state and is undone.

A setting the caller applies again before every step, such as a vehicle's gravity on a planet,
replays exactly as long as that gravity is not zero.

A vehicle's world up is not saved, and Jolt has no setter for it, so a restore cannot put it back.
Each step sets the world up to the opposite of the gravity the vehicle uses (world gravity or
`VehicleMut::set_gravity`). In zero gravity a step keeps the world up it had. So after a restore
in zero gravity, the vehicle keeps the world up the abandoned run ended with. If that run had
gravity in another direction, a vehicle with a pitch and roll limit
(`VehicleSettings::max_pitch_roll_angle` below π) replays differently.

The wheel contacts a vehicle reports are empty right after a restore until the next step, because
Jolt clears a wheel's contact body on restore.

Events are not part of the state: `restore_state` neither clears nor rewinds the queue that
`take_events` drains, and a restore reports no activation changes. Take the events before a rollback
and drop those of the abandoned steps.

## The bytes

A `WorldState` gives neither its bytes nor `==`. Jolt writes fields it has never initialised into
its stream (a wheel's contact position, normal and lateral direction before the wheel's first
contact, `VehicleConstraint::SaveState`), so the stream may hold bytes without a defined value. A
restore copies them back as they were and Jolt uses them only for a wheel with a contact, so a
restore is not affected, but Rust may not read them as `u8`; the state keeps them as
`MaybeUninit<u8>`, which only Jolt copies. Compare what a world reports (poses, velocities,
character and vehicle state) instead. Loading a state into another world, or from disk, is not
supported.

## What the tests check

`tests/state.rs` saves a scene with stacks, a motor-driven hinge, a car, a walking character with an
inner body and a ragdoll, runs on, restores and replays every tick bit for bit, with 1 and 4
workers, in one process and across two. Further tests cover a detour with different inputs before
the restore, saved and unsaved configuration, structural changes, selected bodies and soft bodies.
