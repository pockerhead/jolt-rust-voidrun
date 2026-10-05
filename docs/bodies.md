# Body controls

Rigid bodies are created from a `Shape` and `BodySettings` and changed through `BodyMut`
(`PhysicsWorld::body_mut`). This guide covers the controls beyond poses, velocities and forces:
impulses, kinematic moves, waking and sleeping, sensors, user data, locked axes, and changing a
body's shape or motion type. Every call checks its input first; a refused call changes nothing.

## Impulses and forces

A force (`add_force`) acts over the next step and is cleared after it. An impulse changes the
velocity at once: `add_impulse` adds `impulse / mass`, `add_angular_impulse` the inverse inertia
times the angular impulse, and `add_impulse_at_point` both, for an impulse at a world point. Each
wakes the body. One impulse may change the velocity by at most `limits::MAX_VELOCITY_CHANGE` (twice
the speed limit, enough to reverse a body at full speed) and the angular velocity by at most
`limits::MAX_ANGULAR_VELOCITY_CHANGE`; Jolt then clamps the speed. Static and kinematic bodies
ignore impulses, as they ignore forces.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default().gravity(Vec3::ZERO))?;
    let ball = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic().mass(2.0))?;

    // A kick of 4 N·s on 2 kg: 2 m/s at once.
    world.body_mut(ball)?.add_impulse(Vec3::new(4.0, 0.0, 0.0))?;
    assert_eq!(world.body(ball)?.linear_velocity(), Vec3::new(2.0, 0.0, 0.0));
    assert!(world.step(1.0 / 60.0)?.is_complete());
    Ok(())
}
```

## Kinematic platforms

`move_kinematic(position, rotation, dt)` sets the velocities that carry a kinematic body toward a
pose over the next step of `dt` seconds, so bodies resting on it ride along. The velocity stays
after the step: call it every step the platform should move, and set a zero velocity to stop it.
Jolt turns with a small-angle approximation, so the pose is approached rather than met exactly. The
implied velocities must stay within the speed limits.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let deck = Shape::new_box(Vec3::new(2.0, 0.2, 2.0))?;
    let platform = world.create_body(&deck, &BodySettings::new_kinematic())?;
    let crate_shape = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let cargo = world.create_body(
        &crate_shape,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 0.7, 0.0)),
    )?;

    let dt = 1.0 / 60.0;
    for tick in 1..=60 {
        let target = RVec3::new(tick as Real / 60.0, 0.0, 0.0);
        world.body_mut(platform)?.move_kinematic(target, Quat::IDENTITY, dt)?;
        assert!(world.step(dt)?.is_complete());
    }
    assert!(world.body(cargo)?.position().x > 0.5);
    Ok(())
}
```

## Waking and sleeping

`activate` wakes a body and `deactivate` puts it to sleep now, zeroing its velocities. With
`EventSettings::body_activation` both report `ActivationEvent`s for bodies that change state.
`PhysicsWorld::activate_bodies_in_box` wakes every body whose exact bounds overlap a box, in body-id
order, so the result does not depend on the broad phase's history. A restore puts bodies to sleep or
wakes them without events.

## Sensors

`BodySettings::sensor(true)` makes a trigger: Jolt reports its contacts and resolves none of them.
The contact events carry `ContactSettings::is_sensor()`; [events.md](events.md#sensors) lists which
bodies a sensor detects and how sleep, characters, vehicles and soft bodies treat it.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.set_event_settings(EventSettings::default().contacts(true));
    let zone = Shape::new_box(Vec3::new(1.0, 1.0, 1.0))?;
    let trigger = world.create_body(&zone, &BodySettings::new_static().sensor(true))?;
    let ball = world.create_body(
        &Shape::new_sphere(0.3)?,
        &BodySettings::new_dynamic().position(RVec3::new(0.0, 3.0, 0.0)),
    )?;

    let mut entered = false;
    for _ in 0..60 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
        for event in world.take_events().contacts {
            if let ContactEvent::Added { manifold, settings } = event {
                let bodies = [manifold.pair.body1, manifold.pair.body2];
                entered |= settings.is_sensor() && bodies.contains(&trigger);
            }
        }
    }
    assert!(entered);
    // The sensor did not stop the ball.
    assert!(world.body(ball)?.position().y < -1.0);
    Ok(())
}
```

## User data

`BodySettings::user_data` stores a `u64` of the caller's in the body, for example the key of its
entity in an ECS, and `BodyRef::user_data` reads it back. It is fixed at creation, so no restore can
change it.

## Locked axes

`BodySettings::allowed_dofs` limits a body to some of its six degrees of freedom, along and about
the world axes: `AllowedDofs::PLANE_2D` keeps a body in the XY plane, turning about Z. A value must
keep a translation axis. Constraints, vehicles, ragdolls and a rotating rebase refuse such bodies,
because their checks assume an unmasked body.

## Shape and motion type changes

`BodyMut::set_shape` gives a body a new shape and the mass properties a body created with it would
have; it wakes the bodies inside the box that encloses the old and new bounds, which includes
bodies between the two shapes when they lie apart. `BodyMut::set_motion_type` switches a body
between static, kinematic and dynamic. A body created static changes only when created with
`BodySettings::allow_dynamic_or_kinematic(true)`. Both refuse bodies that a character, vehicle,
ragdoll or constraint holds.

Jolt saves neither shapes nor motion types, so each successful change makes earlier `WorldState`s
unrestorable ([state.md](state.md)): a rollback cannot cross it.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(10.0, 0.5, 10.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)))?;
    let block = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
    let hanging = world.create_body(
        &block,
        &BodySettings::new_static()
            .position(RVec3::new(0.0, 3.0, 0.0))
            .object_layer(ObjectLayer::MOVING)
            .allow_dynamic_or_kinematic(true),
    )?;

    let saved = world.save_state();
    world
        .body_mut(hanging)?
        .set_motion_type(MotionType::Dynamic, Activation::Activate)?;
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
    for _ in 0..90 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    assert!(world.body(hanging)?.position().y < 1.0);
    Ok(())
}
```

## Fixed at creation

The sensor flag, user data, locked axes and movement capability have no setters, so a rollback
never has to restore them. Each of these programs fails to compile only because the setter does not
exist:

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_is_sensor(true);
    Ok(())
}
```

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_user_data(7);
    Ok(())
}
```

```rust,compile_fail,E0599
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let id = world.create_body(&Shape::new_sphere(0.5)?, &BodySettings::new_dynamic())?;
    world.body_mut(id)?.set_allowed_dofs(AllowedDofs::PLANE_2D);
    Ok(())
}
```
