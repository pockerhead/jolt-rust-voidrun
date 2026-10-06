# Vehicles

A vehicle is Jolt's `VehicleConstraint` attached to a dynamic body the caller created, its
chassis. oxijolt has three kinds, one per Jolt controller:

| Kind | Creator | Settings | Driver input | Jolt controller |
|---|---|---|---|---|
| `WheeledVehicle` | `PhysicsWorld::create_wheeled_vehicle` | `WheeledVehicleSettings` | `DriverInput` | `WheeledVehicleController` |
| `TrackedVehicle` | `PhysicsWorld::create_tracked_vehicle` | `TrackedVehicleSettings` | `TrackedDriverInput` | `TrackedVehicleController` |
| `Motorcycle` | `PhysicsWorld::create_motorcycle` | `MotorcycleSettings` | `DriverInput` | `MotorcycleController` |

The [guide](guide.md#vehicles) describes the wheeled vehicle and what all kinds share: the chassis,
the collision testers, gravity set by the caller, and the wheel readout. This page adds the tank and
the motorcycle.

## Ids

A creator returns a `VehicleId<K>` typed by the kind; `VehicleId` alone means
`VehicleId<WheeledVehicle>`. The kind selects the methods of `VehicleRef<K>` (`world.vehicle(id)`)
and `VehicleMut<K>` (`world.vehicle_mut(id)`): every kind reads its wheels, engine rpm, gear and
gravity, and sets gravity, the pitch and roll limit and the tester; the driver input and the
kind's own readout (tracks, lean) exist only for its kind, so passing a tank's input to a car does
not compile.

Ids count 1, 2, 3 per world across kinds. `vehicle_ids()` and `vehicle_of_body(body)` return
`AnyVehicleId`, which knows its `kind()` and gives the typed id back with `downcast::<K>()`;
`remove_vehicle` takes either. `VehicleError::NotFound` and `WrongWorld` carry an `AnyVehicleId`.

## A tank

A tracked vehicle runs on two tracks. Each `VehicleTrackSettings` owns its `TrackedWheelSettings`
and names its driven wheel by index among them; the vehicle numbers the left track's wheels first,
then the right track's. The engine drives each track through its differential ratio, and every
wheel turns with its track at the track speed over its own radius.

`TrackedDriverInput` sets the throttle, the brake, and a speed ratio per track in
`[-1, -1/limits::MAX_RATIO]` or `[1/limits::MAX_RATIO, 1]`. Equal ratios drive straight, unequal ones
turn, and opposite ratios turn the tank in place. A ratio of 0 is refused: Jolt divides by a sum
of ratio products when it keeps the two tracks in step, and equal ratios of 1e-30 make it NaN.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;

    let hull = Shape::new_box(Vec3::new(1.7, 0.5, 3.2))?;
    let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.5, 0.0))?;
    let chassis = world.create_body(
        &chassis_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 1.0, 0.0))
            .mass(4000.0)
            .allow_sleeping(false),
    )?;
    // Five wheels per track, each track driven at its rearmost wheel.
    let track = |x: f32| {
        let wheels = [2.4, 1.2, 0.0, -1.2, -2.4]
            .map(|z| TrackedWheelSettings::new(Vec3::new(x, -0.3, z)))
            .to_vec();
        VehicleTrackSettings::new(wheels, 4)
    };
    let settings =
        TrackedVehicleSettings::new(track(1.4), track(-1.4), VehicleCollisionTester::ray(ObjectLayer::MOVING));
    let tank = world.create_tracked_vehicle(chassis, &settings)?;

    // Opposite track ratios: the tank turns in place.
    let pivot = TrackedDriverInput { forward: 1.0, left_ratio: -1.0, right_ratio: 1.0, brake: 0.0 };
    world.vehicle_mut(tank)?.set_driver_input(pivot)?;
    for _ in 0..120 {
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }

    let vehicle = world.vehicle(tank)?;
    assert_eq!(vehicle.track_wheels(TrackSide::Right), 5..10);
    let [left, right] = vehicle.tracks();
    // The driven wheels as vehicle wheel indices.
    assert_eq!((left.driven_wheel, right.driven_wheel), (4, 9));
    assert!(left.speed < 0.0 && right.speed > 0.0);
    assert!(world.body(chassis)?.angular_velocity().y > 0.5);
    Ok(())
}
```

`TrackState` gives a track's driven wheel, its angular velocity and its speed (angular velocity
times the driven wheel's radius). The settings are checked before anything is created: both
tracks need a wheel and a driven wheel among them, an inertia within
`limits::MIN_TRACK_INERTIA..=MAX_TRACK_INERTIA`, at most `limits::MAX_TRACK_INERTIA_RATIO` times
the other track's ([limits](limits.md#track-inertia-ratio)), and a positive differential ratio;
the wheels follow the wheeled vehicle's wheel rules plus a friction of at most
`limits::MAX_FRICTION`, with every radius within a factor `limits::MAX_RATIO` of the driven
wheel's; and the track speeds the drivetrain can reach must keep every term of Jolt's tracked step
within the track drive envelope ([limits](limits.md#track-drive-envelope)).
`create_tracked_vehicle` also weighs the tracks against the chassis: for every wheel, the track's
inertia over the wheel's radius squared, times the chassis' largest inverse effective mass where
the wheel pushes it along its forward or tilted toward its up, must be at most
`limits::MAX_TRACK_MASS_RATIO` ([limits](limits.md#track-mass-ratio)). Jolt's `TankTest` tracks
have a ratio of 0.23 and are accepted up to 11 kg·m²; small wheels, light chassis and hulls with
little roll or pitch inertia next to the tracks' reach need lighter tracks.
`TrackedVehicleSettings::default_engine` and `default_transmission` are Jolt's tracked defaults,
which differ from the wheeled ones.

## A motorcycle

A motorcycle is a wheeled vehicle with exactly two wheels, apart along its forward, and a lean
controller: a spring and damper about the chassis' forward axis that tilt it toward a target lean,
the direction of the ground's push on both wheels. `MotorcycleSettings::new(vehicle)` wraps a
`WheeledVehicleSettings`; the driver input is the wheeled `DriverInput`. `MotorcycleSettings::bike` builds
the motorcycle of Jolt's `MotorcycleTest` sample, the one below, from its front wheel's position and
radius.

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    let floor = Shape::new_box(Vec3::new(100.0, 1.0, 100.0))?;
    world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;

    let frame = Shape::new_box(Vec3::new(0.2, 0.3, 0.4))?;
    let chassis_shape = Shape::new_offset_center_of_mass(&frame, Vec3::new(0.0, -0.3, 0.0))?;
    let chassis = world.create_body(
        &chassis_shape,
        &BodySettings::new_dynamic()
            .position(RVec3::new(0.0, 1.2, 0.0))
            .mass(240.0)
            .allow_sleeping(false),
    )?;
    let wheel = |z: f32| WheelSettings::new(Vec3::new(0.0, -0.27, z)).radius(0.31).width(0.05);
    let front = wheel(0.75).max_steer_angle(30.0_f32.to_radians());
    let rear = wheel(-0.75).max_steer_angle(0.0);
    let bike = WheeledVehicleSettings::new(
        vec![front, rear],
        // The rear wheel alone is driven.
        vec![VehicleDifferentialSettings::new(None, Some(1)).differential_ratio(4.8)],
        VehicleCollisionTester::cast_cylinder(ObjectLayer::MOVING),
    );
    let motorcycle = world.create_motorcycle(chassis, &MotorcycleSettings::new(bike))?;

    // Ride straight, then steer right: the motorcycle leans to the right.
    for tick in 0..300 {
        let right = if tick < 180 { 0.0 } else { 0.15 };
        let input = DriverInput { forward: 0.4, right, ..DriverInput::default() };
        world.vehicle_mut(motorcycle)?.set_driver_input(input)?;
        assert!(world.step(1.0 / 60.0)?.is_complete());
    }
    let lean = world.vehicle(motorcycle)?.lean();
    assert!(lean.angle > 0.05, "{lean:?}");
    Ok(())
}
```

- **Lean readout.** `VehicleRef::lean` gives the target lean (a world-space direction: the zero
  vector until the first step, the world up after a step with the controller off) and the lean
  angles of the target and of the chassis, in radians about the chassis' forward from the world up,
  positive to the right.
- **Both wheels down.** Jolt applies the lean torque only while both wheels touch the ground with
  a positive suspension impulse. A motorcycle in the air or on one wheel gets none, and does not
  right itself before it lands.
- **Switches fixed at creation.** `lean_controller(false)` lets the motorcycle fall over;
  `lean_steering_limit(false)` lets it steer as far as its front wheel allows at any speed. Jolt
  does not save these switches in its state, so they are settings, read back with
  `is_lean_controller_enabled` and `is_lean_steering_limit_enabled`.
- **Steering limit and gravity.** With the steering limit on, Jolt limits the steering angle by the
  lean the turn would need at the current speed, using the gravity the vehicle uses. It applies
  while the forward speed is above 1e-3 m/s and the wheel's steering axis tilts towards the
  vehicle's up. In zero gravity (world or `set_gravity`) the limit is 0: a moving motorcycle cannot
  steer, one at rest turns its front wheel fully.
- **Lean spring.** `max_lean_angle` is at most `MotorcycleSettings::MAX_LEAN_ANGLE` (80°). The
  spring constant and damping must keep the chassis' angular acceleration within
  `limits::MAX_ANGULAR_ACCELERATION`, which `create_motorcycle` checks against the chassis'
  inertia ([limits](limits.md#motorcycle-lean)): Jolt's defaults fit a chassis whose largest
  principal inverse inertia is below about 430 1/(kg·m²).
- **No integral term.** `lean_spring_integration_coefficient` must be 0, Jolt's default.
  Jolt's `SaveState` does not save the integrated lean angle, so with another value a restored
  state can replay differently (it did with 200 in a probe); `create_motorcycle` refuses it with
  `VehicleError::LeanSpringIntegrationNotSaved`.

## Rollback and rebase

A `WorldState` holds each vehicle's driver input, drivetrain and wheels, a tank's track speeds and
a motorcycle's target lean. A tank and a motorcycle replay bit for bit after a restore, from a save
before their first contact, in a turn and during a gear shift (`tests/vehicle_kinds_state.rs`). As
for wheeled vehicles, the world up is not saved: after a restore in zero gravity a vehicle keeps
the world up of the abandoned run, and a motorcycle uses it for its target lean even with the
pitch and roll limit off ([state](state.md)).

A rotating `PhysicsWorld::rebase` rotates each motorcycle's target lean with the gravity overrides
and tester ups. The first step after it computes the target lean from the wheel contacts of the
step before, still in the old frame, which gives a short transient: in
`rebase_rotates_the_target_lean` (25° about +Z) the chassis up differed from the unrebased ride by
at most 0.024, and by half of that a second later.
