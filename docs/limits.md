# Limits

Derivations behind the magnitudes the safe API accepts. The audit table with one row per input
lives in the rustdoc of `joltphysics::limits`; this file holds the reasoning that does not fit
in a table row. Line numbers refer to the vendored Jolt 5.6 sources
(`crates/joltphysics-sys/vendor/JoltPhysics/Jolt/Physics/`).

## Contact settings

A `ContactListener` may change the settings Jolt resolves a contact with. Every setter of
`ContactSettings` and `SoftBodyContactSettings` checks its value and refuses one outside its
range with a `ContactSettingsError`, leaving the settings unchanged. The rules below that depend
on the contact (sensor bodies, the lever `R`) are checked again on the value a listener returns,
against the contact it was called for, because a listener can assign a value kept from another
contact; `docs/events.md` ("Panics") says what happens to a value that fails.

### Friction and restitution

Combined friction follows the body rule, `0..=limits::MAX_FRICTION`. Jolt's default combined
friction of two bodies at the body bound is `sqrt(MAX_FRICTION²) = MAX_FRICTION`, so a listener
cannot give a contact more friction than two bodies already can, and the body bound's probe
(`friction_at_the_bound_keeps_contacts_finite`) covers it. Combined restitution follows the
body rule `0..=1`.

### Inverse mass and inertia scales

Jolt documents the scales as "0 = infinite mass, 1 = use original mass, 2 = body has half the
mass" and multiplies them into the inverse mass and the inverse inertia of each body for the
contact (`ContactConstraintManager.cpp:930-949`); the soft body solver does the same for the
vertices and the other body (`SoftBodyMotionProperties.cpp:205-215`). Every bound this crate
derives for masses and inertia (`limits::MIN_MASS`, the inertia conditioning floor) assumes the
inverse mass and inertia a body has; a scale above 1 would raise them past what those bounds
were derived for. The API therefore accepts `0..=1`: a contact can make a body heavier or
immovable, never lighter. This is a policy of this API, not a Jolt limit.

### Sensor contacts

Jolt starts a contact's `mIsSensor` as `body1.IsSensor() || body2.IsSensor()` and asserts that a
callback does not turn a contact with a sensor body into an ordinary one
(`ContactConstraintManager.cpp:915`, `:1194`, `:1466`, "Sensors cannot be converted into regular
bodies by a contact callback!"). `ContactSettings::set_is_sensor(false)` is refused for such a
contact; making an ordinary contact a sensor contact is allowed. The soft body path has no such
assertion, so `SoftBodyContactSettings::set_is_sensor` takes either value.

### Surface velocity

Jolt applies the relative surface velocity at the friction point as `v + ω × r1`, `r1` the
friction point relative to body 1's centre of mass (`ContactConstraintManager.cpp:164-168`), and
feeds the result into the friction constraint as a target velocity. Bounding `v` by
`limits::MAX_LINEAR_VELOCITY` and `ω` by `limits::MAX_ANGULAR_VELOCITY` alone would still allow
`|ω| · |r1|` of about 47 rad/s times a lever of up to 2 km (`limits::MAX_SHAPE_EXTENT`), some
9e4 m/s. The setters therefore also require, in `f64`,

    |v| + |ω| · R <= limits::MAX_LINEAR_VELOCITY

with `R` the largest distance from body 1's centre of mass to a contact point of the manifold,
on either body, computed once per callback from the manifold the listener sees. The friction
point is an average of those points, so `|r1| <= R`, and `|v + ω × r1| <= |v| + |ω| · |r1|` keeps
the target speed within the linear velocity bound bodies have. Setting one of the two velocities
checks it together with the current value of the other.

The bound is checked by `surface_velocities_are_bounded_alone_and_together` (a lever of 2000 m:
`|ω|` of 0.25 rad/s fills the bound alone, and the next `f32` above is refused) and exercised by
`a_conveyor_moves_a_resting_cube` (2 m/s moves a resting cube along the floor).
