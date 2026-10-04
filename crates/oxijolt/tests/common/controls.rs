//! Helpers of the body control, sensor and body settings tests.

use oxijolt::*;

use super::{cube_shape, step};

/// A dynamic unit cube at `position` with `dofs`.
pub fn add_locked_cube(world: &mut PhysicsWorld, position: RVec3, dofs: AllowedDofs) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &BodySettings::new_dynamic()
                .position(position)
                .allowed_dofs(dofs),
        )
        .unwrap()
}

/// The pose and velocities of `id`.
pub fn motion(world: &PhysicsWorld, id: BodyId) -> (RVec3, Quat, Vec3, Vec3) {
    let body = world.body(id).unwrap();
    (
        body.position(),
        body.rotation(),
        body.linear_velocity(),
        body.angular_velocity(),
    )
}

/// A static sensor box of half extent `half` at `position`.
pub fn add_static_sensor(world: &mut PhysicsWorld, half: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            &Shape::new_box(Vec3::new(half, half, half)).unwrap(),
            &BodySettings::new_static().position(position).sensor(true),
        )
        .unwrap()
}

/// Whether `event` is between `a` and `b`, in either order.
pub fn is_between(event: &ContactEvent, a: BodyId, b: BodyId) -> bool {
    let pair = event.pair();
    [pair.body1, pair.body2] == [a, b] || [pair.body1, pair.body2] == [b, a]
}

/// The contact events between `a` and `b`, as `a` (added), `p` (persisted) or `r` (removed),
/// with the settings of the added and persisted ones.
pub fn sensor_contacts(
    events: &[ContactEvent],
    a: BodyId,
    b: BodyId,
) -> Vec<(char, Option<ContactSettings>)> {
    events
        .iter()
        .filter(|event| is_between(event, a, b))
        .map(|event| match event {
            ContactEvent::Added { settings, .. } => ('a', Some(*settings)),
            ContactEvent::Persisted { settings, .. } => ('p', Some(*settings)),
            ContactEvent::Removed(_) => ('r', None),
        })
        .collect()
}

/// The kinds of [`sensor_contacts`] as a string.
pub fn contact_kinds(events: &[ContactEvent], a: BodyId, b: BodyId) -> String {
    sensor_contacts(events, a, b)
        .iter()
        .map(|(kind, _)| kind)
        .collect()
}

/// Steps `ticks` times and returns every contact event.
pub fn contact_events(world: &mut PhysicsWorld, ticks: usize) -> Vec<ContactEvent> {
    let mut events = Vec::new();
    for _ in 0..ticks {
        step(world, 1);
        events.extend(world.take_events().contacts);
    }
    events
}

/// Steps until `body` sleeps, at most 600 ticks, and asserts that it does.
pub fn fall_asleep(world: &mut PhysicsWorld, body: BodyId) {
    for _ in 0..600 {
        if world.body(body).unwrap().is_sleeping() {
            return;
        }
        step(world, 1);
    }
    panic!("the body never fell asleep");
}

/// Whether `result` is a refusal for an invalid value.
pub fn invalid<T: std::fmt::Debug>(result: Result<T, BodyError>) -> bool {
    matches!(result, Err(BodyError::InvalidValue(_)))
}

/// The largest positive `f32` in `0..=high` that `accepts` accepts, by bisection on the bits;
/// `accepts` must accept 0 and be monotonic.
pub fn largest_accepted(high: f32, mut accepts: impl FnMut(f32) -> bool) -> f32 {
    let (mut low, mut high) = (0_u32, high.to_bits());
    assert!(accepts(0.0));
    while low < high {
        let middle = low + (high - low).div_ceil(2);
        if accepts(f32::from_bits(middle)) {
            low = middle;
        } else {
            high = middle - 1;
        }
    }
    f32::from_bits(low)
}

/// Asserts that `id`'s pose and velocities are finite.
pub fn assert_finite(world: &PhysicsWorld, id: BodyId) {
    let (p, q, v, w) = motion(world, id);
    let p: [Real; 3] = p.into();
    let values = [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z];
    assert!(p.iter().all(|c| c.is_finite()) && values.iter().all(|c| c.is_finite()));
}

/// A dynamic body of `shape` with `mass` that starts asleep, without gravity.
pub fn add_sleeping(world: &mut PhysicsWorld, shape: &Shape, mass: f32, position: RVec3) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_dynamic()
                .position(position)
                .mass(mass)
                .activation(Activation::DontActivate),
        )
        .unwrap()
}

/// A kinematic body of `shape` at `position`, awake or asleep as `activation` says.
pub fn add_kinematic(
    world: &mut PhysicsWorld,
    shape: &Shape,
    position: RVec3,
    activation: Activation,
) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_kinematic()
                .position(position)
                .activation(activation),
        )
        .unwrap()
}

/// The angle in radians between two rotations.
pub fn angle_between(a: Quat, b: Quat) -> f32 {
    let dot = (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
        .abs()
        .min(1.0);
    2.0 * dot.acos()
}

/// The distance between two points, in `f64`.
pub fn distance(a: RVec3, b: RVec3) -> f64 {
    let d = [a.x - b.x, a.y - b.y, a.z - b.z].map(f64::from);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}
