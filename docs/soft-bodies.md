# Soft bodies

A soft body is Jolt's particle body: vertices joined by constraints, with triangle faces that other
bodies collide with. It suits cloth, flags, ropes, pressurised balls and squishy solids. In oxijolt
a soft body is an ordinary body of its world: `PhysicsWorld::create_soft_body` returns a `BodyId`,
the body counts against the world's body limit, is removed with `remove_body`, saved by
`save_state`, moved by `rebase` and found by queries.

## A cloth on a pole

```rust
use oxijolt::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut world = PhysicsWorld::new(WorldSettings::default())?;

    // A 5 x 5 cloth of 2 m, pinned (kinematic) at its two far corners.
    let n: u32 = 5;
    let mut vertices = Vec::new();
    for z in 0..n {
        for x in 0..n {
            let position = Vec3::new(0.5 * x as f32 - 1.0, 0.0, 0.5 * z as f32 - 1.0);
            let pinned = z == 0 && (x == 0 || x == n - 1);
            vertices.push(if pinned {
                SoftBodyVertex::kinematic(position)
            } else {
                SoftBodyVertex { inverse_mass: 10.0, ..SoftBodyVertex::new(position) }
            });
        }
    }
    let mut faces = Vec::new();
    for z in 0..n - 1 {
        for x in 0..n - 1 {
            let i = z * n + x;
            faces.push([i, i + n, i + n + 1]);
            faces.push([i, i + n + 1, i + 1]);
        }
    }
    let cloth = SoftBodySharedSettings::builder(vertices, faces)
        .create_constraints(SoftBodyBendType::Distance, SoftBodyVertexAttributes::default())
        .build()?;
    let id = world.create_soft_body(
        &cloth,
        &SoftBodySettings::default().position(RVec3::new(0.0, 3.0, 0.0)),
    )?;

    for _ in 0..60 {
        world.step(1.0 / 60.0)?;
    }
    // The free edge hangs below the pinned one.
    let hanging = world.soft_body(id)?.vertices();
    assert!(hanging[(n * n - 1) as usize].position.y < 2.5);

    // Drop the cloth: unpin both corners with a vertex mass of 0.1 kg.
    let mut soft = world.soft_body_mut(id)?;
    soft.set_vertex_inverse_mass(0, 10.0)?;
    soft.set_vertex_inverse_mass(n - 1, 10.0)?;
    for _ in 0..30 {
        world.step(1.0 / 60.0)?;
    }
    assert!(world.soft_body(id)?.vertices()[0].position.y < 3.0);
    Ok(())
}
```

## Shared settings

`SoftBodySharedSettings::builder(vertices, faces)` collects the particles and faces, and `build`
checks them all before anything reaches Jolt. One built value may serve many bodies in many worlds
on many threads; each body keeps its own reference, so the settings may be dropped afterwards.

- **Vertices** (`SoftBodyVertex`) have a position relative to the body origin, a velocity and an
  inverse mass: 0 pins the vertex (kinematic, it moves only by its velocity), otherwise the inverse
  of a mass of 1 g to `limits::MAX_MASS`.
- **Faces** are triangles of vertex indices, wound counter-clockwise seen from outside. They are
  what other bodies collide with and what queries hit.
- **Generated constraints.** `create_constraints(bend_type, attributes)` makes Jolt's edge, shear,
  bend and long range attachment constraints from the faces, with the same
  `SoftBodyVertexAttributes` (compliances, LRA type and multiplier) for every vertex;
  `create_constraints_per_vertex` takes one set per vertex. Without either call, or explicit
  constraints, the particles are not tied together.
- **Explicit constraints.** `edge` (`SoftBodyEdge`), `dihedral_bend` (`SoftBodyDihedralBend`) and
  `volume` (`SoftBodyVolume`, one per tetrahedron of a solid body) add constraints to the generated
  ones.

`SoftBodySettings` places one body (position, rotation, object layer) and holds the per-body values:
iterations, damping, friction, restitution, gravity factor, pressure, vertex radius and the speed
limit. The defaults are Jolt's.

## Reading and writing vertices

`PhysicsWorld::soft_body(id)` gives every vertex in world space (`vertices`, or `vertices_into` to
reuse a buffer). `PhysicsWorld::soft_body_mut(id)` writes them; every setter checks its input first,
changes nothing when it fails, and wakes the body when it succeeds:
- `set_vertex_velocity` sets a world-space velocity;
- `set_vertex_inverse_mass` pins (0) or unpins a vertex; Jolt then recomputes the body's mass and
  inertia;
- `move_kinematic_vertex(index, target, dt)` moves a pinned vertex to a target over the next step.
  The velocity it sets stays, as for a kinematic body, until the vertex is moved again or stopped.

`BodyMut::add_force` works on a soft body: Jolt spreads the force over the vertices.

## What a soft body refuses

- `BodyMut::set_linear_velocity`, `set_angular_velocity` and `add_torque`: Jolt keeps velocities per
  vertex and ignores these for a soft body (`BodyError::SoftBody`).
- `BodyMut::add_force_at_point`: Jolt applies the force at the centre but also adds its torque,
  which a soft body never clears.
- Constraints and vehicles: Jolt's constraints cannot operate on soft bodies
  (`ConstraintError::Body(BodyError::SoftBody)`); pin a vertex and move it instead.
- Wheels look through soft bodies: Jolt's vehicle constraint solves the body under a wheel as a
  rigid body, so the wheel collision testers report only the rigid ground below.
- Soft bodies collide with rigid bodies, not with each other; Jolt does not implement that.

## Mass, inertia and pressure rules

These rules keep Jolt's mass, inertia and pressure arithmetic finite for the body as created;
[limits.md](limits.md) derives them. They are checked when the body is created, and vertex masses
and forces again when they change. Each rule refuses some bodies Jolt would simulate; the
workarounds are listed with them. A body that shrinks after creation, and solver settings that
diverge, are not covered ([coverage.md](coverage.md#not-covered)).

- **1 g per vertex.** A movable vertex weighs at least `limits::MIN_MASS` (1 g), and the movable
  vertices together at most `limits::MAX_MASS`. A light, finely divided body is refused at its real
  masses: a 1 m² cotton cloth of 0.15 kg at 21 × 21 vertices (0.34 g each), a 2 m flag of 0.4 kg at
  30 × 30, or a 57 g ball of 162 vertices. Use fewer vertices or give each at least 1 g. Gravity and
  damping move a vertex the same at any mass; pressure and added forces move a heavier vertex less,
  and a heavier body pushes the rigid bodies it touches harder.
- **Vertices around the origin.** Jolt computes a body's inertia in `f32` about the body origin and
  decomposes it unless a vertex is kinematic. Vertices far from the origin compared with their
  spread, or on one line, are refused ([limits.md](limits.md#soft-body-inertia)). Author the
  vertices around the origin and place the body with `SoftBodySettings::position`. The check is
  conservative for thin bodies: a free ribbon narrower than about 1/70 of its length, or a tube of
  radius below about 1/130 of its length, is refused even centred on the origin, although Jolt
  decomposes it. Widen it or make a vertex kinematic, which skips the check.
- **Pressure** needs counter-clockwise faces that enclose, about the body origin, a volume large
  enough for it at the start geometry ([limits.md](limits.md#soft-body-pressure)). The error bound
  of that volume grows with the cube of the distance from the origin, so a pressurised body must
  also keep its vertices around the origin.
- **Forces.** The force added to a soft body in one step is bounded by the acceleration it gives its
  lightest vertex, and unpinning a vertex rechecks it.

## Saving, rebasing, events

- A `WorldState` holds the vertex positions and velocities and the bounds; vertex inverse masses are
  configuration it does not save ([state.md](state.md)).
- `rebase` moves the vertices with the body: they are stored relative to it.
- Soft body contacts and validations are events of their own (`EventSettings::soft_body_contacts`,
  `soft_body_validations`), and a `ContactListener` can change or reject a soft body contact
  ([events.md](events.md)).

## Not bound

Skinned soft bodies (Jolt's skin constraints, which pin vertices to an animated skeleton) are in
neither layer.
