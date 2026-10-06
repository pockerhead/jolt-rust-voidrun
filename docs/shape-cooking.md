# Shape cooking

Building a large mesh costs time at load: Jolt sorts the triangles into a tree and compresses
them. `Shape::save_binary_state` writes a built shape to bytes, and
`Shape::restore_binary_state` turns them back into an equal shape by copying Jolt's built data. A
game builds its level meshes once, when it builds its assets, and restores them when a level
loads. For the 188 871-triangle mesh of the [real-mesh tests](real-meshes.md) that was 3 MB of
bytes, restored in 0.5 ms against 113 ms to build.

```rust
use oxijolt::{BodySettings, PhysicsWorld, Shape, Vec3, WorldSettings};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // When the assets are built: build the level mesh once and save it.
    let vertices = [
        Vec3::new(-10.0, 0.0, -10.0),
        Vec3::new(10.0, 0.0, -10.0),
        Vec3::new(10.0, 0.0, 10.0),
        Vec3::new(-10.0, 0.0, 10.0),
    ];
    let (level, _dropped) = Shape::new_mesh(&vertices, &[[0, 2, 1], [0, 3, 2]])?;
    let cooked: Vec<u8> = level.save_binary_state()?;
    // A game writes `cooked` to a file next to its other assets.

    // When the level loads: restore it.
    // SAFETY: the bytes were saved by this build, above, and nothing changed them.
    let level = unsafe { Shape::restore_binary_state(&cooked) }?;
    let mut world = PhysicsWorld::new(WorldSettings::default())?;
    world.create_body(&level, &BodySettings::new_static())?;
    Ok(())
}
```

## What is saved

Every shape kind the crate builds: box, sphere, capsule, cylinder, tapered capsule and cylinder,
convex hull, mesh, heightfield, plane, compound (static or from a `MutableCompound`), scaled and
offset-centre-of-mass shapes, with their children. A child or a material shared by several parents
is saved once and shared again after restoring. A mesh keeps its `max_convex_extent`, a compound
child its user data, a `PhysicsMaterial` its user data. Two saves of equal shapes give equal bytes,
in one process or two, and a restored shape saves to the bytes it came from.

## The bytes belong to one build

Jolt's cooked data has no header and no version: its layout changes between Jolt versions, and Jolt
assumes it is correct. The bytes therefore start with a 48-byte header that names the build:

| Bytes | Field |
|---|---|
| 0..8 | `OXJSHAPE` |
| 8..12 | `BinaryStateError::FORMAT_VERSION`, little-endian |
| 12..16 | flags: double precision, cross-platform determinism, byte order |
| 16..32 | build id: XXH64 of the pinned Jolt and joltc commits and the extension revision |
| 32..40 | payload length |
| 40..48 | XXH64 checksum of the header and the payload |

The payload is one record per shape, children first, each with its type, child and material
indices and Jolt's own bytes, plus the materials. Restore refuses, with
`ShapeError::BinaryState`:

| Bytes | Error |
|---|---|
| without the magic | `BinaryStateError::NotBinaryState` |
| of another format version, precision, determinism mode, byte order, Jolt or joltc commit or extension revision | `OtherBuild` |
| cut short | `Truncated` |
| changed after they were saved | `Corrupt` |
| followed by other bytes, or giving a shape outside the crate's rules (extent, sub-shape ids, expanded size) | `Malformed` |
| with a record structure the joltc extension refuses (a type it does not restore, a child that is not an earlier record, the wrong number of children or materials, a length that does not match) | `Rejected` |

So cook the shapes again whenever the game updates `oxijolt-sys` or changes its features; a cooked
file of the old build is refused, not misread.

## Why restoring is `unsafe`

The checksum detects damage, not forgery: anyone can compute it. Jolt does not validate the inside
of its own records (array lengths, mesh tree offsets, hull indices), so bytes made to pass the
checksum can make Jolt read and write out of bounds. Restore only bytes your own build wrote, such
as the game's shipped assets; never bytes from another player or a server you do not control.
Every accidental change is refused without undefined behaviour.

`docs/limits.md` ([shape binary state](limits.md#shape-binary-state)) lists the checks and why no
saved byte is left without a value.
