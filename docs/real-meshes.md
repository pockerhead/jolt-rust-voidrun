# Real meshes

The mesh and convex hull shapes are tested on models from open sources, not only on generated
geometry. Exported models carry what generated grids lack: bevels made of long thin triangles,
degenerate and zero-area triangles, triangles listed twice with opposite windings, and very fine
tessellation.

## The models

All are CC0 or public domain. The small ones are committed under
[`assets/models`](../assets/models) with their licence texts; the large ones are downloaded by
`scripts/fetch_models.py`. [`assets/models/models.tsv`](../assets/models/models.tsv) records for
each the source URL (and the archive member and archive SHA-256 for Kenney's kits, whose URLs
change when a kit is re-uploaded), the file's SHA-256, author, licence, size and triangle count.

| Model | Role | Author, licence | Triangles | Where |
|---|---|---|---|---|
| `radio.glb` | prop with 1 mm bevels; a third of its triangles are slivers | Kenney, CC0 | 432 | committed |
| `kitchenFridgeLarge.glb` | prop, 20 reversed triangle pairs | Kenney, CC0 | 436 | committed |
| `bathtub.glb` | prop, a hollow to land in | Kenney, CC0 | 602 | committed |
| `bookcaseOpen.glb` | prop, 28 zero-area triangles | Kenney, CC0 | 320 | committed |
| `oloid256_tri.obj` | convex hull: every vertex on the surface | Keenan Crane, CC0 | 512 | committed |
| `track-straight.glb` | level tile, 10 x 10 m, a character walks across | Kenney, CC0 | 106 | committed |
| `corridor-wide-corner.glb` | level piece, every triangle has a reversed twin | Kenney, CC0 | 1766 | committed |
| `spot.obj` | closed smooth mesh | Keenan Crane, public domain | 5856 | downloaded |
| `ScatteringSkull.gltf` + `.bin` | large mesh | Vladimir Petkovic (Khronos sample), CC0 | 188 871 | downloaded |

`crates/mesh-import` (not published) reads them: OBJ, and glTF and GLB with the node transforms
applied.

## What is checked

`tests/real_meshes.rs` (committed models, every CI leg including the assertions leg) and
`tests/real_meshes_downloaded.rs` (ignored unless the models are fetched; the CI job
`real-meshes` fetches them, with and without assertions) check per model:

- `Shape::new_mesh` builds the mesh, and the triangles it drops cover at most 0.1 % of the
  surface area; above that the test fails and the mesh rule has to change.
- `Shape::new_convex_hull` of all vertices builds; the oloid's hull passes within 2 mm of every
  vertex (measured 1.02 mm; Jolt keeps at most 256 points and lets a left-out point lie up to its
  hull tolerance, 1 mm, outside).
- The mesh saves and restores (`Shape::save_binary_state`), and the restored mesh saves to the
  same bytes.
- Spheres and boxes dropped on flat places of the mesh are at rest after 3 s (slower than
  0.05 m/s) with a ray down from them hitting the mesh within their size plus Jolt's penetration
  slop (2 cm), and no body's centre crosses a mesh surface between two ticks. On the closed curved
  models (spot, skull), which have no flat place, the bodies fall from up to 2 m with
  `MotionQuality::LinearCast`; there the check is that none crosses the surface and each ends on
  the mesh or the floor.
- A humanoid character (`CharacterSettings::humanoid(1.8, 0.3)`) walks 8 m along the track tile
  and 6.25 m across the corridor's two-sided floor, on the ground every tick, ending within 0.5 m
  of the goal (measured 0.3 m short of it in the corridor, at the goal on the track).
- `tests/real_meshes_determinism.rs` drops 20 spheres and boxes on the track tile, the corridor
  and the skull and compares 300 ticks of every body with 1 and 4 workers in separate processes.

## Dropped triangles and timings

Measured on one Windows machine with the release build. Props are built with
`MeshSettings::max_convex_extent(2.0)`, the size of the convex shapes that meet them (hand-sized
items, characters); the other models keep the default extent (1100 m). "Default extent" gives
the props' numbers with the default.

| Model | Dropped | Dropped area | Default extent | Mesh build | Saved bytes | Save | Restore |
|---|---|---|---|---|---|---|---|
| radio | 30 | 0.038 % | 88, 0.75 % | 0.22 ms | 9 322 | 14 µs | 2.3 µs |
| kitchenFridgeLarge | 8 | 0.0004 % | 25, 0.012 % | 0.22 ms | 9 730 | 11 µs | 2.3 µs |
| bathtub | 3 (degenerate) | 0 | 17, 0.018 % | 0.35 ms | 14 278 | 24 µs | 2.1 µs |
| bookcaseOpen | 28 (zero area) | 0 | the same | 0.15 ms | 6 618 | 5 µs | 1.4 µs |
| oloid | 28 | 0.041 % | | 0.26 ms | 7 726 | 9 µs | 2.1 µs |
| track-straight | 12 (degenerate) | 0 | | 0.06 ms | 2 562 | 4 µs | 2.2 µs |
| corridor-wide-corner | 0 | 0 | | 0.91 ms | 43 990 | 73 µs | 15 µs |
| spot | 0 | 0 | | 3.1 ms | 88 750 | 138 µs | 28 µs |
| ScatteringSkull at 10x | 14 | 0.0004 % | | 113 ms | 3 026 318 | 3.0 ms | 0.54 ms |

Two findings:

- The default convex extent is generous for small props: the radio's 1 mm bevel strips are kept
  for convex shapes up to 2 m but dropped for 1100 m, which is 0.75 % of its area. Build props
  for the convex shapes that will touch them ([limits](limits.md#convex-shapes-against-meshes)).
- The skull is 0.25 m tall. At that size its 188 871 triangles average 1.5 mm², twice which is
  below the 1e-5 m² the mesh rule keeps above Jolt's sliver assertion (1e-6 m², see
  [limits](limits.md#triangle-meshes)), so every triangle is dropped and `Shape::new_mesh` returns
  `MeshError::NoTriangles` (`the_skull_at_its_own_size_is_too_fine_for_jolt` pins it). The tests
  use it as a 2.5 m statue. Use a convex hull or a decimated mesh for small detailed objects.

Restoring a cooked mesh was about 200 times faster than building it for the skull and 30 to 110
times for the other models ([shape cooking](shape-cooking.md)).

## Running the checks

```text
python scripts/fetch_models.py --out target/models
OXIJOLT_MODELS=target/models cargo test -p oxijolt --test real_meshes --test real_meshes_downloaded --test real_meshes_determinism -- --include-ignored --nocapture
```

The fetch script uses only Python's standard library, keeps a file only when its SHA-256 matches
`models.tsv`, skips files that are already there and checks the committed files too. The
playground shows the models in its [`meshes` and `model` scenes](playground.md).
