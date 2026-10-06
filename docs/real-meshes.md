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
  surface area; above that the test fails and the mesh rule has to change. The skull at its own
  size is built for convex shapes up to 2 m and may drop at most 0.5 %.
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
  the mesh or the floor. The skull gets 3 cm spheres and 4 cm boxes at its own size and 0.1 m
  ones as a 2.5 m statue.
- Every file of a model, the skull's glTF buffer too, must match its SHA-256 before it is read;
  `a_changed_skull_buffer_is_refused` changes one byte of a normal, which the importer ignores.
- A humanoid character (`CharacterSettings::humanoid(1.8, 0.3)`) walks 8 m along the track tile
  and 6.25 m across the corridor's two-sided floor, on the ground every tick, ending within 0.5 m
  of the goal (measured 0.3 m short of it in the corridor, at the goal on the track).
- `tests/real_meshes_determinism.rs` drops 20 spheres and boxes on the track tile, the corridor
  and the skull statue and compares 300 ticks of every body, and which bodies start touching the
  mesh in each tick, with 1 and 4 workers in separate processes; at least 15 of the 20 must touch
  the mesh (measured 19, 20 and 18).

## Dropped triangles and timings

Measured on one Windows machine with the release build. Every model is built with the default
`MeshSettings`, so for convex shapes up to 300 m, except the skull at its own size, built with
`MeshSettings::max_convex_extent(2.0)`. "At 1100 m" gives the props' numbers with
`MeshSettings::max_convex_extent(1100.0)`.

| Model | Dropped | Dropped area | At 1100 m | Mesh build | Saved bytes | Save | Restore |
|---|---|---|---|---|---|---|---|
| radio | 14 | 0.024 % | 64, 0.69 % | 0.25 ms | 9 558 | 13 µs | 3 µs |
| kitchenFridgeLarge | 2 | 0.00002 % | 15, 0.0091 % | 0.23 ms | 10 094 | 9 µs | 3 µs |
| bathtub | 3 | < 0.00001 % | 15, 0.018 % | 0.56 ms | 14 278 | 32 µs | 12 µs |
| bookcaseOpen | 28 (zero area) | 0 | the same | 0.17 ms | 6 618 | 8 µs | 3 µs |
| oloid | 12 | 0.0039 % | | 0.25 ms | 7 686 | 15 µs | 5 µs |
| track-straight | 12 (degenerate) | 0 | | 0.07 ms | 2 562 | 5 µs | 3 µs |
| corridor-wide-corner | 0 | 0 | | 0.98 ms | 43 990 | 77 µs | 17 µs |
| spot | 0 | 0 | | 3.5 ms | 88 750 | 146 µs | 29 µs |
| ScatteringSkull, 0.25 m, for 2 m | 3 426 | 0.48 % | | 115 ms | 2 976 438 | 3.1 ms | 0.59 ms |
| ScatteringSkull at 10x | 0 | 0 | | 113 ms | 3 026 002 | 3.3 ms | 0.56 ms |

Two findings:

- The convex extent decides how thin a kept triangle can be. The radio's 1 mm bevel strips are
  kept at the default extent and dropped at 1100 m, where they are 0.69 % of its area. A mesh
  that must meet convex shapes larger than 300 m is built with a larger extent and loses such
  bevels ([limits](limits.md#convex-shapes-against-meshes)).
- The skull is 0.25 m tall and its 188 871 triangles average 1.5 mm². Twice the area of 0.47 % of
  it (by area) is at or below 1e-6 m², where Jolt's collision asserts
  ([limits](limits.md#triangle-meshes)); no rule can keep those. Built for convex shapes up to 2 m
  it loses 0.48 %, at the default 300 m 4.2 % (19 452 triangles), because rounding in the space of
  a large convex shape takes more from small triangles. A mesh of a small detailed object that
  only meets small bodies is built with a small `max_convex_extent`.

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
