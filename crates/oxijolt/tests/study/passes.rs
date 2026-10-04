//! The game's own queries around the move (spec C.1), with a configuration's padding.

use std::collections::BTreeMap;

use oxijolt::*;

use crate::common::math::{add, dot, f3, norm, rvec3, scale, V3};
use crate::common::walker::{capsule, from_y_to, CENTRE_UP, RADIUS, SNAP};
use crate::common::Groups;

/// The capsule's centre for body origin `origin`.
fn centre(origin: V3, up: V3) -> RVec3 {
    rvec3(add(origin, scale(up, f64::from(CENTRE_UP))))
}

/// Q6, the depenetration push: every obstacle the capsule at `origin` overlaps, the deepest
/// hit per collider (body and compound child), each pushing along its outward normal by its
/// depth plus the padding `p`, summed in collider order and capped at `RADIUS + p` (spec D.2
/// rule 2). Returns the push and the deepest overlap found.
pub fn depenetrate(
    world: &PhysicsWorld,
    filter: &QueryFilter<'_>,
    origin: V3,
    up: V3,
    p: f32,
) -> (V3, f64) {
    let shape = capsule();
    let query = CollideShape::new(&shape, centre(origin, up), from_y_to(up));
    let hits = world.collide_shape(&query, filter).unwrap();
    let mut deepest: BTreeMap<(u32, Option<u32>), CollideShapeHit> = BTreeMap::new();
    for hit in hits {
        if hit.penetration_depth <= 0.0 {
            continue;
        }
        let key = (
            hit.body.to_raw(),
            hit.compound_child.map(|child| child.index),
        );
        let keep = deepest
            .get(&key)
            .is_none_or(|kept| hit.penetration_depth > kept.penetration_depth);
        if keep {
            deepest.insert(key, hit);
        }
    }
    let mut push = [0.0; 3];
    let mut max_depth: f64 = 0.0;
    for hit in deepest.values() {
        let depth = f64::from(hit.penetration_depth);
        max_depth = max_depth.max(depth);
        push = add(push, scale(f3(hit.normal), depth + f64::from(p)));
    }
    let limit = f64::from(RADIUS + p);
    let length = norm(push);
    if length > limit {
        push = scale(push, limit / length);
    }
    (push, max_depth)
}

/// Q4, the terrain support normal: the capsule cast along -up by `p + 0.05` with target
/// distance `p` against terrain only; a hit within the target distance is cast again with
/// target 0 for an exact normal. Returns the obstacle's outward normal.
pub fn terrain_support_normal(
    world: &PhysicsWorld,
    terrain: ObjectLayer,
    origin: V3,
    up: V3,
    p: f32,
) -> Option<V3> {
    let shape = capsule();
    let layers = [terrain];
    let filter = QueryFilter::new().object_layers(&layers);
    let down = scale(up, -(f64::from(p) + 0.05));
    let cast = |target: f32| {
        let query = ShapeCast::new(
            &shape,
            centre(origin, up),
            from_y_to(up),
            crate::common::math::vec3(down),
        )
        .target_distance(target);
        world.cast_shape(&query, &filter).unwrap()
    };
    let hit = cast(p)?;
    if hit.fraction == 0.0 {
        if let Some(exact) = cast(0.0) {
            return Some(f3(exact.normal));
        }
    }
    Some(f3(hit.normal))
}

/// What Q5 found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snap {
    pub distance: f64,
    pub normal: V3,
}

/// Q5, the floor snap: the capsule cast along -up by `SNAP` with target distance `p`; a
/// walkable, not dynamic hit gives the distance to move down. With `structure_edges`, a steep
/// hit on a structure is accepted too when the capsule moved down to reach it (fraction above
/// 0) and its normal faces up: a ledge edge below the capsule, not a wall the padded capsule
/// already touches (a study remedy, not the game's pass).
pub fn snap(
    world: &PhysicsWorld,
    filter: &QueryFilter<'_>,
    origin: V3,
    up: V3,
    p: f32,
    structure_edges: bool,
) -> Option<Snap> {
    let shape = capsule();
    let query = ShapeCast::new(
        &shape,
        centre(origin, up),
        from_y_to(up),
        crate::common::math::vec3(scale(up, -f64::from(SNAP))),
    )
    .target_distance(p);
    let hit = world.cast_shape(&query, filter).unwrap()?;
    let normal = f3(hit.normal);
    let walkable = dot(normal, up) >= std::f64::consts::FRAC_1_SQRT_2;
    let structure = hit
        .compound_child
        .is_some_and(|child| child.user_data == Groups::STRUCTURE);
    let edge_below = structure && hit.fraction > 0.0 && dot(normal, up) > 0.0;
    let dynamic = world.body(hit.body).unwrap().motion_type() == MotionType::Dynamic;
    ((walkable || (structure_edges && edge_below)) && !dynamic).then_some(Snap {
        distance: f64::from(hit.distance),
        normal,
    })
}
