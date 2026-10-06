//! Compound edits through bodies: a `MutableCompound` publishes shapes that `set_shape`
//! installs, with the wake, mass, id, alias, rule and state behaviour of a shape change.

mod common;

use common::constraint_kinds::create_every_kind;
use common::controls::*;
use common::events::add_cloth;
use common::meshes::grid;
use common::*;
use oxijolt::*;

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);
/// Half extent of a ledge tile: 2 m wide, 0.5 m thick, 2 m deep.
const TILE: Vec3 = Vec3::new(1.0, 0.25, 1.0);
/// Height of a ledge body's origin; the tiles' tops are at 2.25.
const LEDGE_Y: Real = 2.0;

fn tile() -> Shape {
    Shape::new_box(TILE).unwrap()
}

/// An editor of `count` tiles in a row along +X, 2.5 m apart from x = 0, with user data
/// `10 + index`.
fn row(tile: &Shape, count: u32) -> MutableCompound {
    let children: Vec<_> = (0..count)
        .map(|i| child(tile, Vec3::new(2.5 * i as f32, 0.0, 0.0), 10 + i))
        .collect();
    MutableCompound::from_children(&children).unwrap()
}

/// A static body of the editor's current shape at `x`, `LEDGE_Y`.
fn add_ledge(world: &mut PhysicsWorld, editor: &MutableCompound, x: Real) -> BodyId {
    world
        .create_body(
            &editor.to_shape().unwrap(),
            &BodySettings::new_static().position(RVec3::new(x, LEDGE_Y, 0.0)),
        )
        .unwrap()
}

/// Installs the editor's current shape on `body`.
fn commit(
    world: &mut PhysicsWorld,
    body: BodyId,
    editor: &MutableCompound,
    activation: Activation,
) {
    let shape = editor.to_shape().unwrap();
    world
        .body_mut(body)
        .unwrap()
        .set_shape(&shape, None, activation)
        .unwrap();
}

/// The first hit of a ray straight down through `(x, z)` from 10 m up.
fn ray_down(world: &PhysicsWorld, x: Real, z: Real) -> Option<RayHit> {
    let ray = RayCast {
        origin: RVec3::new(x, 10.0, z),
        direction: Vec3::new(0.0, -20.0, 0.0),
    };
    world.cast_ray(ray, &QueryFilter::new()).unwrap()
}

/// The compound child of `body` at `(x, 0)` in the tiles' height: a ray from inside a tile
/// hits that tile at its start, below anything resting on it.
fn child_below(world: &PhysicsWorld, body: BodyId, x: Real) -> Option<CompoundSubShape> {
    let ray = RayCast {
        origin: RVec3::new(x, LEDGE_Y + 0.1, 0.0),
        direction: Vec3::new(0.0, -0.2, 0.0),
    };
    world
        .cast_ray(ray, &QueryFilter::new())
        .unwrap()
        .filter(|hit| hit.body == body)
        .and_then(|hit| hit.compound_child)
}

/// A dynamic cube resting asleep on the tile under `x`.
fn sleeping_cube_at(world: &mut PhysicsWorld, x: Real) -> BodyId {
    let cube = add_cube(world, RVec3::new(x, LEDGE_Y + 0.75, 0.0));
    fall_asleep(world, cube);
    cube
}

fn activated(world: &mut PhysicsWorld) -> Vec<BodyId> {
    world
        .take_events()
        .activations
        .into_iter()
        .filter_map(|event| match event {
            ActivationEvent::Activated(id) => Some(id),
            ActivationEvent::Deactivated(_) => None,
        })
        .collect()
}

#[test]
fn removing_a_child_drops_the_cube_it_carried() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().body_activation(true));
    add_floor(&mut world);
    let tile = tile();
    let mut editor = row(&tile, 2);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let on_first = sleeping_cube_at(&mut world, 0.0);
    let on_second = sleeping_cube_at(&mut world, 2.5);
    let far = add_cube(&mut world, RVec3::new(20.0, 0.5, 0.0));
    fall_asleep(&mut world, far);
    for id in [on_first, on_second, far] {
        assert!(world.body(id).unwrap().is_sleeping());
    }
    world.take_events();

    editor.remove_shape(1).unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    // Both cubes lie in the old bounds, the far one in neither; wakes go in id order.
    assert_eq!(activated(&mut world), [on_first, on_second]);
    assert!(world.body(far).unwrap().is_sleeping());
    step(&mut world, 120);
    let fallen = world.body(on_second).unwrap().position().y;
    assert!(fallen < 0.6, "{fallen}");
    let kept = world.body(on_first).unwrap().position().y;
    assert!((kept - (LEDGE_Y + 0.75)).abs() < 0.03, "{kept}");
}

#[test]
fn wake_order_does_not_depend_on_broad_phase_optimisation() {
    let woken = |optimize: bool| {
        let mut world = world(GRAVITY, 1);
        world.set_event_settings(EventSettings::default().body_activation(true));
        add_floor(&mut world);
        let tile = tile();
        let mut editor = row(&tile, 3);
        let ledge = add_ledge(&mut world, &editor, 0.0);
        let cubes: Vec<_> = [0.0, 2.5, 5.0]
            .into_iter()
            .map(|x| sleeping_cube_at(&mut world, x))
            .collect();
        // A sleeping cube moved away: the broad phase may still hold its old bounds.
        world
            .body_mut(cubes[0])
            .unwrap()
            .set_position(RVec3::new(30.0, 0.5, 0.0), Activation::DontActivate)
            .unwrap();
        if optimize {
            world.optimize_broad_phase();
        }
        world.take_events();
        editor.remove_shape(2).unwrap();
        commit(&mut world, ledge, &editor, Activation::DontActivate);
        activated(&mut world)
            .iter()
            .map(|id| id.to_raw())
            .collect::<Vec<_>>()
    };
    let plain = woken(false);
    assert_eq!(plain.len(), 2, "{plain:?}");
    assert_eq!(plain, woken(true));
}

#[test]
fn an_added_child_catches_a_falling_cube() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let tile = tile();
    let mut editor = row(&tile, 1);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let cube = add_cube(&mut world, RVec3::new(5.0, 6.0, 0.0));
    step(&mut world, 10);

    editor
        .add_shape(&child(&tile, Vec3::new(5.0, 0.0, 0.0), 77))
        .unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    step(&mut world, 120);
    let y = world.body(cube).unwrap().position().y;
    assert!((y - (LEDGE_Y + 0.75)).abs() < 0.03, "{y}");
    assert_eq!(
        child_below(&world, ledge, 5.0),
        Some(CompoundSubShape {
            index: 1,
            user_data: 77
        })
    );
}

#[test]
fn a_moved_child_pushes_a_resting_cube_up() {
    let mut world = world(GRAVITY, 1);
    let tile = tile();
    let mut editor = row(&tile, 2);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let cube = sleeping_cube_at(&mut world, 2.5);

    // The tile rises into the cube, so the pair stays in contact and the cube is pushed out.
    editor
        .modify_shape(1, Vec3::new(2.5, 0.3, 0.0), Quat::IDENTITY, None)
        .unwrap();
    assert_eq!(editor.sub_shape_user_data(1), Some(11));
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    assert!(!world.body(cube).unwrap().is_sleeping());
    step(&mut world, 60);
    let y = world.body(cube).unwrap().position().y;
    assert!(y > LEDGE_Y + 0.75 + 0.2, "{y}");
}

#[test]
fn a_replaced_child_keeps_its_slot_user_data() {
    let mut world = world(GRAVITY, 1);
    let tile = tile();
    let mut editor = row(&tile, 3);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let slab = Shape::new_box(Vec3::new(1.0, 0.75, 1.0)).unwrap();
    editor
        .modify_shape(1, Vec3::new(2.5, 0.0, 0.0), Quat::IDENTITY, Some(&slab))
        .unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    let hit = ray_down(&world, 2.5, 0.0).unwrap();
    assert_eq!(
        hit.compound_child,
        Some(CompoundSubShape {
            index: 1,
            user_data: 11
        })
    );
    // The taller slab's top is at 2.75.
    assert!(
        (hit.distance - (10.0 - 2.75)).abs() < 1.0e-4,
        "{}",
        hit.distance
    );
}

#[test]
fn later_children_move_down_after_a_removal() {
    let tile = tile();
    for count in [4, 5, 8, 9] {
        let mut world = world(GRAVITY, 1);
        let mut editor = row(&tile, count);
        let ledge = add_ledge(&mut world, &editor, 0.0);
        let x = |i: u32| 2.5 * Real::from(i as u16);
        let middle = count / 2;
        editor.remove_shape(middle).unwrap();
        editor.remove_shape(count - 2).unwrap();
        commit(&mut world, ledge, &editor, Activation::DontActivate);
        for i in 0..count {
            let expected = if i == middle || i == count - 1 {
                None
            } else {
                let index = if i > middle { i - 1 } else { i };
                Some(CompoundSubShape {
                    index,
                    user_data: 10 + i,
                })
            };
            assert_eq!(child_below(&world, ledge, x(i)), expected, "{count}: {i}");
        }
    }

    // A count that crosses a power of two changes the id of a child nobody touched.
    let mut world = world(GRAVITY, 1);
    let mut editor = row(&tile, 4);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let before = ray_down(&world, 0.0, 0.0).unwrap().sub_shape_id;
    editor
        .add_shape(&child(&tile, Vec3::new(10.0, 0.0, 0.0), 14))
        .unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    let after = ray_down(&world, 0.0, 0.0).unwrap();
    assert_ne!(after.sub_shape_id, before);
    assert_eq!(
        after.compound_child,
        Some(CompoundSubShape {
            index: 0,
            user_data: 10
        })
    );
}

#[test]
fn a_commit_reaches_only_the_body_it_is_set_on() {
    let materials = [
        PhysicsMaterial::new(20).unwrap(),
        PhysicsMaterial::new(21).unwrap(),
    ];
    let tiles = materials
        .each_ref()
        .map(|material| Shape::new_box_with_material(TILE, 0.05, material).unwrap());
    let mut editor = MutableCompound::from_children(&[
        child(&tiles[0], Vec3::ZERO, 0),
        child(&tiles[1], Vec3::new(2.5, 0.0, 0.0), 1),
    ])
    .unwrap();
    let first = editor.to_shape().unwrap();
    let scaled = Shape::scaled(&first, Vec3::new(1.5, 1.5, 1.5)).unwrap();

    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let edited = world
        .create_body(&first, &BodySettings::new_static())
        .unwrap();
    let alias = world
        .create_body(
            &first,
            &BodySettings::new_static().position(RVec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
    let scaled_alias = world
        .create_body(
            &scaled,
            &BodySettings::new_static().position(RVec3::new(40.0, 0.0, 0.0)),
        )
        .unwrap();
    let mut other_world = common::world(GRAVITY, 1);
    let elsewhere = other_world
        .create_body(&first, &BodySettings::new_static())
        .unwrap();

    editor.remove_shape(1).unwrap();
    commit(&mut world, edited, &editor, Activation::DontActivate);
    drop((editor, first, scaled, tiles, materials));

    // Rays through the second tile of each body; the scaled body's second tile is 1.5 times
    // as far from its origin.
    let second = |at: Real| ray_down(&world, at, 0.0).and_then(|hit| hit.compound_child);
    assert_eq!(second(2.5), None);
    let kept = Some(CompoundSubShape {
        index: 1,
        user_data: 1,
    });
    assert_eq!(second(22.5), kept);
    assert_eq!(
        ray_down(&world, 43.75, 0.0).map(|hit| hit.body),
        Some(scaled_alias)
    );
    let in_other_world = ray_down(&other_world, 2.5, 0.0).unwrap();
    assert_eq!(
        (in_other_world.body, in_other_world.compound_child),
        (elsewhere, kept)
    );

    // The untouched alias still carries its materials.
    let ball = Shape::new_sphere(0.2).unwrap();
    let dropped = world
        .create_body(
            &ball,
            &BodySettings::new_dynamic().position(RVec3::new(22.5, 0.6, 0.0)),
        )
        .unwrap();
    step(&mut world, 30);
    let events = world.take_events();
    let landed = events.contacts.iter().find_map(|event| match event {
        ContactEvent::Added { manifold, .. } if manifold.pair.body2 == dropped => {
            Some((manifold.pair.body1, manifold.materials))
        }
        _ => None,
    });
    assert_eq!(landed, Some((alias, [Some(21), None])));
}

/// What a dynamic body's mass and inertia show: its mass, and its velocities after the same
/// impulses at its centre of mass, as bits. Unlike an impulse at a point, these do not depend
/// on the body's position, which Jolt rounds when the centre of mass moves.
fn mass_response(world: &mut PhysicsWorld, id: BodyId) -> Vec<u32> {
    let mut body = world.body_mut(id).unwrap();
    body.add_angular_impulse(Vec3::new(0.3, -0.2, 0.5)).unwrap();
    body.add_impulse(Vec3::new(1.0, 2.0, -0.5)).unwrap();
    let (v, w) = (body.linear_velocity(), body.angular_velocity());
    let mut bits = vec![body.mass().unwrap().to_bits()];
    bits.extend([v.x, v.y, v.z, w.x, w.y, w.z].map(f32::to_bits));
    bits
}

/// Rotation and velocities of a body, as bits.
fn spin(world: &PhysicsWorld, id: BodyId) -> Vec<u32> {
    let body = world.body(id).unwrap();
    let (q, v, w) = (
        body.rotation(),
        body.linear_velocity(),
        body.angular_velocity(),
    );
    [q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]
        .map(f32::to_bits)
        .to_vec()
}

#[test]
fn a_dynamic_compound_losing_a_child_gets_fresh_mass_properties() {
    let block = Shape::new_box(Vec3::new(0.4, 0.2, 0.3)).unwrap();
    let ball = Shape::new_sphere(0.3).unwrap();
    let capsule = Shape::new_capsule(0.3, 0.1).unwrap();
    let pose = |settings: BodySettings| {
        settings
            .position(RVec3::new(1.0, 2.0, 3.0))
            .rotation(quat_about(Vec3::new(0.48, 0.6, -0.64), 0.7))
    };
    for mass in [None, Some(3.0)] {
        let with_mass = |settings: BodySettings| match mass {
            Some(mass) => settings.mass(mass),
            None => settings,
        };
        let mut editor = MutableCompound::from_children(&[
            child(&block, Vec3::new(0.2, 0.0, 0.0), 0),
            CompoundChild {
                rotation: quat_about(Vec3::new(0.0, 0.6, 0.8), 0.5),
                ..child(&capsule, Vec3::new(-0.4, 0.1, 0.0), 1)
            },
            child(&ball, Vec3::new(-0.3, 0.4, 0.1), 2),
        ])
        .unwrap();
        let mut world = world(Vec3::ZERO, 1);
        let id = world
            .create_body(
                &editor.to_shape().unwrap(),
                &with_mass(pose(BodySettings::new_dynamic())),
            )
            .unwrap();
        let origin = world.body(id).unwrap().position();

        editor.remove_shape(1).unwrap();
        let shape = editor.to_shape().unwrap();
        world
            .body_mut(id)
            .unwrap()
            .set_shape(&shape, mass, Activation::Activate)
            .unwrap();
        // The origin stays, up to the rounding of Jolt's centre-of-mass arithmetic.
        let moved = distance(world.body(id).unwrap().position(), origin);
        assert!(moved < 1.0e-5, "{moved}");

        let mut fresh_world = common::world(Vec3::ZERO, 1);
        let fresh = fresh_world
            .create_body(&shape, &with_mass(pose(BodySettings::new_dynamic())))
            .unwrap();
        assert_eq!(
            mass_response(&mut world, id),
            mass_response(&mut fresh_world, fresh),
            "{mass:?}"
        );
        step(&mut world, 1);
        step(&mut fresh_world, 1);
        assert_eq!(spin(&world, id), spin(&fresh_world, fresh), "{mass:?}");
    }
}

/// Checks that `set_shape` refused `editor`'s shape on `id` and left the body, its rays and
/// the world's structure as they were.
fn assert_refused(
    world: &mut PhysicsWorld,
    id: BodyId,
    editor: &MutableCompound,
    mass: Option<f32>,
) {
    let saved = world.save_state();
    let mut before = Vec::new();
    record_body(world, id, &mut before);
    let at = world.body(id).unwrap().position();
    let hit_before = ray_down(world, at.x, at.z).map(|hit| (hit.body, hit.sub_shape_id));
    let shape = editor.to_shape().unwrap();
    let result = world
        .body_mut(id)
        .unwrap()
        .set_shape(&shape, mass, Activation::Activate);
    assert!(result.is_err(), "{id:?}");
    let mut after = Vec::new();
    record_body(world, id, &mut after);
    assert_eq!(after, before, "{id:?}");
    let hit_after = ray_down(world, at.x, at.z).map(|hit| (hit.body, hit.sub_shape_id));
    assert_eq!(hit_after, hit_before, "{id:?}");
    world.restore_state(&saved).unwrap();
}

#[test]
fn commits_follow_the_body_rules_of_set_shape() {
    let mut world = world(Vec3::ZERO, 1);
    let cube = cube_shape();
    let (vertices, triangles) = grid(2, 1.0, |_, _| 0.0);
    let (mesh, _) = Shape::new_mesh(&vertices, &triangles).unwrap();
    let terrain = flat_height_field();
    let tiny = Shape::new_sphere(1.0e-20).unwrap();
    let with = |extra: &Shape| {
        MutableCompound::from_children(&[
            child(&cube, Vec3::ZERO, 0),
            child(extra, Vec3::new(0.0, -2.0, 0.0), 1),
        ])
        .unwrap()
    };
    let only =
        |shape: &Shape| MutableCompound::from_children(&[child(shape, Vec3::ZERO, 0)]).unwrap();
    let at = |x: Real| RVec3::new(x, 0.0, 0.0);
    let dynamic = world
        .create_body(&cube, &BodySettings::new_dynamic().position(at(0.0)))
        .unwrap();
    let kinematic = world
        .create_body(&cube, &BodySettings::new_kinematic().position(at(50.0)))
        .unwrap();
    let movable = world
        .create_body(
            &cube,
            &BodySettings::new_static()
                .position(at(100.0))
                .allow_dynamic_or_kinematic(true),
        )
        .unwrap();
    let sensor = world
        .create_body(
            &cube,
            &BodySettings::new_static().position(at(150.0)).sensor(true),
        )
        .unwrap();
    assert_refused(&mut world, dynamic, &with(&mesh), None);
    assert_refused(&mut world, dynamic, &only(&tiny), None);
    assert_refused(&mut world, kinematic, &with(&terrain), Some(1.0));
    assert_refused(&mut world, movable, &with(&mesh), None);
    assert_refused(&mut world, sensor, &with(&mesh), None);

    world
        .body_mut(dynamic)
        .unwrap()
        .add_force(Vec3::new(1.0, 0.0, 0.0))
        .unwrap();
    assert_refused(&mut world, dynamic, &with(&cube), None);
    world.body_mut(dynamic).unwrap().reset_forces();

    let partner = add_cube(&mut world, at(1.5));
    assert!(create_every_kind(&mut world, dynamic, partner)
        .iter()
        .all(Result::is_ok));
    assert_eq!(
        world.body_mut(dynamic).unwrap().set_shape(
            &with(&cube).to_shape().unwrap(),
            None,
            Activation::Activate
        ),
        Err(BodyError::UsedByConstraint(dynamic))
    );

    let cloth = add_cloth(&mut world, at(-50.0), Quat::IDENTITY);
    assert_eq!(
        world.body_mut(cloth).unwrap().set_shape(
            &with(&cube).to_shape().unwrap(),
            None,
            Activation::Activate
        ),
        Err(BodyError::SoftBody(cloth))
    );
    let capsule = Shape::new_capsule(0.7, 0.4).unwrap();
    let character = world
        .create_character(
            &CharacterSettings::new(&capsule).inner_body(Some(InnerBody {
                shape: &capsule,
                object_layer: ObjectLayer::MOVING,
            })),
            at(-100.0),
            Quat::IDENTITY,
        )
        .unwrap();
    let inner = world.character(character).unwrap().inner_body().unwrap();
    assert_eq!(
        world.body_mut(inner).unwrap().set_shape(
            &with(&cube).to_shape().unwrap(),
            None,
            Activation::Activate
        ),
        Err(BodyError::OwnedByCharacter(inner))
    );

    // A plain static body takes static-only children.
    let plain = world
        .create_body(&cube, &BodySettings::new_static().position(at(200.0)))
        .unwrap();
    commit(&mut world, plain, &with(&terrain), Activation::DontActivate);
}

#[test]
fn editing_and_publishing_do_not_touch_the_world() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let tile = tile();
    let mut editor = row(&tile, 2);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let saved = world.save_state();

    editor
        .add_shape(&child(&tile, Vec3::new(5.0, 0.0, 0.0), 12))
        .unwrap();
    assert!(editor
        .add_shape(&child(&tile, Vec3::new(f32::NAN, 0.0, 0.0), 13))
        .is_err());
    let published = editor.to_shape().unwrap();
    editor
        .modify_shape(2, Vec3::new(1999.0, 0.0, 0.0), Quat::IDENTITY, None)
        .unwrap();
    editor
        .modify_shape(0, Vec3::new(-1999.0, 0.0, 0.0), Quat::IDENTITY, None)
        .unwrap();
    assert!(matches!(
        editor.to_shape(),
        Err(ShapeError::InvalidDimensions(_))
    ));
    world.restore_state(&saved).unwrap();

    world
        .body_mut(ledge)
        .unwrap()
        .set_shape(&published, None, Activation::DontActivate)
        .unwrap();
    assert_eq!(world.restore_state(&saved), Err(StateError::WorldChanged));
}

#[test]
fn a_state_before_a_commit_is_refused_and_one_after_replays_exactly() {
    let mut world = world(GRAVITY, 1);
    add_floor(&mut world);
    let tile = tile();
    let mut editor = row(&tile, 3);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let cubes = [
        sleeping_cube_at(&mut world, 2.5),
        sleeping_cube_at(&mut world, 5.0),
    ];
    let before = world.save_state();

    editor.remove_shape(1).unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    assert_eq!(world.restore_state(&before), Err(StateError::WorldChanged));

    // Saved after the commit and before the next step, with the pending cache invalidation.
    let after = world.save_state();
    let first = run_digest(&mut world, &cubes, 30);
    world.restore_state(&after).unwrap();
    let replay = run_digest(&mut world, &cubes, 30);
    assert_eq!(first, replay);
    let fallen = world.body(cubes[0]).unwrap().position().y;
    assert!(fallen < LEDGE_Y, "{fallen}");
}

#[test]
fn removed_contacts_after_a_commit_carry_ids_of_the_old_shape() {
    let mut world = world(GRAVITY, 1);
    world.set_event_settings(EventSettings::default().contacts(true));
    let tile = tile();
    let mut editor = row(&tile, 5);
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let cube = sleeping_cube_at(&mut world, 10.0);
    world.take_events();

    // Five children use three index bits, four use two: the cube's tile moves from index 4 to
    // index 3 and its old id decodes to child 0 of the new shape.
    editor.remove_shape(1).unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    step(&mut world, 2);
    let events = world.take_events();
    let removed: Vec<_> = events
        .contacts
        .iter()
        .filter_map(|event| match event {
            ContactEvent::Removed(pair) if pair.body1 == ledge && pair.body2 == cube => {
                Some(pair.sub_shape1)
            }
            _ => None,
        })
        .collect();
    assert_eq!(removed.len(), 1, "{events:?}");
    assert_eq!(
        world.compound_sub_shape(ledge, removed[0]).unwrap(),
        Some(CompoundSubShape {
            index: 0,
            user_data: 10
        })
    );
    assert_eq!(
        child_below(&world, ledge, 10.0),
        Some(CompoundSubShape {
            index: 3,
            user_data: 14
        })
    );
}

#[test]
fn a_character_on_a_removed_child_keeps_updating() {
    let mut world = world(GRAVITY, 1);
    let floor = add_floor(&mut world);
    let material = PhysicsMaterial::new(5).unwrap();
    let plain = tile();
    let marked = Shape::new_box_with_material(TILE, 0.05, &material).unwrap();
    let mut editor = MutableCompound::from_children(&[
        child(&plain, Vec3::new(-3.0, 0.0, 0.0), 0),
        child(&marked, Vec3::ZERO, 1),
    ])
    .unwrap();
    let ledge = add_ledge(&mut world, &editor, 0.0);
    let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
    let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 0.8, 0.0));
    let character = world
        .create_character(
            &settings,
            RVec3::new(0.0, LEDGE_Y + 0.25, 0.0),
            Quat::IDENTITY,
        )
        .unwrap();
    world
        .refresh_character_contacts(character, &QueryFilter::new())
        .unwrap();
    assert_eq!(
        world.character(character).unwrap().ground_compound_child(),
        Some(CompoundSubShape {
            index: 1,
            user_data: 1
        })
    );

    editor.remove_shape(1).unwrap();
    commit(&mut world, ledge, &editor, Activation::DontActivate);
    drop((editor, marked, material));
    let ground = world.character(character).unwrap().ground_compound_child();
    assert!(
        ground.is_none_or(|child| child.user_data == 0),
        "{ground:?}"
    );

    let update = ExtendedUpdateSettings::default();
    world
        .update_character(character, 1.0e-6, GRAVITY, &update, &QueryFilter::new())
        .unwrap();
    let mut velocity = Vec3::ZERO;
    for _ in 0..90 {
        velocity.y -= 9.81 / 60.0;
        if world.character(character).unwrap().ground_state() == GroundState::OnGround {
            velocity.y = 0.0;
        }
        world
            .character_mut(character)
            .unwrap()
            .set_linear_velocity(velocity)
            .unwrap();
        world
            .update_character(character, 1.0 / 60.0, GRAVITY, &update, &QueryFilter::new())
            .unwrap();
    }
    let landed = world.character(character).unwrap();
    assert_eq!(landed.ground_body(), Some(floor));
    assert!(landed.position().y.abs() < 0.05, "{:?}", landed.position());
}

#[test]
fn a_committed_shape_nests_and_scales() {
    let tile = tile();
    let editor = row(&tile, 2);
    let published = editor.to_shape().unwrap();
    Shape::new_compound(&[
        child(&published, Vec3::ZERO, 0),
        child(&tile, Vec3::new(10.0, 0.0, 0.0), 1),
    ])
    .unwrap();
    Shape::scaled(&published, Vec3::new(2.0, 2.0, 2.0)).unwrap();
    Shape::scaled(&published, Vec3::new(2.0, 1.0, 1.0)).unwrap();

    // A non-uniform scale needs children turned onto the scale's axes.
    let turned = MutableCompound::from_children(&[CompoundChild {
        rotation: quat_about(Vec3::new(0.0, 1.0, 0.0), std::f32::consts::FRAC_PI_4),
        ..child(&tile, Vec3::ZERO, 0)
    }])
    .unwrap()
    .to_shape()
    .unwrap();
    assert!(matches!(
        Shape::scaled(&turned, Vec3::new(2.0, 1.0, 1.0)),
        Err(ShapeError::InvalidSettings(_))
    ));

    // Nested past 32 bits: a two-child publication is 1 bit wide and Jolt refuses 33.
    let mut nested = published;
    for _ in 1..32 {
        nested = Shape::new_compound(&[child(&nested, Vec3::ZERO, 0), child(&tile, Vec3::ZERO, 1)])
            .unwrap();
    }
    let too_wide =
        Shape::new_compound(&[child(&nested, Vec3::ZERO, 0), child(&tile, Vec3::ZERO, 1)]);
    assert!(
        matches!(too_wide, Err(ShapeError::Rejected(_))),
        "{:?}",
        too_wide.err()
    );

    // A one-child publication under 32 bits of parents would push its index at bit 32.
    let mut nested = row(&tile, 1).to_shape().unwrap();
    for level in 1..=32 {
        let next =
            Shape::new_compound(&[child(&nested, Vec3::ZERO, 0), child(&tile, Vec3::ZERO, 1)]);
        if level < 32 {
            nested = next.unwrap();
        } else {
            assert!(
                matches!(next, Err(ShapeError::InvalidSettings(_))),
                "{:?}",
                next.err()
            );
        }
    }
}
