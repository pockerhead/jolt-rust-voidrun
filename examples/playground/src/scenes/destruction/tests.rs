//! The destruction scene's tests.

use super::*;

fn scene() -> Destruction {
    Destruction::new(
        &SceneConfig {
            worker_threads: Some(1),
            ..SceneConfig::default()
        },
        1,
    )
    .unwrap()
}

/// Pushes knocked-out bricks out of the way, behind the wall.
const OUT: Vec3 = Vec3::new(0.0, 0.0, -KNOCK_SPEED);

fn run(scene: &mut Destruction, ticks: u32) {
    for _ in 0..ticks {
        scene.update(&Input::default()).unwrap();
    }
}

/// Removes the loose bricks, so that nothing but the ground is in the way of the pieces.
fn clear_debris(scene: &mut Destruction) {
    for body in std::mem::take(&mut scene.debris) {
        scene.tracked.remove(&mut scene.world, body).unwrap();
    }
}

/// The angle in radians by which `body` is turned from where it started.
fn tilt(scene: &Destruction, body: BodyId) -> f32 {
    glam_quat(scene.world.body(body).unwrap().rotation()).angle_between(glam::Quat::IDENTITY)
}

#[test]
fn the_wall_stands_on_the_ground() {
    let mut scene = scene();
    let wall = scene.pieces[0].body;
    run(&mut scene, 120);
    assert!(tilt(&scene, wall) < 0.01, "{}", tilt(&scene, wall));
    let y = position(scene.world.body(wall).unwrap().position()).y;
    assert!(y.abs() < 0.03, "{y}");
}

#[test]
fn duplicate_hits_remove_one_brick() {
    let mut scene = scene();
    scene.knock_out(vec![(5, OUT), (5, OUT), (5, OUT)]).unwrap();
    assert_eq!(scene.brick_count(), 47);
    assert_eq!((scene.pieces.len(), scene.debris.len()), (1, 1));
    assert!(scene.piece_of_brick(5).is_none());
    // A brick that is gone already changes nothing.
    scene.knock_out(vec![(5, OUT)]).unwrap();
    assert_eq!((scene.brick_count(), scene.debris.len()), (47, 1));
}

#[test]
fn destroying_every_brick_removes_the_wall() {
    let mut scene = scene();
    let bodies = scene.world.body_count();
    let every: Vec<(u32, Vec3)> = (0..48).map(|brick| (brick, Vec3::ZERO)).collect();
    scene.knock_out(every).unwrap();
    assert!(scene.pieces.is_empty());
    assert_eq!(scene.debris.len(), 48);
    assert_eq!(scene.world.body_count(), bodies - 1 + 48);
    run(&mut scene, 30);
}

/// Without its bottom row but the rightmost brick, the wall stands on one brick at its
/// right end, its centre of mass far beside that support, and falls over to the left until
/// its open end lies on the ground. It lands there inside the contact it already has with the
/// ground and breaks its left end, not where it still stands.
#[test]
fn a_wall_on_one_brick_falls_over_and_breaks_where_it_lands() {
    let mut scene = scene();
    let wall = scene.pieces[0].body;
    let bottom: Vec<(u32, Vec3)> = (0..WALL[0] - 1).map(|brick| (brick, OUT)).collect();
    scene.knock_out(bottom).unwrap();
    assert_eq!(scene.pieces.len(), 1, "the wall stays one piece");
    assert_eq!(scene.pieces[0].body, wall);
    // The bottom left corner of brick 8, now the left end of the lowest row.
    let (_, [x, y, _]) = scene.piece_of_brick(8).unwrap();
    let open_end = [x - BRICK[0], y - BRICK[1], 0.0];
    run(&mut scene, 150);
    let motion = Motion::of(&scene.world, wall).unwrap();
    assert!(motion.at(open_end).y < 0.03, "{}", motion.at(open_end));
    assert!(tilt(&scene, wall) > 0.05, "{}", tilt(&scene, wall));
    assert_eq!(scene.impact_breaks, 1);
    assert!(scene.piece_of_brick(8).is_none(), "its open end broke");
    assert!(
        scene.piece_of_brick(7).is_some(),
        "the brick it stands on is whole"
    );
}

/// Bricks cut out around the top right corner leave it a piece of its own, which falls as
/// one body onto the row below the cut, lands hard enough to break there, and the wall
/// stands.
#[test]
fn a_cut_loose_corner_falls_as_one_body_and_breaks_where_it_lands() {
    let mut scene = scene();
    let wall = scene.pieces[0].body;
    let cut: Vec<(u32, Vec3)> = CORNER_CUT.map(|brick| (brick, OUT)).to_vec();
    scene.knock_out(cut).unwrap();
    assert_eq!(scene.pieces.len(), 2);
    let corner = scene.pieces[1].body;
    let ids: Vec<u32> = scene.pieces[1].bricks.iter().map(|&(id, _)| id).collect();
    assert_eq!(ids, [38, 39, 46, 47]);
    assert_eq!(scene.pieces[0].bricks.len(), 48 - 5 - 4);
    clear_debris(&mut scene);
    let start = position(scene.world.body(corner).unwrap().position());
    // The gap below it is a brick high, 0.24 m, which it falls in about 13 ticks.
    run(&mut scene, 10);
    let now = position(scene.world.body(corner).unwrap().position());
    assert!(start.y - now.y > 0.1, "{start} {now}");
    assert_eq!(scene.pieces[1].bricks.len(), 4);
    assert_eq!(scene.impact_breaks, 0);
    run(&mut scene, 50);
    assert_eq!(scene.impact_breaks, 1);
    let corner_bricks = scene
        .pieces
        .iter()
        .find(|piece| piece.body == corner)
        .map_or(0, |piece| piece.bricks.len());
    assert!(corner_bricks < 4, "{corner_bricks}");
    assert!(tilt(&scene, wall) < 0.05, "{}", tilt(&scene, wall));
}

/// Without its three lower rows, the top of the wall falls flat onto the ground from
/// 0.72 m and breaks at an end of its lowest row, where a corner of the contact hit hardest.
#[test]
fn a_falling_wall_breaks_where_it_lands() {
    let mut scene = scene();
    let lower: Vec<(u32, Vec3)> = (0..3 * WALL[0]).map(|brick| (brick, OUT)).collect();
    scene.knock_out(lower).unwrap();
    clear_debris(&mut scene);
    assert_eq!(scene.pieces.len(), 1);
    let mut ticks = 0;
    while scene.impact_breaks == 0 && ticks < 60 {
        assert_eq!(scene.pieces.len(), 1, "it falls as one piece");
        run(&mut scene, 1);
        ticks += 1;
    }
    assert_eq!(scene.impact_breaks, 1, "it landed within {ticks} ticks");
    assert!(scene.brick_count() < 24, "{}", scene.brick_count());
    let an_end = scene.piece_of_brick(24).is_none() || scene.piece_of_brick(31).is_none();
    assert!(an_end, "an end of its lowest row broke out");
}

/// The top row and the two outer columns, one arch, dropped from a metre, land on both
/// feet at once. The break is at one foot, where the arch touched the ground, not in the
/// span between the feet.
#[test]
fn a_falling_arch_breaks_at_a_foot() {
    let mut scene = scene();
    let span = (0..5 * WALL[0]).filter(|id| id % WALL[0] != 0 && id % WALL[0] != WALL[0] - 1);
    scene.knock_out(span.map(|id| (id, OUT)).collect()).unwrap();
    clear_debris(&mut scene);
    assert_eq!(scene.pieces.len(), 1);
    let arch = scene.pieces[0].body;
    let raised = rvec([0.0, 1.0, 0.0]);
    scene
        .world
        .body_mut(arch)
        .unwrap()
        .set_position(raised, Activation::Activate)
        .unwrap();
    let mut ticks = 0;
    while scene.impact_breaks == 0 && ticks < 90 {
        run(&mut scene, 1);
        ticks += 1;
    }
    assert_eq!(scene.impact_breaks, 1, "it landed within {ticks} ticks");
    let feet = [0, WALL[0] - 1].map(|id| scene.piece_of_brick(id).is_none());
    assert!(feet[0] != feet[1], "one foot broke: {feet:?}");
    let top = 5 * WALL[0]..6 * WALL[0];
    assert!(top.into_iter().all(|id| scene.piece_of_brick(id).is_some()));
}

/// Without its three lower rows but the brick at their bottom right corner, the top of the
/// wall falls onto that loose brick with its right end, tips over it and breaks off the left
/// end, which lands on the ground. Loose bricks do not break pieces.
#[test]
fn a_wall_that_tips_over_a_loose_brick_breaks_where_it_lands() {
    let mut scene = scene();
    let lower: Vec<(u32, Vec3)> = (0..3 * WALL[0])
        .filter(|&brick| brick != WALL[0] - 1)
        .map(|brick| (brick, OUT))
        .collect();
    scene.knock_out(lower).unwrap();
    // The corner brick is left alone, the last loose brick.
    let support = scene.debris.pop_back().unwrap();
    clear_debris(&mut scene);
    scene.debris.push_back(support);
    let wall = scene.pieces[0].body;
    let mut ticks = 0;
    while scene.impact_breaks == 0 && ticks < 60 {
        run(&mut scene, 1);
        ticks += 1;
    }
    assert_eq!(scene.impact_breaks, 1, "it landed within {ticks} ticks");
    assert!(tilt(&scene, wall) > 0.03, "{}", tilt(&scene, wall));
    assert_eq!(scene.pieces.len(), 1);
    assert!(scene.piece_of_brick(24).is_none(), "its left end broke off");
    assert!(scene.piece_of_brick(31).is_some(), "its right end is whole");
    run(&mut scene, 60);
    assert_eq!(scene.impact_breaks, 1);
}

/// A wall that is republished meets the ground in new contacts while it stands, which
/// break nothing.
#[test]
fn a_standing_wall_does_not_break_on_its_new_contacts() {
    let mut scene = scene();
    scene.knock_out(vec![(44, OUT)]).unwrap();
    run(&mut scene, 30);
    scene.knock_out(vec![(43, OUT)]).unwrap();
    run(&mut scene, 90);
    assert_eq!(scene.impact_breaks, 0);
    assert_eq!((scene.pieces.len(), scene.brick_count()), (1, 46));
}

/// Once [`MAX_IMPACT_BREAKS`] impacts broke pieces, a landing breaks nothing more.
#[test]
fn impacts_stop_breaking_pieces_after_the_last_allowed_one() {
    let mut scene = scene();
    scene.impact_breaks = MAX_IMPACT_BREAKS;
    let lower: Vec<(u32, Vec3)> = (0..3 * WALL[0]).map(|brick| (brick, OUT)).collect();
    scene.knock_out(lower).unwrap();
    clear_debris(&mut scene);
    run(&mut scene, 60);
    assert_eq!(scene.impact_breaks, MAX_IMPACT_BREAKS);
    assert_eq!((scene.pieces.len(), scene.brick_count()), (1, 24));
}

/// Two hard impacts in one step break once: the corner of the wall lands on the stub with
/// every contact reported twice, and only the first spends the step's impact, the last one
/// allowed. That impact breaks both pieces where they touch.
#[test]
fn one_impact_per_step_breaks_pieces_the_last_allowed_one_included() {
    let mut scene = scene();
    scene
        .knock_out(CORNER_CUT.map(|brick| (brick, OUT)).to_vec())
        .unwrap();
    clear_debris(&mut scene);
    let (wall, corner) = (scene.pieces[0].body, scene.pieces[1].body);
    scene.impact_breaks = MAX_IMPACT_BREAKS - 1;
    for _ in 0..60 {
        let before = scene.piece_motions().unwrap();
        step(&mut scene.world).unwrap();
        let events = scene.world.take_events();
        scene.tracked.sync(&scene.world, &events);
        let once = scene.hit_bricks(&events.contacts, &before).unwrap();
        if scene.impact_breaks == MAX_IMPACT_BREAKS - 1 {
            scene.knock_out(once).unwrap();
            continue;
        }
        scene.impact_breaks = MAX_IMPACT_BREAKS - 1;
        let twice = [events.contacts.clone(), events.contacts].concat();
        let hits = scene.hit_bricks(&twice, &before).unwrap();
        assert_eq!(scene.impact_breaks, MAX_IMPACT_BREAKS);
        let ids = |hits: &[(u32, Vec3)]| {
            let mut ids: Vec<u32> = hits.iter().map(|&(id, _)| id).collect();
            ids.sort_unstable();
            ids.dedup();
            ids
        };
        assert_eq!(ids(&hits), ids(&once));
        let broken: Vec<BodyId> = ids(&hits)
            .into_iter()
            .map(|id| scene.piece_of_brick(id).unwrap().0.body)
            .collect();
        assert!(
            broken.contains(&wall) && broken.contains(&corner),
            "{broken:?}"
        );
        scene.knock_out(hits).unwrap();
        run(&mut scene, 60);
        assert_eq!(scene.impact_breaks, MAX_IMPACT_BREAKS);
        return;
    }
    panic!("the corner never landed hard");
}

#[test]
fn groups_follow_face_contact_in_the_laid_wall() {
    let scene = scene();
    let bricks = &scene.pieces[0].bricks;
    assert_eq!(touching_groups(bricks), vec![(0..48).collect::<Vec<u32>>()]);
    // Brick 8 (row 1, column 0) is shifted right, so it touches bricks 0 and 1 below it
    // but not brick 2.
    let at = |id: u32| bricks.iter().find(|&&(b, _)| b == id).unwrap().1;
    assert!(touch(at(8), at(0)) && touch(at(8), at(1)) && !touch(at(8), at(2)));
    assert!(touch(at(0), at(1)) && !touch(at(0), at(2)));
    let split: Vec<Brick> = bricks
        .iter()
        .copied()
        .filter(|&(id, _)| !CORNER_CUT.contains(&id))
        .collect();
    let groups = touching_groups(&split);
    assert_eq!(groups.len(), 2);
    assert_eq!(groups[1], [38, 39, 46, 47]);
}

/// The digest of `ticks` scripted ticks of `session`.
fn scripted(session: &mut crate::session::Session, ticks: u32) -> u64 {
    let mut digest = Digest::default();
    let mut list = DrawList::default();
    for _ in 0..ticks {
        session.tick_scripted().unwrap();
        session.write_state(&mut digest).unwrap();
        session.draw(&mut list).unwrap();
        digest.draw_list(&list);
    }
    digest.finish()
}

#[test]
fn reset_after_complete_destruction_rebuilds_the_wall() {
    use crate::scene::SceneKind;
    use crate::session::Session;
    let config = SceneConfig {
        worker_threads: Some(1),
        ..SceneConfig::default()
    };
    let fresh_bodies = scene().world.body_count();
    let mut fresh = Session::new(SceneKind::Destruction, config.clone()).unwrap();
    let first = scripted(&mut fresh, 60);
    let mut session = Session::new(SceneKind::Destruction, config).unwrap();
    // Click every brick from the front, top row first, so that what is left stands on a
    // full bottom row until the last brick goes.
    let mut bricks = scene().pieces.remove(0).bricks;
    bricks.sort_by_key(|&(id, _)| std::cmp::Reverse(id / WALL[0]));
    let hud = |session: &mut Session| {
        let mut list = DrawList::default();
        session.draw(&mut list).unwrap();
        list.hud[0].clone()
    };
    for &(brick, _) in &bricks {
        let input = Input {
            edges: Edges {
                pick: Some(click_at(brick)),
                ..Edges::default()
            },
            ..Input::default()
        };
        session.tick(input).unwrap();
    }
    assert!(
        hud(&mut session).starts_with("wall: 0 bricks"),
        "{}",
        hud(&mut session)
    );
    assert!(session.scene().world().body_count() > fresh_bodies);

    session.reset().unwrap();
    assert_eq!(session.scene().world().body_count(), fresh_bodies);
    assert!(hud(&mut session).starts_with("wall: 48 bricks, published 0 times"));
    assert_eq!(scripted(&mut session, 60), first);
}

#[test]
fn a_hit_decodes_to_the_brick_at_its_child() {
    let scene = scene();
    let ray = RayCast::new(
        rvec([0.0, 2.0 * BRICK[1] * 2.5, 2.0]),
        Vec3::new(0.0, 0.0, -4.0),
    );
    let picked = scene.picked_brick(ray).unwrap().unwrap();
    let (_, at) = scene.piece_of_brick(picked).unwrap();
    assert!((at[1] - 2.0 * BRICK[1] * 2.5).abs() <= BRICK[1]);
    assert!(at[0].abs() <= 2.0 * BRICK[0]);
}
