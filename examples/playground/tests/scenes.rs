//! Every scene run headless: scripts reach their milestones and repeat bit for bit with one
//! and four workers and after other scenes, resets rebuild the same scene, and drawn poses
//! match the bodies.

use oxijolt::{BodySettings, EventSettings, Vec3};
use playground::cli::DEFAULT_FRAMES;
use playground::digest::Digest;
use playground::draw::{colours, DrawList};
use playground::headless;
use playground::math::{position_f32, rvec};
use playground::scene::{new_world, step, SceneConfig, SceneKind};
use playground::session::Session;
use playground::tracked::Tracked;
use playground::visual::{Shaped, Visuals};

fn threads(n: u32) -> SceneConfig {
    SceneConfig {
        worker_threads: Some(n),
    }
}

/// The default headless run covers every scene's whole clip, so it checks every milestone.
#[test]
fn the_default_run_covers_every_clip() {
    for kind in SceneKind::ALL {
        let session = Session::new(kind, SceneConfig::default()).unwrap();
        assert!(
            session.scene().record_ticks() <= DEFAULT_FRAMES,
            "{}",
            kind.name()
        );
    }
}

/// A run past the end of the clip fails on a missed milestone, so both runs reached them all.
#[test]
fn every_script_reaches_its_milestones_alike_with_one_and_four_workers() {
    for kind in SceneKind::ALL {
        let one = headless::run(kind, DEFAULT_FRAMES, threads(1))
            .unwrap_or_else(|error| panic!("{error}"));
        let four = headless::run(kind, DEFAULT_FRAMES, threads(4))
            .unwrap_or_else(|error| panic!("{error}"));
        assert_eq!(one, four, "{}", kind.name());
        assert!(one.bodies > 0, "{one}");
    }
}

#[test]
fn a_scene_run_alone_matches_the_run_after_another_scene() {
    let pile = SceneKind::from_name("pile").unwrap();
    let alone = headless::run(pile, 120, threads(2)).unwrap();
    for kind in SceneKind::ALL {
        headless::run(kind, 30, threads(2)).unwrap();
    }
    assert_eq!(headless::run(pile, 120, threads(2)).unwrap(), alone);
}

/// The digest of a session's state and draw list after `ticks` scripted ticks.
fn run_session(session: &mut Session, ticks: u32) -> u64 {
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

/// A reset after the whole script, with everything it pressed and spawned, starts over.
#[test]
fn reset_builds_the_same_scene() {
    for kind in SceneKind::ALL {
        let mut session = Session::new(kind, threads(1)).unwrap();
        let first = run_session(&mut session, 60);
        let rest = session.scene().record_ticks() - 60;
        run_session(&mut session, rest);
        session.reset().unwrap();
        assert_eq!(session.tick_count(), 0);
        assert_eq!(run_session(&mut session, 60), first, "{}", kind.name());
    }
}

#[test]
fn every_scene_lists_its_controls() {
    for kind in SceneKind::ALL {
        let session = Session::new(kind, SceneConfig::default()).unwrap();
        let scene = session.scene();
        assert!(!scene.controls().is_empty(), "{}", kind.name());
        assert!(!scene.milestones().all().is_empty(), "{}", kind.name());
        assert!(scene.record_ticks() >= 60);
    }
}

#[test]
fn a_body_that_falls_asleep_is_drawn_at_rest() {
    let (mut world, layers) = new_world(&threads(1), 1, EventSettings::default()).unwrap();
    let mut visuals = Visuals::new(1);
    let mut tracked = Tracked::default();
    let ground = BodySettings::new_static().object_layer(layers.ground);
    tracked
        .spawn(
            &mut world,
            &Shaped::plane(10.0).unwrap(),
            &ground,
            &mut visuals,
            colours::GROUND,
        )
        .unwrap();
    let falling = BodySettings::new_dynamic()
        .position(rvec([0.0, 2.0, 0.0]))
        .object_layer(layers.moving);
    let cube = Shaped::cuboid([0.5; 3]).unwrap();
    let body = tracked
        .spawn(&mut world, &cube, &falling, &mut visuals, colours::BODY)
        .unwrap();
    let mut asleep = false;
    for _ in 0..600 {
        step(&mut world).unwrap();
        let events = world.take_events();
        tracked.sync(&world, &events);
        if events
            .activations
            .iter()
            .any(|event| matches!(event, oxijolt::ActivationEvent::Deactivated(id) if *id == body))
        {
            asleep = true;
            break;
        }
    }
    assert!(asleep, "the cube falls asleep");
    assert!(!world.active_body_poses().iter().any(|pose| pose.id == body));
    let reading = world.body(body).unwrap();
    let mut list = DrawList::default();
    tracked.draw(&mut list);
    let drawn = list.solids[1];
    assert_eq!(drawn.position, position_f32(reading.position()));
    assert_eq!(drawn.rotation, <[f32; 4]>::from(reading.rotation()));
}

#[test]
fn the_pose_cache_matches_the_bodies() {
    let (mut world, layers) = new_world(&threads(1), 1, EventSettings::default()).unwrap();
    let mut visuals = Visuals::new(1);
    let mut tracked = Tracked::default();
    let ground = BodySettings::new_static().object_layer(layers.ground);
    tracked
        .spawn(
            &mut world,
            &Shaped::plane(10.0).unwrap(),
            &ground,
            &mut visuals,
            colours::GROUND,
        )
        .unwrap();
    let cube = Shaped::cuboid([0.3; 3]).unwrap();
    for i in 0..12 {
        let settings = BodySettings::new_dynamic()
            .position(rvec([
                (i % 4) as f32 * 0.7 - 1.0,
                0.4 + (i / 4) as f32 * 0.65,
                0.0,
            ]))
            .linear_velocity(Vec3::new(0.0, 0.0, if i == 5 { 2.0 } else { 0.0 }))
            .object_layer(layers.moving);
        tracked
            .spawn(&mut world, &cube, &settings, &mut visuals, colours::BODY)
            .unwrap();
    }
    let mut saw_sleep = false;
    for _ in 0..400 {
        step(&mut world).unwrap();
        let events = world.take_events();
        saw_sleep |= events
            .activations
            .iter()
            .any(|event| matches!(event, oxijolt::ActivationEvent::Deactivated(_)));
        tracked.sync(&world, &events);
        for body in tracked.bodies() {
            let reading = world.body(body).unwrap();
            assert_eq!(
                tracked.pose(body),
                Some((reading.position(), reading.rotation()))
            );
        }
    }
    assert!(saw_sleep, "the run includes sleep transitions");
}

#[cfg(feature = "debug-renderer")]
#[test]
fn the_wireframe_of_a_scene_is_capped() {
    use oxijolt::{DebugLineSettings, DebugLines, QueryFilter};
    let session = Session::new(SceneKind::from_name("queries").unwrap(), threads(1)).unwrap();
    let world = session.scene().world();
    let centre = rvec(session.scene().camera().target);
    let mut lines = DebugLines::new();
    world
        .debug_lines_into(
            &DebugLineSettings::new(centre, 12.0),
            &QueryFilter::new(),
            &mut lines,
        )
        .unwrap();
    assert!(lines.lines().len() > 1000, "{}", lines.lines().len());
    assert!(!lines.is_truncated());
    world
        .debug_lines_into(
            &DebugLineSettings::new(centre, 12.0).max_lines(1000),
            &QueryFilter::new(),
            &mut lines,
        )
        .unwrap();
    assert_eq!(lines.lines().len(), 1000);
    assert!(lines.is_truncated());
}
