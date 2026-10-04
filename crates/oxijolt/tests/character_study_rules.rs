//! The study caller's rule 7 (spec D.2): steep terrain is a wall the character slides down, a
//! steep structure (a step edge reads steep) holds it, and the terrain veto stands after the
//! autostep and the floor snap. `docs/character-study.md` describes the study.

mod common;
mod study;

use common::math::{norm, sub};
use common::walker::{from_y_to, Carry, G};
use common::DT;
use oxijolt::*;
use study::config::{Config, Rule7};
use study::controller::{carried, ends_grounded, feed, tick, Held, TickReport};
use study::scenes::{Scene, SceneKey};

/// Under spec D.2's rule 7, steep terrain support vetoes every way of grounding, the autostep
/// and the snap included, so a snapped tick on steep terrain keeps its downward vel_up and the
/// next tick falls faster. The reference near step's rule lets a successful autostep or snap
/// ground the character on steep terrain; only the `walker` row uses it.
#[test]
fn the_terrain_veto_stands_after_the_autostep_and_the_snap() {
    let none = Held {
        stepped: false,
        snapped: false,
        jolt: false,
    };
    let stepped = Held {
        stepped: true,
        ..none
    };
    let snapped = Held {
        snapped: true,
        ..none
    };
    let jolt = Held { jolt: true, ..none };
    let falling = feed(0.0, true, true);
    assert_eq!(
        falling,
        -G * DT,
        "grounded and walking feeds one tick of gravity"
    );
    for held in [stepped, snapped, jolt] {
        assert!(!ends_grounded(Rule7::Spec, falling, held, true), "{held:?}");
        assert!(ends_grounded(Rule7::Spec, falling, held, false), "{held:?}");
        assert!(
            !ends_grounded(Rule7::Spec, 0.1, held, false),
            "rising: {held:?}"
        );
    }
    assert!(!ends_grounded(Rule7::Spec, falling, none, false));

    for held in [stepped, snapped] {
        assert!(
            ends_grounded(Rule7::Reference, falling, held, true),
            "{held:?}"
        );
    }
    assert!(!ends_grounded(Rule7::Reference, falling, jolt, true));

    let grounded = ends_grounded(Rule7::Spec, falling, snapped, true);
    let vel_up = carried(falling, grounded, false);
    assert_eq!(vel_up, falling, "the slide keeps its vel_up");
    assert!(feed(vel_up, grounded, true) < vel_up, "and falls faster");
}

/// Standing still on a 50 degree face. On a structure (a sharp box ramp) Jolt reports
/// OnSteepGround and the character stays grounded with vel_up 0 and does not move; on terrain (a
/// heightfield plane) it is never grounded and slides with vel_up falling every tick. Checked
/// for `spec-d2` (the Q4 cast decides), `recommended` (Jolt's ground state decides) and
/// `walker`.
#[test]
fn a_steep_structure_holds_the_character_and_steep_terrain_does_not() {
    let structure = SceneKey::BoxRamp {
        deg10: 500,
        tilted: false,
    };
    for config in [Config::spec_d2(), Config::recommended(), Config::walker()] {
        let name = config.name;
        let run = still_run(structure, &config);
        assert!(
            run.iter()
                .all(|r| r.ground == GroundState::OnSteepGround && !r.on_terrain),
            "{name}: the structure must read steep for this case"
        );
        for (t, r) in run.iter().enumerate() {
            assert!(
                r.grounded && !r.sliding && r.vel_up == 0.0,
                "{name} on the structure, tick {t}: grounded {} sliding {} vel_up {}",
                r.grounded,
                r.sliding,
                r.vel_up
            );
        }
        let drift = norm(sub(run.last().unwrap().end, run[0].start));
        assert!(drift < 1e-3, "{name} on the structure drifted {drift} m");

        let run = still_run(SceneKey::plane(50.0), &config);
        assert!(run[0].sliding, "{name} on terrain: tick 0 slides");
        for (t, r) in run.iter().enumerate() {
            assert!(!r.grounded, "{name} on terrain, tick {t}: grounded");
            if t > 0 {
                assert!(
                    r.vel_up < run[t - 1].vel_up,
                    "{name} on terrain, tick {t}: vel_up {} after {}",
                    r.vel_up,
                    run[t - 1].vel_up
                );
            }
        }
        let fallen = common::math::dot(sub(run[0].start, run.last().unwrap().end), run[0].up);
        assert!(fallen > 0.5, "{name} on terrain fell only {fallen} m");
    }
}

/// 60 ticks of `config` standing still from the rest pose at the middle of `key`.
fn still_run(key: SceneKey, config: &Config) -> Vec<TickReport> {
    let mut scene = Scene::build(key);
    let start = study::laws::resting(&scene, 0.0, 0.0);
    let walker = config.create_character(&mut scene, start);
    let mut carry = Carry::RESTING;
    (0..60)
        .map(|_| {
            let report = tick(&mut scene, &walker, config, &mut carry, [0.0; 3], None);
            scene.place_actor(report.end, from_y_to(report.up));
            report
        })
        .collect()
}
