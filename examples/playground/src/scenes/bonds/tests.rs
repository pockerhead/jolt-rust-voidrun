//! The bonds scene's tests.

use super::*;

fn scene_with(max_force: f32, max_torque: f32) -> Bonds {
    let config = SceneConfig {
        worker_threads: Some(1),
        ..SceneConfig::default()
    };
    Bonds::with_limits(&config, 1, max_force, max_torque).unwrap()
}

fn scene() -> Bonds {
    scene_with(MAX_BOND_FORCE, MAX_BOND_TORQUE)
}

fn run(scene: &mut Bonds, ticks: u32) {
    for _ in 0..ticks {
        scene.update(&Input::default()).unwrap();
    }
}

/// The bonds that are disabled, by index.
fn broken(scene: &Bonds) -> Vec<usize> {
    (0..scene.bonds.len())
        .filter(|&index| !scene.is_intact(index).unwrap())
        .collect()
}

fn column(end: End) -> Option<u32> {
    match end {
        End::Brick(brick) => Some(brick % WALL[0]),
        End::Pier => None,
    }
}

#[test]
fn the_wall_stands_while_propped() {
    let mut scene = scene();
    let anchors = scene.bonds.iter().filter(|b| b.ends.contains(&End::Pier));
    assert_eq!((scene.bonds.len(), anchors.count()), (119, 2));
    for _ in 0..RELEASE_FROM {
        scene.update(&Input::default()).unwrap();
        // The limits are at least twice what the propped wall puts on a bond.
        assert!(scene.highest_load < 0.5, "{}", scene.highest_load);
    }
    assert!(broken(&scene).is_empty());
    assert!(scene.milestones.is_reached("wall stands"));
    assert!(!scene.is_broken_in_two().unwrap());
    for &brick in &scene.bricks {
        let speed = glam(scene.world.body(brick).unwrap().linear_velocity()).length();
        assert!(speed < 0.01, "{speed}");
    }
}

/// The first bonds break after the prop starts to go down, on a step whose load is beyond
/// their limit, and every step before kept every load within it.
#[test]
fn lowering_the_prop_overloads_and_breaks_bonds() {
    let mut scene = scene();
    while scene.load_breaks == 0 {
        assert!(scene.tick < RELEASE_FROM + 60, "nothing broke");
        assert!(scene.highest_load <= 1.0, "{}", scene.highest_load);
        scene.update(&Input::default()).unwrap();
    }
    assert!(scene.tick > RELEASE_FROM, "{}", scene.tick);
    assert!(scene.milestones.is_reached("bond broke"));
    // A disabled bond keeps the readouts of the step that broke it.
    for index in broken(&scene) {
        assert!(scene.load(index).unwrap() > 1.0);
    }
}

/// The wall hangs from the pier, so the first bonds to break have a brick next to the pier.
#[test]
fn breaks_start_near_the_pier() {
    let mut scene = scene();
    while scene.load_breaks == 0 {
        assert!(scene.tick < RELEASE_FROM + 60, "nothing broke");
        scene.update(&Input::default()).unwrap();
    }
    for index in broken(&scene) {
        let columns = scene.bonds[index].ends.map(column);
        assert!(
            columns.iter().flatten().any(|&c| c <= PIER_COLUMNS),
            "{:?}",
            scene.bonds[index].ends
        );
    }
}

/// The bricks that lose their last chain of bonds to the pier fall, and the anchors to the
/// pier hold.
#[test]
fn the_free_end_falls_off() {
    let mut scene = scene();
    let ticks = scene.record_ticks();
    run(&mut scene, ticks);
    assert!(scene.is_broken_in_two().unwrap());
    assert!(scene.milestones.missing().is_empty());
    let anchors = (0..scene.bonds.len()).filter(|&i| scene.bonds[i].ends.contains(&End::Pier));
    for index in anchors {
        assert!(scene.is_intact(index).unwrap());
    }
    let mut fallen = 0;
    for (id, &brick) in scene.bricks.iter().enumerate() {
        let y = position(scene.world.body(brick).unwrap().position()).y;
        if laid_at(id as u32)[1] - y > 0.3 {
            fallen += 1;
        }
    }
    assert!(fallen > WALL[0] * WALL[1] / 2, "{fallen} bricks fell");
}

/// With limits that the first step exceeds on many bonds, the first step breaks exactly the
/// budget: the most loaded bonds.
#[test]
fn at_most_the_budget_breaks_per_step() {
    let mut scene = scene_with(100.0, 10.0);
    scene.update(&Input::default()).unwrap();
    let mut loads: Vec<(usize, f32)> = (0..scene.bonds.len())
        .filter(|&i| !scene.bonds[i].ends.contains(&End::Pier))
        .map(|i| (i, scene.load(i).unwrap()))
        .collect();
    let overloaded = loads.iter().filter(|(_, load)| *load > 1.0).count();
    assert!(overloaded > MAX_BREAKS_PER_STEP, "{overloaded}");
    loads.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    let mut most: Vec<usize> = loads[..MAX_BREAKS_PER_STEP].iter().map(|l| l.0).collect();
    most.sort_unstable();
    assert_eq!(broken(&scene), most);
    assert_eq!(scene.load_breaks, MAX_BREAKS_PER_STEP as u32);
}

/// Unbreakable limits: the prop goes down and no bond breaks, so only the load breaks bonds.
#[test]
fn unbreakable_thresholds_never_break() {
    let mut scene = scene_with(f32::MAX, f32::MAX);
    let ticks = scene.record_ticks();
    run(&mut scene, ticks);
    assert!(broken(&scene).is_empty());
    assert_eq!(scene.load_breaks, 0);
    assert!(scene.milestones.is_reached("wall stands"));
    assert!(!scene.milestones.is_reached("bond broke"));
    assert!(!scene.is_broken_in_two().unwrap());
}

/// A click breaks every bond of the brick under the cursor and no other.
#[test]
fn a_click_breaks_every_bond_of_the_brick() {
    let mut scene = scene();
    let brick = 20;
    let [x, y, _] = laid_at(brick);
    let ray = RayCast::new(rvec([x, y, 3.0]), Vec3::new(0.0, 0.0, -6.0));
    let input = Input {
        edges: crate::input::Edges {
            pick: Some(ray),
            ..Default::default()
        },
        ..Input::default()
    };
    scene.update(&input).unwrap();
    let of_brick: Vec<usize> = (0..scene.bonds.len())
        .filter(|&i| scene.bonds[i].ends.contains(&End::Brick(brick)))
        .collect();
    assert_eq!(of_brick.len(), 6, "two in its row, two above, two below");
    assert_eq!(broken(&scene), of_brick);
    assert_eq!(scene.click_breaks, 6);
}
