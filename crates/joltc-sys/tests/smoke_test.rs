//! Smoke tests for the raw bindings: a world can be created, stepped and read.

mod framework;

use framework::*;
use joltc_sys::*;

#[test]
fn setup_teardown() {
    drop(TestWorld::new(2));
}

#[test]
fn box_falls_onto_static_box() {
    let world = TestWorld::new(2);
    let bodies = world.body_interface();

    let floor = create_box(
        bodies,
        vec3(100.0, 1.0, 100.0),
        rvec3(0.0, -1.0, 0.0),
        JPH_MotionType_Static,
        OL_NON_MOVING,
        JPH_Activation_DontActivate,
    );
    let falling = create_box(
        bodies,
        vec3(0.5, 0.5, 0.5),
        rvec3(0.0, 2.0, 0.0),
        JPH_MotionType_Dynamic,
        OL_MOVING,
        JPH_Activation_Activate,
    );

    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let mut position = rvec3(0.0, 0.0, 0.0);
    // SAFETY: `bodies` belongs to the live `world`, `falling` is a body in it
    // and `position` is a live local.
    unsafe { JPH_BodyInterface_GetPosition(bodies, falling, &mut position) };

    assert!(
        position.x.is_finite() && position.y.is_finite() && position.z.is_finite(),
        "{position:?}"
    );
    assert!(position.x.abs() < 1e-3, "{position:?}");
    assert!(position.z.abs() < 1e-3, "{position:?}");
    // The floor's top face is at y = 0 and the box's half height is 0.5.
    assert!(position.y > 0.45 && position.y < 0.55, "{position:?}");

    // SAFETY: both ids are bodies of the live `world`, each removed once.
    unsafe {
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, falling);
        JPH_BodyInterface_RemoveAndDestroyBody(bodies, floor);
    }
}
