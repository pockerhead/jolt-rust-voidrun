//! The smallest Avian program: a ball dropped on a ground for one second, in a headless Bevy
//! app. Built alone, it measures the build time and binary size the engine brings.

use std::time::Duration;

use avian3d::prelude::*;
use bevy::app::PluginsState;
use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;

fn main() {
    let step = Duration::from_secs_f64(1.0 / 60.0);
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, TransformPlugin, PhysicsPlugins::default()))
        .insert_resource(Time::<Fixed>::from_duration(step))
        .insert_resource(TimeUpdateStrategy::ManualDuration(step));
    app.world_mut().spawn((
        RigidBody::Static,
        Collider::cuboid(100.0, 1.0, 100.0),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));
    let ball = app
        .world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            Transform::from_xyz(0.0, 5.0, 0.0),
        ))
        .id();
    // `App::run` would do this; a manually updated app finishes its plugins itself.
    while app.plugins_state() != PluginsState::Ready {
        bevy::tasks::tick_global_task_pools_on_main_thread();
    }
    app.finish();
    app.cleanup();
    for _ in 0..61 {
        app.update();
    }
    let height = app.world().get::<Position>(ball).map_or(f32::NAN, |p| p.y);
    println!("height after one second: {height}");
}
