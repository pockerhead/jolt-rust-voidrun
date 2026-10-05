//! A group filter table built before anything else of the crate ran in this process: the
//! builder initialises Jolt itself. This binary has one test, so nothing runs before it.

use oxijolt::{CollisionGroup, GroupFilterTableBuilder};

#[test]
fn a_table_built_before_any_world_works() {
    let mut builder = GroupFilterTableBuilder::new(3).unwrap();
    builder.disable_collision(0, 2).unwrap();
    let table = builder.build();
    assert!(!table.is_collision_enabled(2, 0).unwrap());
    assert!(table.is_collision_enabled(0, 1).unwrap());
    let a = CollisionGroup::new(&table, 1, 0).unwrap();
    let b = CollisionGroup::new(&table, 1, 2).unwrap();
    assert!(!a.can_collide(&b));
    drop((a, b));
    drop(table);
}
