//! Scenes and listeners shared by the event tests.

use joltphysics::*;

use super::soft_body::Cloth;
use super::step;

pub fn contacts() -> EventSettings {
    EventSettings::default().contacts(true)
}

pub fn every_event() -> EventSettings {
    EventSettings::default()
        .persisted_contacts(true)
        .body_activation(true)
        .soft_body_contacts(true)
        .soft_body_validations(true)
}

/// A static compound of two 2 x 1 x 2 boxes side by side along X, user data 10 and 11, top face
/// at y = 0.
pub fn add_two_box_floor(world: &mut PhysicsWorld) -> BodyId {
    let half = Shape::new_box(Vec3::new(1.0, 0.5, 1.0)).unwrap();
    let child = |x, user_data| CompoundChild {
        shape: &half,
        position: Vec3::new(x, 0.0, 0.0),
        rotation: Quat::IDENTITY,
        user_data,
    };
    let floor = Shape::new_compound(&[child(-1.0, 10), child(1.0, 11)]).unwrap();
    world
        .create_body(
            &floor,
            &BodySettings::new_static().position(RVec3::new(0.0, -0.5, 0.0)),
        )
        .unwrap()
}

/// A dynamic cube of half extent 0.25 at `position`.
pub fn add_small_cube(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.25, 0.25, 0.25)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_dynamic().position(position))
        .unwrap()
}

pub fn kinds(events: &WorldEvents) -> Vec<char> {
    events
        .contacts
        .iter()
        .map(|event| match event {
            ContactEvent::Added { .. } => 'a',
            ContactEvent::Persisted { .. } => 'p',
            ContactEvent::Removed(_) => 'r',
        })
        .collect()
}

pub fn lift(world: &mut PhysicsWorld, id: BodyId, position: RVec3) {
    world
        .body_mut(id)
        .unwrap()
        .set_position(position, Activation::Activate)
        .unwrap();
}

/// A static 4 x 1 x 4 box with its top face at y = 0, at `x`.
pub fn add_table(world: &mut PhysicsWorld, x: Real) -> BodyId {
    let shape = Shape::new_box(Vec3::new(2.0, 0.5, 2.0)).unwrap();
    world
        .create_body(
            &shape,
            &BodySettings::new_static().position(RVec3::new(x, -0.5, 0.0)),
        )
        .unwrap()
}

pub fn add_cloth(world: &mut PhysicsWorld, position: RVec3, rotation: Quat) -> BodyId {
    let cloth = Cloth::new(6, 0.2);
    let settings = cloth
        .builder()
        .create_constraints(SoftBodyBendType::None, SoftBodyVertexAttributes::default())
        .build()
        .unwrap();
    world
        .create_soft_body(
            &settings,
            &SoftBodySettings::default()
                .position(position)
                .rotation(rotation),
        )
        .unwrap()
}

/// Steps `ticks` times and returns every soft body contact snapshot.
pub fn soft_contacts(world: &mut PhysicsWorld, ticks: usize) -> Vec<SoftBodyContacts> {
    let mut contacts = Vec::new();
    for _ in 0..ticks {
        step(world, 1);
        contacts.extend(world.take_events().soft_body_contacts);
    }
    contacts
}

/// A listener that changes nothing.
pub struct NoOp;

impl ContactListener for NoOp {}

/// Doubles the friction of every contact, within the bound.
pub struct RoughContacts;

impl ContactListener for RoughContacts {
    fn contact_added(&self, manifold: &ContactManifold, settings: &mut ContactSettings) {
        self.contact_persisted(manifold, settings);
    }

    fn contact_persisted(&self, _: &ContactManifold, settings: &mut ContactSettings) {
        let doubled = (2.0 * settings.combined_friction()).min(limits::MAX_FRICTION);
        settings.set_combined_friction(doubled).unwrap();
    }
}
