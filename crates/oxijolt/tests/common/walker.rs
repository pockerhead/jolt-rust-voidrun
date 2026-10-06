//! Reference implementation of VOIDRUN's near-band step (spec D.2) on CharacterVirtual; the game
//! keeps its own copy, as with the query API of spec C.2.
//!
//! The fixtures are built here on the binding, with no game code: a planet of radius 99 whose
//! centre is at `(0, -99, 0)`, so up is radial; chunks with a heightfield terrain body and
//! compound structure bodies; and a walker made of a character and the game's own kinematic
//! actor capsule.
//!
//! Positions in this module are the game's body origin: the centre of the capsule's lower
//! sphere, 0.4 above the feet. At rest on flat ground the origin is 0.42 above the surface,
//! foot 0.4 plus padding 0.02.

use oxijolt::*;

use super::math::*;
use super::{quat_about, Groups, DT};

/// Planet radius of the fixtures, metres.
pub const R: f64 = 99.0;
/// Gravity of the game, m/s², radial.
pub const G: f32 = 9.8;
/// Capsule radius.
pub const RADIUS: f32 = 0.4;
/// Half height of the capsule's cylinder.
pub const HALF_HEIGHT: f32 = 0.70845;
/// Height of the capsule centre above the body origin.
pub const CENTRE_UP: f32 = 0.70844734;
/// How far the character keeps from geometry.
pub const PADDING: f32 = 0.02;
/// Distance of the floor snap (stick to floor), metres.
pub const SNAP: f32 = 0.3;
/// Height above the feet beyond which a terrain surface counts as burying the walker.
pub const RECOVERY_THRESHOLD: f32 = 0.3;
/// Height of the body origin above the surface at rest: foot 0.4 plus padding.
pub const REST_HEIGHT: f32 = RADIUS + PADDING;
/// Height of the body origin above the character position along up. Jolt places the shape at
/// position + rotation * shape offset + padding * up (`CharacterVirtual::GetCenterOfMassPosition`),
/// and the shape offset is `CENTRE_UP` along the character's Y, so the lower sphere centre sits
/// `PADDING + CENTRE_UP - HALF_HEIGHT` above the position.
pub const ORIGIN_ABOVE_POSITION: f32 = PADDING + CENTRE_UP - HALF_HEIGHT;
/// The game's autostep (spec D.1): the highest step the walker climbs, above its feet, metres.
pub const STEP_HEIGHT: f32 = 0.45;
/// Free room the walker needs on top of a step before it climbs it, metres (spec D.1).
pub const STEP_MIN_WIDTH: f32 = 0.5;
/// How far the walker moves forward onto a step it climbs, at least, metres. The lower sphere
/// must end up over the step's edge so that the edge supports it: from the padding distance in
/// front of the face that takes `(RADIUS + PADDING) * (1 - sin 45°)` = 0.123 m.
pub const STEP_FORWARD: f32 = 0.15;

/// The planet's centre.
pub const CENTRE: V3 = [0.0, -R, 0.0];

/// Radial up at `p`.
pub fn up_at(p: V3) -> V3 {
    normalize(sub(p, CENTRE))
}

/// The shortest rotation that maps +Y to the unit vector `up`.
pub fn from_y_to(up: V3) -> Quat {
    // (Y x up, 1 + Y . up), normalised.
    let q = [up[2], 0.0, -up[0], 1.0 + up[1]];
    let length = q.iter().map(|c| c * c).sum::<f64>().sqrt();
    let [x, y, z, w] = q.map(|c| (c / length) as f32);
    Quat::from_xyzw(x, y, z, w)
}

/// The fixture's object layers: terrain heightfields, chunk compounds (structures and
/// features), dropped items and actor capsules.
#[derive(Clone, Copy, Debug)]
pub struct Layers {
    pub terrain: ObjectLayer,
    pub chunk: ObjectLayer,
    pub item: ObjectLayer,
    pub actor: ObjectLayer,
}

/// A world without engine gravity (gravity is radial and applied by the caller) whose solver
/// pairs exist only with items (spec A.3).
pub fn fixture_world(worker_threads: u32) -> (PhysicsWorld, Layers) {
    let mut collision = CollisionLayers::new(2);
    let fixed = BroadPhaseLayer::new(0);
    let moving = BroadPhaseLayer::new(1);
    let layers = Layers {
        terrain: collision.add_object_layer(fixed),
        chunk: collision.add_object_layer(fixed),
        item: collision.add_object_layer(moving),
        actor: collision.add_object_layer(moving),
    };
    for other in [layers.terrain, layers.chunk, layers.item, layers.actor] {
        collision.enable_collision(layers.item, other);
    }
    let world = PhysicsWorld::new(super::jobs::with_threads(
        WorldSettings::default()
            .gravity(Vec3::ZERO)
            .layers(collision),
        worker_threads,
    ))
    .unwrap();
    (world, layers)
}

/// Pose of a chunk centred on the planet surface at `angle` radians from the anchor (the top of
/// the planet) in the XY plane, with its local Y along the radial up there.
pub fn chunk_pose(angle: f64) -> (RVec3, Quat) {
    let position = add(CENTRE, [R * angle.sin(), R * angle.cos(), 0.0]);
    let rotation = quat_about(Vec3::new(0.0, 0.0, 1.0), -angle as f32);
    (rvec3(position), rotation)
}

/// Heightfield settings of spec B: a 33 x 33 field centred on its body origin, `side` metres
/// wide, with 16 bits per sample so slopes keep their shape.
fn field_settings(side: f32) -> HeightFieldSettings<'static> {
    let spacing = side / 32.0;
    HeightFieldSettings::default()
        .offset(Vec3::new(-side / 2.0, 0.0, -side / 2.0))
        .scale(Vec3::new(spacing, 1.0, spacing))
        .bits_per_sample(16)
}

/// A 33 x 33 heightfield 32 m wide whose heights `height(x, z)` are given in the chunk's local
/// frame.
pub fn height_field(height: impl Fn(f64, f64) -> f64) -> Shape {
    let mut samples = vec![0.0_f32; 33 * 33];
    for iz in 0..33 {
        for ix in 0..33 {
            let (x, z) = (ix as f64 - 16.0, iz as f64 - 16.0);
            samples[iz * 33 + ix] = height(x, z) as f32;
        }
    }
    Shape::new_height_field(33, &samples, &field_settings(32.0)).unwrap()
}

/// Flat terrain: the planet's surface, sampled in the local frame of a chunk centred on it.
pub fn flat_terrain() -> Shape {
    height_field(|x, z| (R * R - x * x - z * z).sqrt() - R)
}

/// Adds a static terrain body of `shape` at `pose`.
pub fn add_terrain(
    world: &mut PhysicsWorld,
    layers: &Layers,
    shape: &Shape,
    pose: (RVec3, Quat),
) -> BodyId {
    world
        .create_body(
            shape,
            &BodySettings::new_static()
                .position(pose.0)
                .rotation(pose.1)
                .object_layer(layers.terrain),
        )
        .unwrap()
}

/// The flat terrain of the chunk at `angle` from the anchor.
pub fn flat_chunk(world: &mut PhysicsWorld, layers: &Layers, angle: f64) -> BodyId {
    add_terrain(world, layers, &flat_terrain(), chunk_pose(angle))
}

/// A slope rising along +x from `c` with gradient `tan` (plateau after 12 m of run), flat before
/// it, as a heightfield in the anchor chunk's frame. Heights are relative to a plane, not the
/// sphere, so the slope angle is exact near the anchor.
pub fn sloped(c: f64, tan: f64) -> Shape {
    height_field(move |x, _| tan * (x - c).clamp(0.0, 12.0))
}

/// A static chunk compound holding one box of half extents `half` at `centre` in the anchor
/// frame, tilted by `tilt_z` radians about Z, with the structure group as its user data. The box
/// is sharp (convex radius 0), as the game's structures are (spec B).
pub fn structure_box(
    world: &mut PhysicsWorld,
    layers: &Layers,
    centre: V3,
    half: V3,
    tilt_z: f32,
) -> BodyId {
    let block = Shape::new_box_with_convex_radius(vec3(half), 0.0).unwrap();
    let chunk = Shape::new_compound(&[CompoundChild {
        shape: &block,
        position: Vec3::ZERO,
        rotation: quat_about(Vec3::new(0.0, 0.0, 1.0), tilt_z),
        user_data: Groups::STRUCTURE,
    }])
    .unwrap();
    world
        .create_body(
            &chunk,
            &BodySettings::new_static()
                .position(rvec3(centre))
                .object_layer(layers.chunk),
        )
        .unwrap()
}

/// What blocked the last move: the first colliding contact steeper than 45° in Jolt's order,
/// from the last solver pass only. A diagnostic.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Blocker {
    pub body: Option<BodyId>,
    /// The collision group: a compound child's user data, else the group of the body's layer.
    pub group: Option<u32>,
    pub normal: Vec3,
}

/// A walker: the character that moves and the game's own actor capsule, which the controller
/// ignores.
#[derive(Clone, Copy, Debug)]
pub struct Walker {
    pub id: CharacterId,
    pub actor: BodyId,
    pub layers: Layers,
    /// Autostep height, metres; `STEP_HEIGHT` unless a test measures another.
    pub step_height: f32,
    /// Whether the controller also sees the item layer (dynamic bodies). The game's controller
    /// mask has no item group; tests set this to show what happens otherwise.
    pub items_in_filter: bool,
}

/// The character settings of the game's controller (spec D.1).
pub fn controller_settings(capsule: &Shape) -> CharacterSettings<'_> {
    CharacterSettings::new(capsule)
        .shape_offset(Vec3::new(0.0, CENTRE_UP, 0.0))
        .character_padding(PADDING)
        .max_slope_angle(45.0_f32.to_radians())
        .enhanced_internal_edge_removal(true)
        // Contacts below the plane through the lower sphere centre support the character: the
        // same rule as Jolt's CharacterPlanetTest (`Plane(Y, -radius)` there, with its position
        // at the shape bottom). Here the lower sphere centre sits ORIGIN_ABOVE_POSITION above the
        // position, in the character's local space.
        .supporting_volume(Vec3::new(0.0, 1.0, 0.0), -ORIGIN_ABOVE_POSITION)
}

/// The capsule of the walker and of its actor body.
pub fn capsule() -> Shape {
    Shape::new_capsule(HALF_HEIGHT, RADIUS).unwrap()
}

/// The character position for body origin `origin` with up `up`.
pub fn position_for(origin: V3, up: V3) -> RVec3 {
    rvec3(sub(origin, scale(up, f64::from(ORIGIN_ABOVE_POSITION))))
}

/// The body origin of the walker now.
pub fn origin(world: &PhysicsWorld, walker: &Walker) -> V3 {
    character_origin(world, walker.id)
}

/// The body origin of character `id` now.
pub fn character_origin(world: &PhysicsWorld, id: CharacterId) -> V3 {
    let character = world.character(id).unwrap();
    add(
        v3(character.position()),
        scale(f3(character.up()), f64::from(ORIGIN_ABOVE_POSITION)),
    )
}

/// Creates a walker with its body origin at `origin`, upright on the planet.
pub fn add_walker(world: &mut PhysicsWorld, layers: &Layers, origin: V3) -> Walker {
    let capsule = capsule();
    let up = up_at(origin);
    let rotation = from_y_to(up);
    let actor_shape = Shape::new_compound(&[CompoundChild {
        shape: &capsule,
        position: Vec3::new(0.0, CENTRE_UP, 0.0),
        rotation: Quat::IDENTITY,
        user_data: Groups::ACTOR,
    }])
    .unwrap();
    let actor = world
        .create_body(
            &actor_shape,
            &BodySettings::new_kinematic()
                .position(rvec3(origin))
                .rotation(rotation)
                .object_layer(layers.actor),
        )
        .unwrap();
    let id = world
        .create_character(
            &controller_settings(&capsule).up(vec3(up)),
            position_for(origin, up),
            rotation,
        )
        .unwrap();
    let walker = Walker {
        id,
        actor,
        layers: *layers,
        step_height: STEP_HEIGHT,
        items_in_filter: false,
    };
    // A new character knows no ground; the game's walkers have been updated before their first
    // near step, so the fixture finds the ground once here.
    let filter_layers = [layers.terrain, layers.chunk, layers.actor];
    world
        .refresh_character_contacts(id, &controller_filter(&walker, &filter_layers))
        .unwrap();
    walker
}

/// Moves the actor capsule to the walker's pose, as the game does after all moves of a tick.
pub fn sync_actor(world: &mut PhysicsWorld, walker: &Walker) {
    let origin = origin(world, walker);
    let rotation = world.character(walker.id).unwrap().rotation();
    world
        .body_mut(walker.actor)
        .unwrap()
        .set_position_and_rotation(rvec3(origin), rotation, Activation::DontActivate)
        .unwrap();
}

/// The input of one near step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NearInput {
    /// The displacement wanted this tick, metres, tangent to the planet.
    pub desired: V3,
    /// Velocity along up carried from the last tick, m/s.
    pub vel_up: f32,
    /// Whether the walker was grounded after the last tick.
    pub grounded_prev: bool,
}

/// The result of one near step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NearOutput {
    /// The body origin after the step.
    pub pos: V3,
    pub grounded: bool,
    pub vel_up: f32,
    /// Realised displacement over dt, m/s; zero when nothing was wanted.
    pub velocity: V3,
    /// Standing on terrain steeper than 45°: falling and sliding along it.
    pub sliding: bool,
    /// A colliding contact whose normal points against up.
    pub ceiling: bool,
    pub blocker: Option<Blocker>,
    /// Whether the underground recovery moved the walker this tick.
    pub recovered: bool,
}

/// The controller's query filter: terrain, chunk compounds (structures and features) and actors,
/// without the walker's own actor capsule.
///
/// Items are not in the game's filter. With the item layer added, the autostep still never steps
/// onto a dynamic body (it skips dynamic contacts and dynamic tops), but `CharacterVirtual` itself
/// climbs a low dynamic edge: it creeps onto a sharp dynamic box up to about 0.37 m.
pub fn controller_filter<'a>(walker: &Walker, layers: &'a [ObjectLayer]) -> QueryFilter<'a> {
    QueryFilter::new()
        .object_layers(layers)
        .child_groups(1 << Groups::STRUCTURE | 1 << Groups::FEATURE | 1 << Groups::ACTOR)
        .exclude_body(walker.actor)
}

/// The underground recovery (D.2 rule 1): when the terrain surface lies more than
/// `RECOVERY_THRESHOLD` above the feet, the walker is put on it. Returns the surface point of
/// the terrain in object layer `terrain` that buries a walker with body origin `origin`.
pub fn buried_under(world: &PhysicsWorld, terrain: ObjectLayer, origin: V3, up: V3) -> Option<V3> {
    let feet = sub(origin, scale(up, f64::from(RADIUS)));
    let start = add(feet, scale(up, 50.0));
    let ball = Shape::new_sphere(0.05).unwrap();
    let terrain = [terrain];
    let cast = ShapeCast::new(&ball, rvec3(start), Quat::IDENTITY, vec3(scale(up, -50.0)));
    let hit = world
        .cast_shape(&cast, &QueryFilter::new().object_layers(&terrain))
        .unwrap()?;
    let surface = v3(hit.point);
    (dot(sub(surface, feet), up) > f64::from(RECOVERY_THRESHOLD)).then_some(surface)
}

/// One near step (spec D.2), a function of the world, the character's own state and `input`.
pub fn near_step(world: &mut PhysicsWorld, walker: &Walker, input: NearInput) -> NearOutput {
    let layers = [
        walker.layers.terrain,
        walker.layers.chunk,
        walker.layers.actor,
        walker.layers.item,
    ];
    let seen = if walker.items_in_filter { 4 } else { 3 };
    let filter = controller_filter(walker, &layers[..seen]);
    let mut vel_up = input.vel_up;
    let mut grounded_prev = input.grounded_prev;

    // Up is radial at the body origin, set with the rotation for this update.
    let start = origin(world, walker);
    let up = up_at(start);
    {
        let mut character = world.character_mut(walker.id).unwrap();
        character.set_up(vec3(up)).unwrap();
        character.set_rotation(from_y_to(up)).unwrap();
        character.set_position(position_for(start, up)).unwrap();
    }

    // 1. Underground recovery.
    let mut recovered = false;
    let mut old = start;
    if let Some(surface) = buried_under(world, walker.layers.terrain, start, up) {
        old = add(surface, scale(up, f64::from(REST_HEIGHT)));
        world
            .character_mut(walker.id)
            .unwrap()
            .set_position(position_for(old, up))
            .unwrap();
        world
            .refresh_character_contacts(walker.id, &filter)
            .unwrap();
        vel_up = 0.0;
        grounded_prev = true;
        recovered = true;
    }

    // 2. Depenetration: Jolt's penetration recovery resolves overlaps within the move.

    // 3. Vertical feed. A jump arrives as a positive vel_up and is airborne from its first tick.
    let walking = input.desired != [0.0; 3];
    if !grounded_prev || vel_up > 0.0 {
        vel_up -= G * DT;
    } else if walking {
        vel_up = -G * DT;
    } else {
        vel_up = 0.0;
    }
    let velocity = add(
        scale(input.desired, 1.0 / f64::from(DT)),
        scale(up, f64::from(vel_up)),
    );
    world
        .character_mut(walker.id)
        .unwrap()
        .set_linear_velocity(vec3(velocity))
        .unwrap();

    // 4. Move, with Jolt's walk stairs off (see `autostep`); 5. the floor snap is Jolt's stick to
    // floor, only after support and not rising.
    let snap = if grounded_prev && vel_up <= 0.0 {
        scale(up, -f64::from(SNAP))
    } else {
        [0.0; 3]
    };
    let extended = ExtendedUpdateSettings::default()
        .stick_to_floor_step_down(vec3(snap))
        .walk_stairs_step_up(Vec3::ZERO);
    world
        .update_character(
            walker.id,
            DT,
            vec3(scale(up, -f64::from(G))),
            &extended,
            &filter,
        )
        .unwrap();

    let character = world.character(walker.id).unwrap();
    let up32 = vec3(up);
    let along_up = |n: Vec3| n.x * up32.x + n.y * up32.y + n.z * up32.z;
    let contacts = character.active_contacts();
    let ceiling = contacts
        .iter()
        .any(|contact| contact.had_collision && along_up(contact.contact_normal) < 0.0);
    let cos_45 = std::f32::consts::FRAC_1_SQRT_2;
    let blocker = contacts
        .iter()
        .find(|contact| contact.had_collision && along_up(contact.contact_normal) < cos_45)
        .map(|contact| Blocker {
            body: contact.body,
            group: character
                .contact_compound_child(contact)
                .map(|child| child.user_data)
                .or_else(|| {
                    character
                        .contact_object_layer(contact)
                        .and_then(|layer| layer_group(&walker.layers, layer))
                }),
            normal: contact.contact_normal,
        });

    // The game's autostep, for a walker on the ground that a steep obstacle held back.
    let stepped = grounded_prev
        && vel_up <= 0.0
        && walking
        && autostep(world, walker, &filter, up, input.desired, old, &contacts);
    let character = world.character(walker.id).unwrap();

    // 6 and 7. Rising is airborne; steep terrain is a wall that the walker slides down, while a
    // steep structure contact (a step edge) still holds it.
    let ground = character.ground_state();
    let ground_layer = character.ground_body().and_then(|body| {
        contacts
            .iter()
            .find(|contact| contact.body == Some(body))
            .and_then(|contact| character.contact_object_layer(contact))
    });
    let on_terrain = ground_layer == Some(walker.layers.terrain);
    let sliding = ground == GroundState::OnSteepGround && on_terrain;
    let grounded = vel_up <= 0.0
        && (stepped
            || ground == GroundState::OnGround
            || (ground == GroundState::OnSteepGround && !on_terrain));

    // 8. vel_up carry.
    if grounded || (ceiling && vel_up > 0.0) {
        vel_up = 0.0;
    }

    // 9. Velocity.
    let pos = add(
        v3(character.position()),
        scale(f3(character.up()), f64::from(ORIGIN_ABOVE_POSITION)),
    );
    let velocity = if walking {
        scale(sub(pos, old), 1.0 / f64::from(DT))
    } else {
        [0.0; 3]
    };
    NearOutput {
        pos,
        grounded,
        vel_up,
        velocity,
        sliding,
        ceiling,
        blocker,
        recovered,
    }
}

/// The game's autostep (spec D.1) after a move that a steep obstacle held back: when the room
/// `STEP_MIN_WIDTH` ahead at `step_height` above the feet is free and has a walkable top below
/// it, the walker is put down on the step, `max(remaining step, STEP_FORWARD)` further on.
/// Returns whether it stepped.
///
/// Jolt's own walk stairs is not used for this: it judges a step by the surface normal at the
/// contact, and a capsule pressing on the face of a sharp box touches it at the box's top edge,
/// where `BoxShape::GetSurfaceNormal` picks the top face's normal or the side's by float rounding.
/// Jolt then misses some steps, climbs others, and lifts the walker onto edges it must not climb.
/// The casts here judge by the contact normal, which is the geometry's own.
fn autostep(
    world: &mut PhysicsWorld,
    walker: &Walker,
    filter: &QueryFilter<'_>,
    up: V3,
    desired: V3,
    start: V3,
    contacts: &[CharacterContact],
) -> bool {
    autostep_character(
        world,
        walker.id,
        walker.step_height,
        filter,
        up,
        desired,
        start,
        contacts,
    )
}

/// The autostep of character `id` for a step of at most `step_height` above the feet, after a
/// move from body origin `start` that wanted `desired` and ended with `contacts`.
#[allow(clippy::too_many_arguments)]
pub fn autostep_character(
    world: &mut PhysicsWorld,
    id: CharacterId,
    step_height: f32,
    filter: &QueryFilter<'_>,
    up: V3,
    desired: V3,
    start: V3,
    contacts: &[CharacterContact],
) -> bool {
    let cos_45 = std::f64::consts::FRAC_1_SQRT_2;
    let wanted = norm(desired);
    let along = scale(desired, 1.0 / wanted);
    let here = character_origin(world, id);
    let achieved = dot(sub(here, start), along).max(0.0);
    if achieved + 1e-4 >= wanted {
        return false;
    }
    // Step towards the steep obstacle pushed against most directly, as Jolt's walk stairs does
    // (contacts more than 75° off the walking direction do not count).
    let mut direction = None;
    let mut best = 75.0_f64.to_radians().cos();
    for contact in contacts {
        let solid = contact.had_collision
            && !contact.was_discarded
            && !contact.is_sensor
            && contact.motion_type != MotionType::Dynamic;
        let normal = f3(contact.contact_normal);
        let normal_up = dot(normal, up);
        let inward = sub(scale(up, normal_up), normal);
        let length = norm(inward);
        if !solid || normal_up >= cos_45 || length < 1e-6 {
            continue;
        }
        let inward = scale(inward, 1.0 / length);
        if dot(inward, along) > best {
            best = dot(inward, along);
            direction = Some(inward);
        }
    }
    let Some(direction) = direction else {
        return false;
    };

    let capsule = capsule();
    let rotation = from_y_to(up);
    let centre = |o: V3| rvec3(add(o, scale(up, f64::from(CENTRE_UP))));
    let cast = |from: V3, by: V3, padded: bool| {
        let query = ShapeCast::new(&capsule, centre(from), rotation, vec3(by));
        let query = if padded {
            query.target_distance(PADDING)
        } else {
            query
        };
        world.cast_shape(&query, filter).unwrap()
    };
    let walkable = |hit: &ShapeCastHit| {
        dot(f3(hit.normal), up) >= cos_45
            && world.body(hit.body).unwrap().motion_type() != MotionType::Dynamic
    };
    // Lifted so that the padded capsule's bottom, at the ground when resting, clears a top
    // `step_height` above the feet. The head room is tested with the bare capsule, which keeps
    // its padding from the walls beside it.
    let lift = f64::from(step_height + PADDING);
    if cast(here, scale(up, lift + f64::from(PADDING)), false).is_some() {
        return false;
    }
    let raised = add(here, scale(up, lift));
    if cast(raised, scale(direction, f64::from(STEP_MIN_WIDTH)), true).is_some() {
        return false;
    }
    let room = add(raised, scale(direction, f64::from(STEP_MIN_WIDTH)));
    match cast(room, scale(up, -lift), true) {
        Some(top) if walkable(&top) => {}
        _ => return false,
    }
    let forward = (wanted - achieved).max(f64::from(STEP_FORWARD));
    let ahead = add(raised, scale(direction, forward));
    let Some(landing) = cast(ahead, scale(up, -lift), true).filter(|hit| walkable(hit)) else {
        return false;
    };
    let landed = sub(ahead, scale(up, f64::from(landing.distance)));
    if dot(sub(landed, here), up) <= 1e-4 {
        return false;
    }
    world
        .character_mut(id)
        .unwrap()
        .set_position(position_for(landed, up))
        .unwrap();
    world.refresh_character_contacts(id, filter).unwrap();
    true
}

/// The group the fixture keeps in `layer`, for bodies that are not compounds.
fn layer_group(layers: &Layers, layer: ObjectLayer) -> Option<u32> {
    if layer == layers.terrain {
        Some(Groups::TERRAIN)
    } else if layer == layers.actor {
        Some(Groups::ACTOR)
    } else if layer == layers.item {
        Some(Groups::ITEM)
    } else {
        None
    }
}

/// What the game carries between near steps.
#[derive(Clone, Copy, Debug)]
pub struct Carry {
    pub vel_up: f32,
    pub grounded: bool,
}

impl Carry {
    pub const RESTING: Self = Self {
        vel_up: 0.0,
        grounded: true,
    };
}

/// One near step with `desired`, then the actor capsule follows the walker.
pub fn near_tick(
    world: &mut PhysicsWorld,
    walker: &Walker,
    carry: &mut Carry,
    desired: V3,
) -> NearOutput {
    let out = near_step(
        world,
        walker,
        NearInput {
            desired,
            vel_up: carry.vel_up,
            grounded_prev: carry.grounded,
        },
    );
    sync_actor(world, walker);
    carry.vel_up = out.vel_up;
    carry.grounded = out.grounded;
    out
}

/// The tangent of the planet at `origin` closest to world direction `direction`, scaled to
/// `metres`.
pub fn tangent(origin: V3, direction: V3, metres: f64) -> V3 {
    let up = up_at(origin);
    let flat = sub(direction, scale(up, dot(direction, up)));
    scale(normalize(flat), metres)
}

/// The game's player on top of the near step (spec D.3): ground speed by gait; in the air or
/// sliding the horizontal velocity is kept with friction 0.98 and control 0.05 per tick; a jump
/// sets vel_up to 4.5 when grounded and not sliding.
#[derive(Clone, Copy, Debug)]
pub struct Player {
    pub carry: Carry,
    pub last: NearOutput,
    /// Up at the start of the last step, along which that step's vertical motion went.
    pub last_up: V3,
}

impl Player {
    /// A player standing still for one tick, so it has a last output.
    pub fn new(world: &mut PhysicsWorld, walker: &Walker) -> Self {
        let mut carry = Carry::RESTING;
        let last_up = up_at(origin(world, walker));
        let last = near_tick(world, walker, &mut carry, [0.0; 3]);
        Self {
            carry,
            last,
            last_up,
        }
    }

    /// The horizontal part of the last realised velocity.
    pub fn horizontal_velocity(&self) -> V3 {
        let v = self.last.velocity;
        sub(v, scale(self.last_up, dot(v, self.last_up)))
    }

    /// One tick with `input` (a world direction or zero) at `speed` m/s; jumps when asked and
    /// allowed.
    pub fn tick(
        &mut self,
        world: &mut PhysicsWorld,
        walker: &Walker,
        input: V3,
        speed: f64,
        jump: bool,
    ) -> NearOutput {
        let on_ground = self.last.grounded && !self.last.sliding;
        let desired = if on_ground {
            if input == [0.0; 3] {
                [0.0; 3]
            } else {
                tangent(self.last.pos, input, speed * f64::from(DT))
            }
        } else {
            let control = if input == [0.0; 3] {
                [0.0; 3]
            } else {
                tangent(self.last.pos, input, speed)
            };
            scale(
                add(
                    scale(self.horizontal_velocity(), 0.98),
                    scale(control, 0.05),
                ),
                f64::from(DT),
            )
        };
        if jump && on_ground {
            self.carry.vel_up = 4.5;
        }
        self.last_up = up_at(origin(world, walker));
        self.last = near_tick(world, walker, &mut self.carry, desired);
        self.last
    }
}

/// The scene of the scripted runs (replay and determinism): terrain that is flat for x < 0 and
/// rises from x = 0 at 30° for z >= 0 and at 60° for z <= -1 (3 m of run, then a plateau), and
/// a 0.3 m structure step across the path at x -3 to -2.
pub fn script_scene(worker_threads: u32) -> (PhysicsWorld, Layers) {
    let (mut world, layers) = fixture_world(worker_threads);
    let (tan30, tan60) = (30.0_f64.to_radians().tan(), 60.0_f64.to_radians().tan());
    let terrain = height_field(|x, z| {
        let tan = if z > -0.5 { tan30 } else { tan60 };
        tan * x.clamp(0.0, 3.0)
    });
    add_terrain(&mut world, &layers, &terrain, (RVec3::ZERO, Quat::IDENTITY));
    structure_box(&mut world, &layers, [-2.5, -0.2, 1.0], [0.5, 0.5, 2.0], 0.0);
    (world, layers)
}

/// Where the scripted walker starts: resting on the flat part, 6 m before the rise.
pub fn script_start() -> V3 {
    let ground = [-6.0, 0.0, 1.0];
    add(ground, scale(up_at(ground), f64::from(REST_HEIGHT)))
}

/// The scripted input of tick `tick`: a world direction, a speed in m/s and whether to jump.
/// Jog over the step and up the 30° rise, walk back down and across to the 60° rise, jump at it
/// and slide down.
pub fn script_input(tick: usize) -> (V3, f64, bool) {
    match tick {
        0..=129 => ([1.0, 0.0, 0.0], 3.5, false),
        130..=189 => ([-1.0, 0.0, 0.0], 2.0, false),
        190..=259 => ([0.0, 0.0, -1.0], 2.0, false),
        260 => ([1.0, 0.0, 0.0], 3.5, true),
        261..=279 => ([1.0, 0.0, 0.0], 3.5, false),
        _ => ([0.0; 3], 0.0, false),
    }
}

/// One scripted tick of `player`.
pub fn script_tick(
    world: &mut PhysicsWorld,
    walker: &Walker,
    player: &mut Player,
    tick: usize,
) -> NearOutput {
    let (input, speed, jump) = script_input(tick);
    player.tick(world, walker, input, speed, jump)
}

/// Appends the walker's state after a tick to `digest`: the character state, the near step's
/// outputs and the active contacts (body, character, sub-shape and normal), as bits.
pub fn record_walker(
    world: &PhysicsWorld,
    walker: &Walker,
    out: &NearOutput,
    digest: &mut Vec<u8>,
) {
    let character = world.character(walker.id).unwrap();
    assert!(!character.max_hits_exceeded());
    digest.extend_from_slice(&character.save_state().as_bytes());
    for value in out.pos.iter().chain(&out.velocity) {
        digest.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    digest.extend_from_slice(&out.vel_up.to_bits().to_le_bytes());
    digest.extend_from_slice(&[u8::from(out.grounded), u8::from(out.sliding)]);
    for contact in character.active_contacts() {
        let body = contact.body.map_or(u32::MAX, BodyId::to_raw);
        let other = contact.character.map_or(u32::MAX, CharacterId::to_raw);
        digest.extend_from_slice(&body.to_le_bytes());
        digest.extend_from_slice(&other.to_le_bytes());
        digest.extend_from_slice(&contact.sub_shape_id.to_raw().to_le_bytes());
        let normal: [f32; 3] = contact.contact_normal.into();
        for value in normal {
            digest.extend_from_slice(&value.to_bits().to_le_bytes());
        }
    }
}
