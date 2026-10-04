//! The study's caller: one fixed generalisation of the reference near step (spec D.2,
//! `tests/common/walker.rs`) whose mechanisms a [`Config`] switches.
//!
//! Order of a tick: up and pose; the underground recovery (Q3); the depenetration push (Q6); a
//! contact refresh; a still update; the vertical feed; the move (ExtendedUpdate); contact
//! readout; the autostep and the floor snap (Q5); classification (rules 6 and 7, with or without
//! the Q4 support normal); the carry (rule 8) and the velocity (rule 9). Where the reference near
//! step has a statement, the arithmetic and order here are the same, so the `walker` row
//! reproduces it bit for bit.

use std::cell::Cell;
use std::time::{Duration, Instant};

use oxijolt::*;

use super::config::{position_for, Config, Refresh, StickGate, Still};
use super::passes::{depenetrate, snap, terrain_support_normal};
use super::scenes::Scene;
use crate::common::math::{add, dot, f3, scale, sub, v3, vec3, V3};
use crate::common::walker::{
    autostep_character, buried_under, controller_filter, from_y_to, Blocker, Carry, Walker, G,
    RADIUS, REST_HEIGHT, STEP_HEIGHT,
};
use crate::common::{Groups, DT};

/// A contact as the trace keeps it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TracedContact {
    pub body: Option<u32>,
    pub sub_shape: u32,
    pub distance: f32,
    pub contact_normal: V3,
    pub surface_normal: V3,
    pub had_collision: bool,
}

/// What the trace adds to a tick: the contacts after the move and the passes' findings.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TraceTick {
    pub contacts: Vec<TracedContact>,
    pub q4_normal: Option<V3>,
    pub q5_distance: Option<f64>,
    pub q6_push: V3,
    pub q6_depth: f64,
}

/// Everything one tick did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickReport {
    pub start: V3,
    pub up: V3,
    /// The locomotion origin minus `start`: what the maintenance passes moved.
    pub maintenance: V3,
    /// What the move moved, from the locomotion origin.
    pub moving: V3,
    /// What the autostep and the snap moved after the move.
    pub post: V3,
    pub end: V3,
    /// Rule 9's velocity: displacement from the locomotion origin over dt, zero without input.
    pub velocity: V3,
    /// The velocity given to the move.
    pub supplied: Vec3,
    /// The character's velocity after the move, as Jolt left it.
    pub jolt_velocity: Vec3,
    pub vel_up: f32,
    pub grounded: bool,
    pub sliding: bool,
    pub ceiling: bool,
    pub blocker: Option<Blocker>,
    /// Ground state before the move (after maintenance), after the move and at the end.
    pub ground_before: GroundState,
    pub ground_after_move: GroundState,
    pub ground: GroundState,
    pub ground_body: Option<BodyId>,
    pub on_terrain: bool,
    pub recovered: bool,
    pub pushed: bool,
    pub deep: bool,
    pub still_ran: bool,
    pub stepped: bool,
    pub snapped: bool,
    pub max_hits_exceeded: bool,
    /// Groups (bit per `Groups` value) with a contact that had a collision after the move.
    pub touched_groups: u32,
}

thread_local! {
    /// The time this thread spent in the move's physics calls since `start_physics_timer`.
    static PHYSICS_TIME: Cell<Option<Duration>> = const { Cell::new(None) };
}

/// Starts summing the time the ticks on this thread spend in physics calls: contact refreshes,
/// updates, the passes' queries and the autostep.
pub fn start_physics_timer() {
    PHYSICS_TIME.with(|time| time.set(Some(Duration::ZERO)));
}

/// The time summed since `start_physics_timer`, which stops.
pub fn take_physics_time() -> Duration {
    PHYSICS_TIME.with(|time| time.replace(None).unwrap_or_default())
}

/// Runs the physics call `f`, timing it when the timer runs.
fn physics<T>(f: impl FnOnce() -> T) -> T {
    match PHYSICS_TIME.with(Cell::get) {
        None => f(),
        Some(sum) => {
            let start = Instant::now();
            let result = f();
            let elapsed = start.elapsed();
            PHYSICS_TIME.with(|time| time.set(Some(sum + elapsed)));
            result
        }
    }
}

/// The body origin of `walker` for padding `p`.
pub fn origin_of(world: &PhysicsWorld, walker: &Walker, p: f32) -> V3 {
    let character = world.character(walker.id).unwrap();
    add(
        v3(character.position()),
        scale(
            f3(character.up()),
            f64::from(super::config::origin_above_position(p)),
        ),
    )
}

/// One tick of `walker` under `cfg`, wanting `desired`.
pub fn tick(
    scene: &mut Scene,
    walker: &Walker,
    cfg: &Config,
    carry: &mut Carry,
    desired: V3,
    trace: Option<&mut TraceTick>,
) -> TickReport {
    let p = cfg.settings.padding;
    let layers = [scene.layers.terrain, scene.layers.chunk, scene.layers.actor];
    let filter = controller_filter(walker, &layers);
    let world = &mut scene.world;
    let mut vel_up = carry.vel_up;
    let mut grounded_prev = carry.grounded;

    let start = origin_of(world, walker, p);
    let up = scene.up.up_at(start);
    {
        let mut character = world.character_mut(walker.id).unwrap();
        character.set_up(vec3(up)).unwrap();
        character.set_rotation(from_y_to(up)).unwrap();
        character.set_position(position_for(start, up, p)).unwrap();
    }

    // Maintenance: underground recovery, depenetration push, refresh, still update.
    let mut moved = false;
    let mut recovered = false;
    let mut old = start;
    if cfg.passes.underground {
        if let Some(surface) = physics(|| buried_under(world, scene.layers.terrain, start, up)) {
            old = add(surface, scale(up, f64::from(REST_HEIGHT)));
            set_origin(world, walker, old, up, p);
            vel_up = 0.0;
            grounded_prev = true;
            recovered = true;
            moved = true;
        }
    }
    let (mut pushed, mut deep, mut q6_push, mut q6_depth) = (false, false, [0.0; 3], 0.0);
    if cfg.passes.q6 {
        (q6_push, q6_depth) = physics(|| depenetrate(world, &filter, old, up, p));
        deep = q6_depth > f64::from(RADIUS + p);
        if q6_push != [0.0; 3] {
            old = add(old, q6_push);
            set_origin(world, walker, old, up, p);
            pushed = true;
            moved = true;
        }
    }
    if cfg.refresh == Refresh::EveryTick || (cfg.refresh == Refresh::WhenMoved && moved) {
        physics(|| world.refresh_character_contacts(walker.id, &filter)).unwrap();
    }
    let still = match cfg.still {
        Still::Never => false,
        Still::Always => true,
        Still::WhenPenetrating => physics(|| world.character(walker.id).unwrap().active_contacts())
            .iter()
            .any(|contact| contact.had_collision && contact.distance < -0.001),
        Still::WhenDeep => deep,
    };
    if still {
        world
            .character_mut(walker.id)
            .unwrap()
            .set_linear_velocity(Vec3::ZERO)
            .unwrap();
        let settings = cfg.extended.update_settings(up, false);
        let gravity = vec3(scale(up, -f64::from(G)));
        physics(|| world.update_character(walker.id, DT, gravity, &settings, &filter)).unwrap();
        let after = origin_of(world, walker, p);
        if after != old {
            old = after;
        }
    }
    let ground_before = world.character(walker.id).unwrap().ground_state();

    // Vertical feed (rule 3), as the reference near step.
    let walking = desired != [0.0; 3];
    if !grounded_prev || vel_up > 0.0 {
        vel_up -= G * DT;
    } else if walking {
        vel_up = -G * DT;
    } else {
        vel_up = 0.0;
    }
    let velocity = add(
        scale(desired, 1.0 / f64::from(DT)),
        scale(up, f64::from(vel_up)),
    );
    let supplied = vec3(velocity);
    world
        .character_mut(walker.id)
        .unwrap()
        .set_linear_velocity(supplied)
        .unwrap();

    // The move.
    let stick = match cfg.stick_gate {
        StickGate::Jolt => true,
        StickGate::Caller => grounded_prev && vel_up <= 0.0,
    };
    let extended = cfg.extended.update_settings(up, stick);
    if let Some(speed) = cfg.moving_recovery {
        world
            .character_mut(walker.id)
            .unwrap()
            .set_penetration_recovery_speed(speed)
            .unwrap();
    }
    let gravity = vec3(scale(up, -f64::from(G)));
    physics(|| world.update_character(walker.id, DT, gravity, &extended, &filter)).unwrap();
    if cfg.moving_recovery.is_some() {
        world
            .character_mut(walker.id)
            .unwrap()
            .set_penetration_recovery_speed(cfg.settings.recovery_speed)
            .unwrap();
    }
    let after_move = origin_of(world, walker, p);

    // Contact readout (rule 4).
    let character = world.character(walker.id).unwrap();
    let up32 = vec3(up);
    let along_up = |n: Vec3| n.x * up32.x + n.y * up32.y + n.z * up32.z;
    let contacts = physics(|| character.active_contacts());
    let ground_after_move = character.ground_state();
    let jolt_velocity = character.linear_velocity();
    let max_hits_exceeded = character.max_hits_exceeded();
    let ceiling = contacts
        .iter()
        .any(|contact| contact.had_collision && along_up(contact.contact_normal) < 0.0);
    let cos_45 = std::f32::consts::FRAC_1_SQRT_2;
    let group_of = |contact: &CharacterContact| {
        character
            .contact_compound_child(contact)
            .map(|child| child.user_data)
            .or_else(|| {
                character
                    .contact_object_layer(contact)
                    .and_then(|layer| layer_group(scene.layers, layer))
            })
    };
    let blocker = contacts
        .iter()
        .find(|contact| contact.had_collision && along_up(contact.contact_normal) < cos_45)
        .map(|contact| Blocker {
            body: contact.body,
            group: group_of(contact),
            normal: contact.contact_normal,
        });
    let touched_groups = contacts
        .iter()
        .filter(|contact| contact.had_collision)
        .filter_map(group_of)
        .fold(0, |groups, group| groups | 1 << group);
    let traced: Vec<TracedContact> = if trace.is_some() {
        contacts
            .iter()
            .map(|contact| TracedContact {
                body: contact.body.map(BodyId::to_raw),
                sub_shape: contact.sub_shape_id.to_raw(),
                distance: contact.distance,
                contact_normal: f3(contact.contact_normal),
                surface_normal: f3(contact.surface_normal),
                had_collision: contact.had_collision,
            })
            .collect()
    } else {
        Vec::new()
    };

    // The autostep and the floor snap (rule 5).
    let stepped = cfg.passes.autostep
        && grounded_prev
        && vel_up <= 0.0
        && walking
        && physics(|| {
            autostep_character(
                world,
                walker.id,
                STEP_HEIGHT,
                &filter,
                up,
                desired,
                old,
                &contacts,
            )
        });
    let mut snapped = false;
    let mut q5_distance = None;
    if cfg.passes.q5
        && grounded_prev
        && vel_up <= 0.0
        && !stepped
        && world.character(walker.id).unwrap().ground_state() != GroundState::OnGround
    {
        let here = origin_of(world, walker, p);
        if let Some(found) = physics(|| snap(world, &filter, here, up, p)) {
            set_origin(world, walker, sub(here, scale(up, found.distance)), up, p);
            physics(|| world.refresh_character_contacts(walker.id, &filter)).unwrap();
            snapped = true;
            q5_distance = Some(found.distance);
        }
    }

    // Rules 6 and 7: rising is airborne; steep terrain is a wall, a steep structure holds.
    let character = world.character(walker.id).unwrap();
    let ground = character.ground_state();
    let ground_body = character.ground_body();
    let ground_layer = ground_body.and_then(|body| {
        contacts
            .iter()
            .find(|contact| contact.body == Some(body))
            .and_then(|contact| character.contact_object_layer(contact))
    });
    let on_terrain = ground_layer == Some(scene.layers.terrain);
    let mut q4_normal = None;
    let sliding = if cfg.passes.q4 {
        let here = origin_of(world, walker, p);
        q4_normal = physics(|| terrain_support_normal(world, scene.layers.terrain, here, up, p));
        q4_normal.is_some_and(|normal| dot(normal, up) < std::f64::consts::FRAC_1_SQRT_2)
    } else {
        ground == GroundState::OnSteepGround && on_terrain
    };
    let grounded = vel_up <= 0.0
        && (stepped
            || snapped
            || (!sliding
                && (ground == GroundState::OnGround
                    || (ground == GroundState::OnSteepGround && !on_terrain))));

    // Rule 8, the carry.
    if grounded || (ceiling && vel_up > 0.0) {
        vel_up = 0.0;
    }

    // Rule 9, the velocity.
    let end = origin_of(world, walker, p);
    let velocity = if walking {
        scale(sub(end, old), 1.0 / f64::from(DT))
    } else {
        [0.0; 3]
    };
    carry.vel_up = vel_up;
    carry.grounded = grounded;
    if let Some(trace) = trace {
        *trace = TraceTick {
            contacts: traced,
            q4_normal,
            q5_distance,
            q6_push,
            q6_depth,
        };
    }
    TickReport {
        start,
        up,
        maintenance: sub(old, start),
        moving: sub(after_move, old),
        post: sub(end, after_move),
        end,
        velocity,
        supplied,
        jolt_velocity,
        vel_up,
        grounded,
        sliding,
        ceiling,
        blocker,
        ground_before,
        ground_after_move,
        ground,
        ground_body,
        on_terrain,
        recovered,
        pushed,
        deep,
        still_ran: still,
        stepped,
        snapped,
        max_hits_exceeded,
        touched_groups,
    }
}

/// Moves `walker` to body origin `origin`.
fn set_origin(world: &mut PhysicsWorld, walker: &Walker, origin: V3, up: V3, p: f32) {
    world
        .character_mut(walker.id)
        .unwrap()
        .set_position(position_for(origin, up, p))
        .unwrap();
}

/// The group the fixtures keep in `layer`, for bodies that are not compounds.
fn layer_group(layers: crate::common::walker::Layers, layer: ObjectLayer) -> Option<u32> {
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
