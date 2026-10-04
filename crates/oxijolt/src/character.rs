//! Virtual characters: Jolt's `CharacterVirtual`, owned by a [`PhysicsWorld`].
//!
//! A character is not a body. It is a shape that the world moves with collision queries in
//! [`PhysicsWorld::update_character`], sliding along what it hits, stepping up stairs and
//! sticking to the floor (Jolt `CharacterVirtual::ExtendedUpdate`). It can optionally carry a
//! kinematic inner body so that bodies and queries see it.
//!
//! Reading a character ([`PhysicsWorld::character`]) takes `&PhysicsWorld`; creating, updating,
//! moving, restoring and removing one take `&mut PhysicsWorld`. Every contact normal a
//! character reports points toward the character: a floor gives a normal along its up, a
//! ceiling one against it.

use std::fmt;
use std::marker::PhantomData;
use std::ptr::{null, NonNull};

use oxijolt_sys::*;

use crate::body::{has_finite_inverse, mass_properties, MotionType};
use crate::filter::with_query_filters;
use crate::limits;
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{
    BodyId, CharacterError, CompoundSubShape, ObjectLayer, PhysicsWorld, Quat, QueryFilter, RVec3,
    Shape, SubShapeId, Vec3,
};

/// What the shape of a character's inner body must satisfy ([`has_finite_inverse`]).
pub(crate) const INNER_BODY_INERTIA_RULE: &str = "inner body shape must give a finite inverse mass and inertia, and an inertia that is not diagonal must meet the rigid body inertia floor of limits";

/// Jolt's invalid `BodyID` and `CharacterID` value.
const INVALID_ID: u32 = 0xffff_ffff;

/// Identifies a character in the world that created it.
///
/// The raw value is the Jolt `CharacterID` the world gave the character: 1 for the first
/// character of a world, then 2, 3 and so on. Ids are never reused within a world, so the same
/// creation history gives the same ids. Jolt orders contacts between characters by these ids.
///
/// A `CharacterId` also remembers its world: using it with another world returns
/// [`CharacterError::WrongWorld`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CharacterId {
    // Declared first, so ids order by Jolt's id before the world.
    raw: u32,
    pub(crate) world: WorldTag,
}

impl CharacterId {
    pub(crate) fn new(raw: u32, world: WorldTag) -> Self {
        Self { raw, world }
    }

    /// The id as Jolt stores it (`CharacterID::GetValue`).
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl fmt::Debug for CharacterId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("CharacterId").field(&self.raw).finish()
    }
}

/// How a character stands, from its last update (Jolt `CharacterBase::EGroundState`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GroundState {
    /// On ground that is not steeper than the maximum slope angle.
    OnGround,
    /// On ground steeper than the maximum slope angle; the character slides down.
    OnSteepGround,
    /// Touching something below that does not hold it up, for example the edge of a step it is
    /// not standing on; the character falls.
    NotSupported,
    /// Touching nothing below.
    InAir,
}

impl GroundState {
    fn from_jph(value: JPH_GroundState) -> Self {
        [
            (JPH_GroundState_OnGround, Self::OnGround),
            (JPH_GroundState_OnSteepGround, Self::OnSteepGround),
            (JPH_GroundState_NotSupported, Self::NotSupported),
            (JPH_GroundState_InAir, Self::InAir),
        ]
        .into_iter()
        .find_map(|(raw, state)| (raw == value).then_some(state))
        // Jolt has exactly these four, asserted equal to joltc's in
        // `native/layout_checks.cpp`. This runs in Rust after the call returned, so a panic
        // here never unwinds into C++.
        .unwrap_or_else(|| unreachable!("Jolt returned unknown ground state {value}"))
    }

    /// Whether the ground holds the character: [`OnGround`](Self::OnGround) or
    /// [`OnSteepGround`](Self::OnSteepGround) (Jolt `CharacterBase::IsSupported`).
    pub fn is_supported(self) -> bool {
        matches!(self, Self::OnGround | Self::OnSteepGround)
    }
}

/// The optional kinematic body a character carries, so that bodies collide with it and queries
/// find it (Jolt `CharacterVirtualSettings::mInnerBodyShape`).
#[derive(Clone, Copy)]
pub struct InnerBody<'a> {
    /// The body's shape, which must suit a kinematic body: no heightfield, a mass and inertia
    /// Jolt can invert, and an inertia that meets the rigid body inertia floor of [`limits`]
    /// when it is not diagonal.
    pub shape: &'a Shape,
    /// The body's object layer, which must exist in the world.
    pub object_layer: ObjectLayer,
}

impl fmt::Debug for InnerBody<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InnerBody")
            .field("object_layer", &self.object_layer)
            .finish_non_exhaustive()
    }
}

/// How to create a character, for [`PhysicsWorld::create_character`]. Build it with
/// [`new`](Self::new) and the setters.
///
/// The defaults are Jolt's `CharacterVirtualSettings` defaults. Lengths are in metres, angles in
/// radians, masses in kg and forces in newtons. The world gives every character its own id; it
/// is not a setting.
#[derive(Clone)]
pub struct CharacterSettings<'a> {
    shape: &'a Shape,
    up: Vec3,
    supporting_volume_normal: Vec3,
    supporting_volume_constant: f32,
    max_slope_angle: f32,
    enhanced_internal_edge_removal: bool,
    mass: f32,
    max_strength: f32,
    shape_offset: Vec3,
    back_face_collision: bool,
    predictive_contact_distance: f32,
    max_collision_iterations: u32,
    max_constraint_iterations: u32,
    min_time_remaining: f32,
    collision_tolerance: f32,
    character_padding: f32,
    max_num_hits: u32,
    hit_reduction_cos_max_angle: f32,
    penetration_recovery_speed: f32,
    inner_body: Option<InnerBody<'a>>,
    collide_with_characters: bool,
    user_data: u64,
}

impl fmt::Debug for CharacterSettings<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CharacterSettings")
            .field("up", &self.up)
            .field("max_slope_angle", &self.max_slope_angle)
            .field("shape_offset", &self.shape_offset)
            .field("character_padding", &self.character_padding)
            .field("inner_body", &self.inner_body)
            .field("collide_with_characters", &self.collide_with_characters)
            .finish_non_exhaustive()
    }
}

/// Jolt's `DegreesToRadians(50.0f)`, the default maximum slope angle.
const DEFAULT_MAX_SLOPE_ANGLE: f32 = 50.0 * (std::f32::consts::PI / 180.0);

impl<'a> CharacterSettings<'a> {
    /// A character of `shape`, with Jolt's defaults for everything else.
    ///
    /// The shape must be convex (a capsule, sphere, box or cylinder):
    /// [`PhysicsWorld::create_character`] rejects other shapes, because the character moves by
    /// casting its shape, and Jolt's casts of compound or heightfield shapes find nothing.
    pub fn new(shape: &'a Shape) -> Self {
        Self {
            shape,
            up: Vec3::new(0.0, 1.0, 0.0),
            supporting_volume_normal: Vec3::new(0.0, 1.0, 0.0),
            supporting_volume_constant: -1.0e10,
            max_slope_angle: DEFAULT_MAX_SLOPE_ANGLE,
            enhanced_internal_edge_removal: false,
            mass: 70.0,
            max_strength: 100.0,
            shape_offset: Vec3::ZERO,
            back_face_collision: true,
            predictive_contact_distance: 0.1,
            max_collision_iterations: 5,
            max_constraint_iterations: 15,
            min_time_remaining: 1.0e-4,
            collision_tolerance: 1.0e-3,
            character_padding: 0.02,
            max_num_hits: 256,
            hit_reduction_cos_max_angle: 0.999,
            penetration_recovery_speed: 1.0,
            inner_body: None,
            collide_with_characters: false,
            user_data: 0,
        }
    }

    /// The initial up direction, a unit vector. Default `+Y`. Change it per update with
    /// [`CharacterMut::set_up`].
    #[must_use]
    pub fn up(mut self, value: Vec3) -> Self {
        self.up = value;
        self
    }

    /// The plane, in the character's local space, that splits contacts that can support the
    /// character (below it) from the rest: points `p` with `normal · p + constant < 0` count.
    /// `normal` must be a unit vector. Default `+Y` and `-1e10`, so every contact counts.
    #[must_use]
    pub fn supporting_volume(mut self, normal: Vec3, constant: f32) -> Self {
        self.supporting_volume_normal = normal;
        self.supporting_volume_constant = constant;
        self
    }

    /// The steepest ground the character can stand on, radians in `[0, π/2]`. Default 50°.
    ///
    /// Jolt turns the slope limit off when its cosine is 0.9999 or above, that is for angles below
    /// about 0.81° (0.0141 rad): then no ground is too steep, so `0.0` means "no limit", not
    /// "flat ground only".
    #[must_use]
    pub fn max_slope_angle(mut self, radians: f32) -> Self {
        self.max_slope_angle = radians;
        self
    }

    /// Whether Jolt removes ghost contacts with internal edges of triangle shapes (heightfields,
    /// meshes). Costs extra CPU per contact. Default false.
    #[must_use]
    pub fn enhanced_internal_edge_removal(mut self, value: bool) -> Self {
        self.enhanced_internal_edge_removal = value;
        self
    }

    /// Mass in kg, between 0 and [`limits::MAX_MASS`], with which the character presses on what
    /// it stands on. Default 70. Each update also bounds the weight impulse it gives, see
    /// [`PhysicsWorld::update_character`].
    #[must_use]
    pub fn mass(mut self, value: f32) -> Self {
        self.mass = value;
        self
    }

    /// Largest force in newtons, at least 0, with which the character pushes bodies.
    /// Default 100.
    #[must_use]
    pub fn max_strength(mut self, value: f32) -> Self {
        self.max_strength = value;
        self
    }

    /// Offset of the shape from the character position, in the character's local space, every
    /// component at most [`limits::MAX_SHAPE_EXTENT`] in absolute value. Default zero.
    #[must_use]
    pub fn shape_offset(mut self, value: Vec3) -> Self {
        self.shape_offset = value;
        self
    }

    /// Whether the character collides with back-facing triangles, so it cannot pass through
    /// triangles from behind. Default true.
    #[must_use]
    pub fn back_face_collision(mut self, value: bool) -> Self {
        self.back_face_collision = value;
        self
    }

    /// How far beyond the shape to look for contacts, metres, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 0.1.
    #[must_use]
    pub fn predictive_contact_distance(mut self, value: f32) -> Self {
        self.predictive_contact_distance = value;
        self
    }

    /// Most collision passes per update, at least 1. Default 5.
    #[must_use]
    pub fn max_collision_iterations(mut self, value: u32) -> Self {
        self.max_collision_iterations = value;
        self
    }

    /// Most constraint-solving passes per collision pass, at least 1. Default 15.
    #[must_use]
    pub fn max_constraint_iterations(mut self, value: u32) -> Self {
        self.max_constraint_iterations = value;
        self
    }

    /// Time in seconds, positive, below which an update stops moving. Default 1e-4.
    #[must_use]
    pub fn min_time_remaining(mut self, value: f32) -> Self {
        self.min_time_remaining = value;
        self
    }

    /// How far the character may penetrate geometry, metres, positive and at most
    /// [`limits::MAX_SHAPE_EXTENT`]. Default 1e-3.
    #[must_use]
    pub fn collision_tolerance(mut self, value: f32) -> Self {
        self.collision_tolerance = value;
        self
    }

    /// How far the character keeps away from geometry, metres, between 0 and
    /// [`limits::MAX_SHAPE_EXTENT`]. The shape sits this far above the character position along
    /// up. Default 0.02.
    #[must_use]
    pub fn character_padding(mut self, value: f32) -> Self {
        self.character_padding = value;
        self
    }

    /// Most contacts collected per query, at least 1. Default 256. Jolt sorts contacts
    /// deterministically only while fewer than this many are found; see
    /// [`CharacterRef::max_hits_exceeded`].
    #[must_use]
    pub fn max_num_hits(mut self, value: u32) -> Self {
        self.max_num_hits = value;
        self
    }

    /// Cosine of the largest angle between two contact normals that Jolt merges into one
    /// contact; -1 turns merging off. Default 0.999 (about 2.5°).
    #[must_use]
    pub fn hit_reduction_cos_max_angle(mut self, value: f32) -> Self {
        self.hit_reduction_cos_max_angle = value;
        self
    }

    /// Fraction in `[0, 1]` of a penetration resolved per update: 0 resolves nothing, 1 all of
    /// it at once. Default 1.
    #[must_use]
    pub fn penetration_recovery_speed(mut self, value: f32) -> Self {
        self.penetration_recovery_speed = value;
        self
    }

    /// The kinematic inner body, or none. Default none.
    ///
    /// Jolt creates the body with the character, moves it with the character and removes it
    /// with the character; it never collides with its own character. While the character
    /// exists, [`PhysicsWorld::remove_body`] refuses to remove it.
    #[must_use]
    pub fn inner_body(mut self, value: Option<InnerBody<'a>>) -> Self {
        self.inner_body = value;
        self
    }

    /// Whether the character collides with the other characters of its world that have this
    /// set. Default false.
    #[must_use]
    pub fn collide_with_characters(mut self, value: bool) -> Self {
        self.collide_with_characters = value;
        self
    }

    /// The caller's value for this character. Jolt also gives it to the inner body as its body
    /// user data. Default 0.
    #[must_use]
    pub fn user_data(mut self, value: u64) -> Self {
        self.user_data = value;
        self
    }

    pub(crate) fn validate(&self, object_layer_count: u32) -> Result<(), CharacterError> {
        let invalid = |what| Err(CharacterError::InvalidValue(what));
        if !is_unit(self.up) {
            return invalid("up must be a finite unit vector");
        }
        if !is_unit(self.supporting_volume_normal) {
            return invalid("supporting volume normal must be a finite unit vector");
        }
        if !self.supporting_volume_constant.is_finite() {
            return invalid("supporting volume constant must be finite");
        }
        if !(is_finite_non_negative(self.max_slope_angle)
            && self.max_slope_angle <= std::f32::consts::FRAC_PI_2)
        {
            return invalid("max slope angle must be between 0 and pi/2");
        }
        if !(0.0..=limits::MAX_MASS).contains(&self.mass) {
            return invalid("mass must be between 0 and limits::MAX_MASS");
        }
        if !is_finite_non_negative(self.max_strength) {
            return invalid("max strength must be finite and not negative");
        }
        if !limits::is_local_offset(self.shape_offset) {
            return invalid("shape offset must be finite and within limits::MAX_SHAPE_EXTENT");
        }
        if !limits::is_local_distance(self.predictive_contact_distance) {
            return invalid(
                "predictive contact distance must be between 0 and limits::MAX_SHAPE_EXTENT",
            );
        }
        if self.max_collision_iterations == 0 || self.max_constraint_iterations == 0 {
            return invalid("iteration counts must be at least 1");
        }
        if !is_finite_positive(self.min_time_remaining) {
            return invalid("min time remaining must be finite and positive");
        }
        if !(limits::is_local_distance(self.collision_tolerance) && self.collision_tolerance > 0.0)
        {
            return invalid(
                "collision tolerance must be positive and at most limits::MAX_SHAPE_EXTENT",
            );
        }
        if !limits::is_local_distance(self.character_padding) {
            return invalid("character padding must be between 0 and limits::MAX_SHAPE_EXTENT");
        }
        if self.max_num_hits == 0 {
            return invalid("max num hits must be at least 1");
        }
        if !self.hit_reduction_cos_max_angle.is_finite() {
            return invalid("hit reduction cos max angle must be finite");
        }
        if !(is_finite_non_negative(self.penetration_recovery_speed)
            && self.penetration_recovery_speed <= 1.0)
        {
            return invalid("penetration recovery speed must be between 0 and 1");
        }
        // SAFETY: the shape is live for the call; the getter only reads it.
        if unsafe { JPH_Shape_GetType(self.shape.as_ptr()) } != JPH_ShapeType_Convex {
            return invalid("the character shape must be convex");
        }
        if let Some(inner) = &self.inner_body {
            if inner.object_layer.get() >= object_layer_count {
                return invalid("inner body object layer does not exist in this world");
            }
            if inner.shape.must_be_static() {
                return invalid("inner body shape can only be used by static bodies");
            }
            // Jolt creates the inner body kinematic, which computes mass properties.
            if !has_finite_inverse(&mass_properties(inner.shape, None)) {
                return invalid(INNER_BODY_INERTIA_RULE);
            }
        }
        Ok(())
    }

    /// The joltc settings for a character with Jolt id `id`. The pointers borrow `self`.
    fn to_jph(&self, id: u32) -> JPH_CharacterVirtualSettings {
        JPH_CharacterVirtualSettings {
            base: JPH_CharacterBaseSettings {
                up: self.up.to_jph(),
                supportingVolume: JPH_Plane {
                    normal: self.supporting_volume_normal.to_jph(),
                    distance: self.supporting_volume_constant,
                },
                maxSlopeAngle: self.max_slope_angle,
                enhancedInternalEdgeRemoval: self.enhanced_internal_edge_removal,
                shape: self.shape.as_ptr(),
            },
            ID: id,
            mass: self.mass,
            maxStrength: self.max_strength,
            shapeOffset: self.shape_offset.to_jph(),
            backFaceMode: if self.back_face_collision {
                JPH_BackFaceMode_CollideWithBackFaces
            } else {
                JPH_BackFaceMode_IgnoreBackFaces
            },
            predictiveContactDistance: self.predictive_contact_distance,
            maxCollisionIterations: self.max_collision_iterations,
            maxConstraintIterations: self.max_constraint_iterations,
            minTimeRemaining: self.min_time_remaining,
            collisionTolerance: self.collision_tolerance,
            characterPadding: self.character_padding,
            maxNumHits: self.max_num_hits,
            hitReductionCosMaxAngle: self.hit_reduction_cos_max_angle,
            penetrationRecoverySpeed: self.penetration_recovery_speed,
            innerBodyShape: self.inner_body.map_or(null(), |inner| inner.shape.as_ptr()),
            innerBodyIDOverride: INVALID_ID,
            innerBodyLayer: self.inner_body.map_or(0, |inner| inner.object_layer.get()),
        }
    }
}

/// What a caller-given character position must satisfy.
const POSITION_RULE: &str = "position must be finite and within limits::MAX_POSITION";

/// What a caller-given character velocity must satisfy.
const LINEAR_VELOCITY_RULE: &str =
    "linear velocity must be finite and at most limits::MAX_LINEAR_VELOCITY long";

/// Whether `v` is finite and of unit length within Jolt's `Vec3::IsNormalized` tolerance
/// (`|length² − 1| <= 1e-5`).
fn is_unit(v: Vec3) -> bool {
    v.is_finite() && (v.dot(v) - 1.0).abs() <= 1.0e-5
}

/// What an update does besides moving the character, for [`PhysicsWorld::update_character`]
/// (Jolt `CharacterVirtual::ExtendedUpdateSettings`). The defaults are Jolt's.
///
/// The vectors are in world space and assume `+Y` up by default; with another up the caller
/// supplies them along its up. A zero stick-to-floor or step-up vector turns that feature off
/// (Jolt's `IsNearZero`). Lengths are in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtendedUpdateSettings {
    stick_to_floor_step_down: Vec3,
    walk_stairs_step_up: Vec3,
    walk_stairs_min_step_forward: f32,
    walk_stairs_step_forward_test: f32,
    walk_stairs_cos_angle_forward_contact: f32,
    walk_stairs_step_down_extra: Vec3,
}

impl Default for ExtendedUpdateSettings {
    fn default() -> Self {
        Self {
            stick_to_floor_step_down: Vec3::new(0.0, -0.5, 0.0),
            walk_stairs_step_up: Vec3::new(0.0, 0.4, 0.0),
            walk_stairs_min_step_forward: 0.02,
            walk_stairs_step_forward_test: 0.15,
            walk_stairs_cos_angle_forward_contact: (75.0 * (std::f32::consts::PI / 180.0_f32))
                .cos(),
            walk_stairs_step_down_extra: Vec3::ZERO,
        }
    }
}

impl ExtendedUpdateSettings {
    /// How far down the character looks for floor to stick to when it was supported before the
    /// update and is not after it, without moving up. Every component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value. Default `(0, -0.5, 0)`.
    #[must_use]
    pub fn stick_to_floor_step_down(mut self, value: Vec3) -> Self {
        self.stick_to_floor_step_down = value;
        self
    }

    /// How high the character steps up when a step blocks it. Every component at most
    /// [`limits::MAX_SHAPE_EXTENT`] in absolute value. Default `(0, 0.4, 0)`.
    ///
    /// The highest step climbed is not this length: a capsule of radius `r` climbs about
    /// step-up + padding + `r (1 - cos max_slope_angle)`, and the step's own rounding changes it
    /// too, so measure it. Jolt judges a step by the surface normal at the contact; on a box with
    /// sharp edges (convex radius 0) the contact sits on the top edge, where float rounding
    /// decides between the top face's normal and the side's, so walk stairs climbs such a step
    /// unreliably.
    #[must_use]
    pub fn walk_stairs_step_up(mut self, value: Vec3) -> Self {
        self.walk_stairs_step_up = value;
        self
    }

    /// Least distance, between 0 and [`limits::MAX_SHAPE_EXTENT`], the character moves forward
    /// after stepping up. Default 0.02.
    #[must_use]
    pub fn walk_stairs_min_step_forward(mut self, value: f32) -> Self {
        self.walk_stairs_min_step_forward = value;
        self
    }

    /// How far ahead, between 0 and [`limits::MAX_SHAPE_EXTENT`], the character tests for floor
    /// after stepping up, so it does not step onto a slope it would slide off. Default 0.15.
    #[must_use]
    pub fn walk_stairs_step_forward_test(mut self, value: f32) -> Self {
        self.walk_stairs_step_forward_test = value;
        self
    }

    /// Cosine of the largest angle between the movement and a step's normal at which the
    /// character steps toward the step's normal instead. Default cos 75°.
    #[must_use]
    pub fn walk_stairs_cos_angle_forward_contact(mut self, value: f32) -> Self {
        self.walk_stairs_cos_angle_forward_contact = value;
        self
    }

    /// Extra distance the character moves down after stepping up, to stay on stairs that go
    /// down. Every component at most [`limits::MAX_SHAPE_EXTENT`] in absolute value. Default
    /// zero.
    #[must_use]
    pub fn walk_stairs_step_down_extra(mut self, value: Vec3) -> Self {
        self.walk_stairs_step_down_extra = value;
        self
    }

    fn validate(&self) -> Result<(), CharacterError> {
        let offsets = limits::is_local_offset(self.stick_to_floor_step_down)
            && limits::is_local_offset(self.walk_stairs_step_up)
            && limits::is_local_offset(self.walk_stairs_step_down_extra);
        if !offsets {
            return Err(CharacterError::InvalidValue(
                "extended update steps must be finite and within limits::MAX_SHAPE_EXTENT",
            ));
        }
        if !self.walk_stairs_cos_angle_forward_contact.is_finite() {
            return Err(CharacterError::InvalidValue(
                "walk stairs cos angle forward contact must be finite",
            ));
        }
        if !(limits::is_local_distance(self.walk_stairs_min_step_forward)
            && limits::is_local_distance(self.walk_stairs_step_forward_test))
        {
            return Err(CharacterError::InvalidValue(
                "walk stairs forward distances must be between 0 and limits::MAX_SHAPE_EXTENT",
            ));
        }
        Ok(())
    }

    fn to_jph(self) -> JPH_ExtendedUpdateSettings {
        JPH_ExtendedUpdateSettings {
            stickToFloorStepDown: self.stick_to_floor_step_down.to_jph(),
            walkStairsStepUp: self.walk_stairs_step_up.to_jph(),
            walkStairsMinStepForward: self.walk_stairs_min_step_forward,
            walkStairsStepForwardTest: self.walk_stairs_step_forward_test,
            walkStairsCosAngleForwardContact: self.walk_stairs_cos_angle_forward_contact,
            walkStairsStepDownExtra: self.walk_stairs_step_down_extra.to_jph(),
        }
    }
}

/// A contact of a character after its last update or contact refresh (Jolt
/// `CharacterVirtual::Contact`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CharacterContact {
    /// The body touched, or `None` for another character.
    pub body: Option<BodyId>,
    /// The character touched, or `None` for a body.
    pub character: Option<CharacterId>,
    /// The leaf of the touched body's shape (a compound child, a heightfield triangle); see
    /// [`CharacterRef::contact_compound_child`].
    pub sub_shape_id: SubShapeId,
    /// The contact point in world space, metres.
    pub position: RVec3,
    /// The contact normal, pointing toward the character: along up for a floor, against up for
    /// a ceiling.
    pub contact_normal: Vec3,
    /// The surface normal of what was touched, pointing toward the character. Equal to the
    /// contact normal when that points further up (Jolt replaces it then).
    pub surface_normal: Vec3,
    /// Distance to the contact, metres: at most 0 is touching, positive is a predictive
    /// contact.
    pub distance: f32,
    /// Fraction of the update's movement after which the contact was hit.
    pub fraction: f32,
    /// Velocity of the contact point, m/s.
    pub linear_velocity: Vec3,
    /// How the touched body moves; a character counts as kinematic.
    pub motion_type: MotionType,
    /// Whether the touched body is a sensor.
    pub is_sensor: bool,
    /// Whether the character collided with it in the last update, not just came near.
    pub had_collision: bool,
    /// Whether the character ignored it (Jolt discards contacts it slides past).
    pub was_discarded: bool,
    /// Whether the character touched a back face of a triangle.
    pub is_back_facing: bool,
}

impl CharacterContact {
    fn from_jph(contact: &JPH_CharacterContact, world: WorldTag) -> Self {
        Self {
            body: (contact.bodyB != INVALID_ID).then(|| BodyId::new(contact.bodyB, world)),
            character: (contact.characterIDB != INVALID_ID)
                .then(|| CharacterId::new(contact.characterIDB, world)),
            sub_shape_id: SubShapeId::new(contact.subShapeIDB),
            position: RVec3::from_jph(contact.position),
            contact_normal: Vec3::from_jph(contact.contactNormal),
            surface_normal: Vec3::from_jph(contact.surfaceNormal),
            distance: contact.distance,
            fraction: contact.fraction,
            linear_velocity: Vec3::from_jph(contact.linearVelocity),
            motion_type: MotionType::from_jph(contact.motionTypeB),
            is_sensor: contact.isSensorB,
            had_collision: contact.hadCollision,
            was_discarded: contact.wasDiscarded,
            is_back_facing: contact.isBackFacingContact,
        }
    }
}

/// The persistent state of a character, from [`CharacterRef::save_state`], to continue a run
/// bit for bit with [`CharacterMut::restore_state`].
///
/// It holds Jolt's `CharacterVirtual::SaveState` stream (pose, velocity, ground data and the
/// contacts the character collided with) and the character's up, which Jolt does not save.
/// It does not hold the settings, the shape or the contacts without collision, and Jolt does not
/// restore a contact's character pointer, material or user data, nor the ground's material or
/// user data. None of these feed the next update: before moving, Jolt reads only the normals and
/// velocities of contacts with collision, and the move rebuilds the contacts.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct CharacterState {
    jolt: Vec<u8>,
    up: [u32; 3],
}

impl CharacterState {
    /// The state as bytes, for digests: Jolt's stream, then the bits of up's three components
    /// in little-endian order. The bytes are specific to this build; there is no way back.
    pub fn as_bytes(&self) -> Vec<u8> {
        let mut bytes = self.jolt.clone();
        for bits in self.up {
            bytes.extend_from_slice(&bits.to_le_bytes());
        }
        bytes
    }
}

/// A Jolt state recorder (`StateRecorderImpl`), owned whole by its owner.
impl JoltObject for JPH_StateRecorder {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the recorder (trait contract), which the extension deletes.
        unsafe { JPH_StateRecorder_Destroy(ptr) };
    }
}

/// A character, of which the world holds the one reference `JPH_CharacterVirtual_Create`
/// returns. Releasing it deletes the character, whose destructor removes and destroys its inner
/// body through the physics system (`CharacterVirtual.cpp`), so the system must outlive it.
impl JoltObject for JPH_CharacterVirtual {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds the one reference (trait contract); the world drops its
        // characters before its physics system (field order in `PhysicsWorld`).
        unsafe { JPH_CharacterBase_Destroy(ptr.cast()) };
    }
}

/// Jolt's `CharacterVsCharacterCollisionSimple`, owned whole by its world. Destroying it only
/// frees its list of character pointers.
impl JoltObject for JPH_CharacterVsCharacterCollision {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the object (trait contract); joltc deletes it through Jolt's
        // virtual destructor.
        unsafe { JPH_CharacterVsCharacterCollision_Destroy(ptr) };
    }
}

/// One character of a world.
pub(crate) struct CharacterEntry {
    character: Owned<JPH_CharacterVirtual>,
    inner_body: Option<BodyId>,
    collides_with_characters: bool,
    /// The mass with which the character presses on what it stands on.
    mass: f32,
}

/// Read access to one character, borrowed from its world.
///
/// Each method reads the character's state as its last update, refresh or setter left it.
pub struct CharacterRef<'w> {
    world: &'w PhysicsWorld,
    id: CharacterId,
    character: NonNull<JPH_CharacterVirtual>,
}

impl CharacterRef<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    fn base(&self) -> *mut JPH_CharacterBase {
        self.character.as_ptr().cast()
    }

    /// The character's id.
    pub fn id(&self) -> CharacterId {
        self.id
    }

    /// Position in world space, metres. The shape sits at this position plus the rotated shape
    /// offset plus the padding along up.
    pub fn position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the character, and changing it needs
        // `&mut PhysicsWorld`; the getter reads a member. `value` is a live local.
        unsafe { JPH_CharacterVirtual_GetPosition(self.ptr(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Rotation.
    pub fn rotation(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetRotation(self.ptr(), &mut value) };
        Quat::from_jph(value)
    }

    /// The up direction, a unit vector.
    pub fn up(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetUp(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Linear velocity, m/s: what the caller set, as the last update changed it.
    pub fn linear_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetLinearVelocity(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// How the character stands.
    pub fn ground_state(&self) -> GroundState {
        // SAFETY: as in `position`.
        GroundState::from_jph(unsafe { JPH_CharacterBase_GetGroundState(self.base()) })
    }

    /// Normal of the ground contact, pointing toward the character; zero when there is none.
    pub fn ground_normal(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundNormal(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The ground contact point in world space, metres.
    pub fn ground_position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundPosition(self.base(), &mut value) };
        RVec3::from_jph(value)
    }

    /// Velocity of the ground under the character, m/s.
    pub fn ground_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterBase_GetGroundVelocity(self.base(), &mut value) };
        Vec3::from_jph(value)
    }

    /// The body the character stands on, or `None` (in the air, or on another character).
    pub fn ground_body(&self) -> Option<BodyId> {
        // SAFETY: as in `position`.
        let raw = unsafe { JPH_CharacterBase_GetGroundBodyId(self.base()) };
        (raw != INVALID_ID).then(|| BodyId::new(raw, self.world.tag))
    }

    /// The leaf of the ground body's shape the character stands on.
    pub fn ground_sub_shape_id(&self) -> SubShapeId {
        // SAFETY: as in `position`.
        SubShapeId::new(unsafe { JPH_CharacterBase_GetGroundSubShapeId(self.base()) })
    }

    /// The compound child the character stands on, with its user data (a collision group);
    /// `None` when the ground is not a compound child or no longer in the world.
    pub fn ground_compound_child(&self) -> Option<CompoundSubShape> {
        let body = self.ground_body()?;
        self.world
            .compound_sub_shape(body, self.ground_sub_shape_id())
            .ok()
            .flatten()
    }

    /// The contacts after the last update or refresh, in Jolt's order (sorted, which is
    /// deterministic while [`max_hits_exceeded`](Self::max_hits_exceeded) is false).
    pub fn active_contacts(&self) -> Vec<CharacterContact> {
        // SAFETY: as in `position`.
        let count = unsafe { JPH_CharacterVirtual_GetNumActiveContacts(self.ptr()) };
        (0..count)
            .map(|index| {
                // SAFETY: an all-zero `JPH_CharacterContact` is valid: integers, floats,
                // `false`, null pointers and `JPH_MotionType_Static` (0).
                let mut contact: JPH_CharacterContact = unsafe { std::mem::zeroed() };
                // SAFETY: as in `position`; `index < count`, so joltc's `at` stays in range, and
                // `contact` is a live local that joltc overwrites.
                unsafe { JPH_CharacterVirtual_GetActiveContact(self.ptr(), index, &mut contact) };
                CharacterContact::from_jph(&contact, self.world.tag)
            })
            .collect()
    }

    /// The compound child `contact` touched, with its user data (a collision group); `None` for
    /// characters, bodies that are not compounds and bodies no longer in the world.
    pub fn contact_compound_child(&self, contact: &CharacterContact) -> Option<CompoundSubShape> {
        self.world
            .compound_sub_shape(contact.body?, contact.sub_shape_id)
            .ok()
            .flatten()
    }

    /// The object layer of the body `contact` touched; `None` for characters and bodies no
    /// longer in the world.
    pub fn contact_object_layer(&self, contact: &CharacterContact) -> Option<ObjectLayer> {
        let body = contact.body?;
        self.world
            .contains(body)
            .then(|| self.world.object_layer_of(body))
    }

    /// The character's inner body, if it has one.
    pub fn inner_body(&self) -> Option<BodyId> {
        self.world.characters[&self.id.raw].inner_body
    }

    /// Whether the last update found more contacts than
    /// [`CharacterSettings::max_num_hits`]. Jolt then drops contacts in an order that is not
    /// guaranteed to be deterministic.
    pub fn max_hits_exceeded(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_CharacterVirtual_GetMaxHitsExceeded(self.ptr()) }
    }

    /// The character's persistent state, to restore later with
    /// [`CharacterMut::restore_state`].
    pub fn save_state(&self) -> CharacterState {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        // SAFETY: as in `position`; `SaveState` is const in Jolt, and the recorder is live and
        // used by this thread only.
        let size = unsafe {
            JPH_CharacterVirtual_SaveState(self.ptr(), recorder.as_ptr());
            JPH_StateRecorder_GetDataSize(recorder.as_ptr())
        };
        let mut jolt = vec![0_u8; size];
        // SAFETY: `jolt` holds exactly `size` writable bytes, which joltc copies at most.
        unsafe { JPH_StateRecorder_CopyData(recorder.as_ptr(), jolt.as_mut_ptr().cast(), size) };
        let up: [f32; 3] = self.up().into();
        CharacterState {
            jolt,
            up: up.map(f32::to_bits),
        }
    }
}

/// Write access to one character, borrowed mutably from its world.
///
/// Read the character through [`PhysicsWorld::character`] once this borrow ends.
pub struct CharacterMut<'w> {
    character: NonNull<JPH_CharacterVirtual>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl CharacterMut<'_> {
    fn ptr(&self) -> *mut JPH_CharacterVirtual {
        self.character.as_ptr()
    }

    /// Moves the character to `position` (metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value), without collision checks. Also moves the
    /// inner body.
    pub fn set_position(&mut self, position: RVec3) -> Result<(), CharacterError> {
        if !limits::is_in_frame(position) {
            return Err(CharacterError::InvalidValue(POSITION_RULE));
        }
        self.write_position(position);
        Ok(())
    }

    /// Writes a finite position without the frame bound of [`set_position`](Self::set_position),
    /// for state the world re-expresses ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_position(&mut self, position: RVec3) {
        debug_assert!(position.is_finite());
        let position = position.to_jph();
        // SAFETY: the world is borrowed mutably through this view and owns the character. Jolt
        // moves the inner body through the locking body interface; this thread holds no body
        // lock. `position` is a live local.
        unsafe { JPH_CharacterVirtual_SetPosition(self.ptr(), &position) };
    }

    /// Sets the rotation, a finite unit quaternion. Also rotates the inner body.
    pub fn set_rotation(&mut self, rotation: Quat) -> Result<(), CharacterError> {
        if !rotation.is_valid_rotation() {
            return Err(CharacterError::InvalidValue(
                "rotation must be a finite unit quaternion",
            ));
        }
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_CharacterVirtual_SetRotation(self.ptr(), &rotation) };
        Ok(())
    }

    /// Sets the up direction, a finite unit vector, for the next updates.
    pub fn set_up(&mut self, up: Vec3) -> Result<(), CharacterError> {
        if !is_unit(up) {
            return Err(CharacterError::InvalidValue(
                "up must be a finite unit vector",
            ));
        }
        let up = up.to_jph();
        // SAFETY: as in `set_position`; the setter writes a member.
        unsafe { JPH_CharacterBase_SetUp(self.ptr().cast(), &up) };
        Ok(())
    }

    /// Sets the linear velocity, m/s, that the next update moves the character with: finite and
    /// at most [`limits::MAX_LINEAR_VELOCITY`] long (Jolt's own length). Jolt does not clamp a
    /// character's velocity; it becomes the displacement the update casts.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), CharacterError> {
        if !limits::is_linear_velocity(velocity) {
            return Err(CharacterError::InvalidValue(LINEAR_VELOCITY_RULE));
        }
        self.write_linear_velocity(velocity);
        Ok(())
    }

    /// Writes a finite velocity without the bound of
    /// [`set_linear_velocity`](Self::set_linear_velocity), for state the world re-expresses
    /// ([`PhysicsWorld::rebase`]).
    pub(crate) fn write_linear_velocity(&mut self, velocity: Vec3) {
        debug_assert!(velocity.is_finite());
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_up`.
        unsafe { JPH_CharacterVirtual_SetLinearVelocity(self.ptr(), &velocity) };
    }

    /// Restores a state saved with [`CharacterRef::save_state`]: pose, velocity, up, ground
    /// data and the contacts with collision.
    ///
    /// Any state from `save_state` may be restored into any character, also of another world.
    /// The result is meaningful when the worlds match: the same bodies with the same ids and the
    /// same character id and settings, as in a replay that rebuilds the world the same way. Ids
    /// in the state that do not resolve are harmless: Jolt checks every body id it looks up, and
    /// the readers here tolerate any sub-shape id. The inner body is moved to the restored pose,
    /// as [`set_position`](Self::set_position) moves it.
    pub fn restore_state(&mut self, state: &CharacterState) -> Result<(), CharacterError> {
        // SAFETY: Jolt is initialised (the world exists). The handle takes over the recorder.
        let recorder = unsafe { Owned::from_raw(JPH_StateRecorder_Create()) }
            .unwrap_or_else(|| unreachable!("`new` does not return null"));
        let up = Vec3::from(state.up.map(f32::from_bits)).to_jph();
        // SAFETY: the recorder is live and used by this thread only; `state.jolt` is readable
        // for its length. The bytes are a complete stream written by `SaveState` of this build
        // (`CharacterState` has no other constructor), so `RestoreState` reads exactly what was
        // written. The world is borrowed mutably and owns the character; `up` and `position` are
        // live locals. Jolt's `RestoreState` writes the pose members without moving the inner
        // body; setting the restored position again moves it there through the locking body
        // interface (`CharacterVirtual::SetPosition`), and this thread holds no body lock.
        let failed = unsafe {
            JPH_StateRecorder_WriteBytes(
                recorder.as_ptr(),
                state.jolt.as_ptr().cast(),
                state.jolt.len(),
            );
            JPH_StateRecorder_Rewind(recorder.as_ptr());
            JPH_CharacterVirtual_RestoreState(self.ptr(), recorder.as_ptr());
            JPH_CharacterBase_SetUp(self.ptr().cast(), &up);
            let mut position = RVec3::ZERO.to_jph();
            JPH_CharacterVirtual_GetPosition(self.ptr(), &mut position);
            JPH_CharacterVirtual_SetPosition(self.ptr(), &position);
            JPH_StateRecorder_IsFailed(recorder.as_ptr())
        };
        debug_assert!(!failed, "a saved character state failed to restore");
        if failed {
            return Err(CharacterError::InvalidValue(
                "character state stream failed",
            ));
        }
        Ok(())
    }
}

impl PhysicsWorld {
    /// Creates a character at `position` (metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value) with `rotation` and returns its id.
    ///
    /// Fails with [`CharacterError::InvalidValue`] when a setting or the pose is out of range
    /// (see the setters of [`CharacterSettings`]), with [`CharacterError::TooManyBodies`] when an
    /// inner body was asked for and the world is full, and with
    /// [`CharacterError::TooManyCharacters`] when the world has run out of character ids.
    /// Nothing is created on failure.
    ///
    /// The new character knows no contacts and reports [`GroundState::InAir`] until its first
    /// update or [`refresh_character_contacts`](Self::refresh_character_contacts). Refresh a
    /// character that starts on the ground: stick to floor acts only when the character was
    /// supported before the update.
    pub fn create_character(
        &mut self,
        settings: &CharacterSettings<'_>,
        position: RVec3,
        rotation: Quat,
    ) -> Result<CharacterId, CharacterError> {
        settings.validate(self.object_layer_count)?;
        if !limits::is_in_frame(position) {
            return Err(CharacterError::InvalidValue(POSITION_RULE));
        }
        if !rotation.is_valid_rotation() {
            return Err(CharacterError::InvalidValue(
                "rotation must be a finite unit quaternion",
            ));
        }
        let raw = self.next_character_id;
        // Jolt's invalid `CharacterID`.
        if raw == INVALID_ID {
            return Err(CharacterError::TooManyCharacters);
        }
        if settings.inner_body.is_some() && !self.has_room_for_bodies(1) {
            return Err(CharacterError::TooManyBodies);
        }
        let collision = if settings.collide_with_characters {
            Some(self.character_collision()?)
        } else {
            None
        };
        // The world passes an explicit id: Jolt's default comes from a process-wide counter,
        // and Jolt orders contacts between characters by id.
        let jolt_settings = settings.to_jph(raw);
        let position = position.to_jph();
        let rotation = rotation.to_jph();
        self.note_structure_change();
        // SAFETY: the system is live and borrowed mutably; the settings, their shape pointers
        // (borrowed from `settings`) and the pose are live for the call, and validated. The
        // character takes its own references to the shapes. The handle takes over the one
        // reference joltc returns.
        let character = unsafe {
            Owned::from_raw(JPH_CharacterVirtual_Create(
                &jolt_settings,
                &position,
                &rotation,
                settings.user_data,
                self.system.as_ptr(),
            ))
        }
        .unwrap_or_else(|| unreachable!("joltc `new`s the character"));
        let inner_body = if settings.inner_body.is_some() {
            // SAFETY: the character is live; the getter reads a member.
            let raw_body = unsafe { JPH_CharacterVirtual_GetInnerBodyID(character.as_ptr()) };
            if raw_body == INVALID_ID {
                // Jolt created no body because the world is full, which the room check above
                // rules out; dropping the character releases it and creates nothing else.
                return Err(CharacterError::TooManyBodies);
            }
            Some(BodyId::new(raw_body, self.tag))
        } else {
            None
        };
        if let Some(collision) = collision {
            // SAFETY: both objects are live and owned by this world, borrowed mutably. The set
            // keeps a pointer to the character until `remove_character` takes it out; the
            // character keeps a pointer to the set, which the world drops after its characters.
            unsafe {
                JPH_CharacterVsCharacterCollisionSimple_AddCharacter(
                    collision.as_ptr(),
                    character.as_ptr(),
                );
                JPH_CharacterVirtual_SetCharacterVsCharacterCollision(
                    character.as_ptr(),
                    collision.as_ptr(),
                );
            }
        }
        if let Some(body) = inner_body {
            self.inner_bodies.insert(body.to_raw());
        }
        self.characters.insert(
            raw,
            CharacterEntry {
                character,
                inner_body,
                collides_with_characters: settings.collide_with_characters,
                mass: settings.mass,
            },
        );
        self.next_character_id += 1;
        Ok(CharacterId::new(raw, self.tag))
    }

    /// The world's character-versus-character set, created on first use.
    fn character_collision(
        &mut self,
    ) -> Result<NonNull<JPH_CharacterVsCharacterCollision>, CharacterError> {
        if self.character_collision.is_none() {
            // SAFETY: Jolt is initialised. The handle takes over the new object.
            let created =
                unsafe { Owned::from_raw(JPH_CharacterVsCharacterCollision_CreateSimple()) }
                    .ok_or(CharacterError::InvalidValue(
                        "could not create the character collision set",
                    ))?;
            self.character_collision = Some(created);
        }
        Ok(self
            .character_collision
            .as_ref()
            .map(Owned::as_non_null)
            .unwrap_or_else(|| unreachable!("set above")))
    }

    /// The entry of `id`, if it names a character of this world.
    fn character_entry(&self, id: CharacterId) -> Result<&CharacterEntry, CharacterError> {
        if id.world != self.tag {
            return Err(CharacterError::WrongWorld(id));
        }
        self.characters
            .get(&id.raw)
            .ok_or(CharacterError::NotFound(id))
    }

    /// Removes a character, and with it its inner body.
    ///
    /// Bodies around the removed inner body are not woken: a kinematic inner body only touches
    /// dynamic bodies, which stay awake while they touch it.
    pub fn remove_character(&mut self, id: CharacterId) -> Result<(), CharacterError> {
        self.character_entry(id)?;
        self.note_structure_change();
        let entry = self
            .characters
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        if entry.collides_with_characters {
            if let Some(collision) = &self.character_collision {
                // SAFETY: both are live and owned by this world, borrowed mutably; the character
                // is in the set (it was added at creation).
                unsafe {
                    JPH_CharacterVsCharacterCollisionSimple_RemoveCharacter(
                        collision.as_ptr(),
                        entry.character.as_ptr(),
                    )
                };
            }
        }
        if let Some(body) = entry.inner_body {
            self.inner_bodies.remove(&body.to_raw());
        }
        // Other characters may keep a pointer to this one in their cached contacts. That is
        // sound: without a contact listener Jolt never dereferences a cached contact's
        // `mCharacterB` (`ValidateContact` and `ContactAdded` return early), joltc's contact
        // readout copies the pointer without dereferencing it, and oxijolt installs no
        // listener and never reads that pointer.
        drop(entry);
        Ok(())
    }

    /// Read access to a character.
    pub fn character(&self, id: CharacterId) -> Result<CharacterRef<'_>, CharacterError> {
        let entry = self.character_entry(id)?;
        Ok(CharacterRef {
            world: self,
            id,
            character: entry.character.as_non_null(),
        })
    }

    /// Write access to a character.
    pub fn character_mut(&mut self, id: CharacterId) -> Result<CharacterMut<'_>, CharacterError> {
        let entry = self.character_entry(id)?;
        Ok(CharacterMut {
            character: entry.character.as_non_null(),
            _world: PhantomData,
        })
    }

    /// The ids of the world's characters, in id order (creation order).
    pub fn character_ids(&self) -> impl Iterator<Item = CharacterId> + '_ {
        self.characters
            .keys()
            .map(|&raw| CharacterId::new(raw, self.tag))
    }

    /// Whether `id` is the inner body of a character of this world.
    pub fn is_inner_body(&self, id: BodyId) -> bool {
        id.world == self.tag && self.inner_bodies.contains(&id.to_raw())
    }

    /// Moves a character by its linear velocity for `delta_time` seconds, colliding with what
    /// `filter` selects, then sticks it to the floor and walks it up stairs as `settings` says
    /// (Jolt `CharacterVirtual::ExtendedUpdate`).
    ///
    /// Set the velocity first with [`CharacterMut::set_linear_velocity`]. `gravity` (m/s²) is
    /// not added to the velocity; Jolt uses it to press on what the character stands on. The
    /// character's own inner body is never hit. Characters that collide with characters also
    /// hit each other, whatever the filter says.
    ///
    /// `delta_time` must be finite, at least [`MIN_DELTA_TIME`](Self::MIN_DELTA_TIME) and at
    /// most [`MAX_DELTA_TIME`](Self::MAX_DELTA_TIME), `gravity` finite and at most
    /// [`limits::MAX_ACCELERATION`] long, the character's mass times the length of `gravity`
    /// times `delta_time` at most [`limits::MAX_WEIGHT_IMPULSE`], the settings valid and the
    /// filter's layers in this world; otherwise nothing happens and
    /// [`CharacterError::InvalidValue`] is returned.
    ///
    /// # Panics
    /// A panic in a filter callback is caught inside the update (the callback then rejects) and
    /// resumed after joltc has returned.
    ///
    /// # Example
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(10.0, 1.0, 10.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    ///
    /// let capsule = Shape::new_capsule(0.7, 0.4)?;
    /// let settings = CharacterSettings::new(&capsule).shape_offset(Vec3::new(0.0, 1.1, 0.0));
    /// let id = world.create_character(&settings, RVec3::new(0.0, 0.5, 0.0), Quat::IDENTITY)?;
    /// let gravity = Vec3::new(0.0, -9.81, 0.0);
    /// for _ in 0..60 {
    ///     world.character_mut(id)?.set_linear_velocity(Vec3::new(1.0, -1.0, 0.0))?;
    ///     world.update_character(id, 1.0 / 60.0, gravity, &ExtendedUpdateSettings::default(), &QueryFilter::new())?;
    /// }
    /// assert_eq!(world.character(id)?.ground_state(), GroundState::OnGround);
    /// # Ok(())
    /// # }
    /// ```
    pub fn update_character(
        &mut self,
        id: CharacterId,
        delta_time: f32,
        gravity: Vec3,
        settings: &ExtendedUpdateSettings,
        filter: &QueryFilter<'_>,
    ) -> Result<(), CharacterError> {
        if !Self::is_valid_delta_time(delta_time) {
            return Err(CharacterError::InvalidValue(
                "delta time must be finite and between MIN_DELTA_TIME and MAX_DELTA_TIME",
            ));
        }
        if !limits::is_acceleration(gravity) {
            return Err(CharacterError::InvalidValue(
                "gravity must be finite and at most limits::MAX_ACCELERATION long",
            ));
        }
        settings.validate()?;
        let entry = self.character_entry(id)?;
        if !limits::is_weight_impulse(entry.mass, gravity, delta_time) {
            return Err(CharacterError::InvalidValue(
                "mass times gravity times delta time must be at most limits::MAX_WEIGHT_IMPULSE",
            ));
        }
        let character = entry.character.as_ptr();
        filter.validate(self).map_err(query_error)?;
        let gravity = gravity.to_jph();
        let settings = settings.to_jph();
        let allocator = self.temp_allocator.as_ptr();
        with_query_filters(self, filter, |raw, _| {
            // SAFETY: `&mut self` gives this call exclusive use of the world, the character and
            // the temp allocator; `gravity` and `settings` are live locals and the filters are
            // live or null (accept everything). The filter callbacks get `&PhysicsWorld` while
            // Jolt writes the inner body's pose and pushes the ground body through the locking
            // body interface: C++ memory, not memory behind that reference, and no Rust field
            // of the world changes. CharacterVirtual queries through the system's locking
            // narrow-phase query, which releases the body lock before calling the shape filter
            // (`NarrowPhaseQuery.cpp`), so the callback's `GetShape` does not deadlock. No body
            // is removed and no shape replaced during the call, so the filters' cached root
            // shape stays valid. The character's own inner body never reaches the filters
            // (`IgnoreSingleBodyFilterChained`), and collisions between characters call no
            // filter.
            unsafe {
                JPH_CharacterVirtual_ExtendedUpdate2(
                    character,
                    delta_time,
                    &gravity,
                    &settings,
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                    allocator,
                )
            }
        })
        .map_err(query_error)
    }

    /// Recomputes a character's contacts and ground at its current pose, without moving it,
    /// colliding with what `filter` selects. Call it after moving a character with a setter or
    /// after a rotating [`rebase`](Self::rebase).
    ///
    /// Fails as [`update_character`](Self::update_character) does for the filter, and panics in
    /// filter callbacks resume the same way.
    pub fn refresh_character_contacts(
        &mut self,
        id: CharacterId,
        filter: &QueryFilter<'_>,
    ) -> Result<(), CharacterError> {
        let character = self.character_entry(id)?.character.as_ptr();
        filter.validate(self).map_err(query_error)?;
        let allocator = self.temp_allocator.as_ptr();
        with_query_filters(self, filter, |raw, _| {
            // SAFETY: as in `update_character`, without the move.
            unsafe {
                JPH_CharacterVirtual_RefreshContacts2(
                    character,
                    null(),
                    raw.object_layer,
                    raw.body,
                    raw.shape,
                    allocator,
                )
            }
        })
        .map_err(query_error)
    }
}

/// A query filter error as a character error.
fn query_error(error: crate::QueryError) -> CharacterError {
    match error {
        crate::QueryError::InvalidValue(what) => CharacterError::InvalidValue(what),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;

    #[test]
    fn default_settings_match_jolt() {
        assert!(ensure_initialized());
        let capsule = Shape::new_capsule(0.5, 0.3).unwrap();
        let ours = CharacterSettings::new(&capsule).to_jph(1);
        // SAFETY: an all-zero struct is valid input for `_Init`, which overwrites it.
        let mut jolt: JPH_CharacterVirtualSettings = unsafe { std::mem::zeroed() };
        // SAFETY: `jolt` is a live local. `_Init` creates an empty shape holding one reference,
        // released below, and empty shape settings whose one reference joltc never releases:
        // one small object per test process.
        unsafe { JPH_CharacterVirtualSettings_Init(&mut jolt) };
        let bits = |v: JPH_Vec3| [v.x, v.y, v.z].map(f32::to_bits);
        assert_eq!(bits(jolt.base.up), bits(ours.base.up));
        assert_eq!(
            bits(jolt.base.supportingVolume.normal),
            bits(ours.base.supportingVolume.normal)
        );
        assert_eq!(
            jolt.base.supportingVolume.distance,
            ours.base.supportingVolume.distance
        );
        assert_eq!(
            jolt.base.maxSlopeAngle.to_bits(),
            ours.base.maxSlopeAngle.to_bits()
        );
        assert_eq!(
            jolt.base.enhancedInternalEdgeRemoval,
            ours.base.enhancedInternalEdgeRemoval
        );
        assert_eq!(jolt.mass, ours.mass);
        assert_eq!(jolt.maxStrength, ours.maxStrength);
        assert_eq!(bits(jolt.shapeOffset), bits(ours.shapeOffset));
        assert_eq!(jolt.backFaceMode, ours.backFaceMode);
        assert_eq!(
            jolt.predictiveContactDistance,
            ours.predictiveContactDistance
        );
        assert_eq!(jolt.maxCollisionIterations, ours.maxCollisionIterations);
        assert_eq!(jolt.maxConstraintIterations, ours.maxConstraintIterations);
        assert_eq!(jolt.minTimeRemaining, ours.minTimeRemaining);
        assert_eq!(jolt.collisionTolerance, ours.collisionTolerance);
        assert_eq!(jolt.characterPadding, ours.characterPadding);
        assert_eq!(jolt.maxNumHits, ours.maxNumHits);
        assert_eq!(jolt.hitReductionCosMaxAngle, ours.hitReductionCosMaxAngle);
        assert_eq!(jolt.penetrationRecoverySpeed, ours.penetrationRecoverySpeed);
        assert!(jolt.innerBodyShape.is_null());
        assert_eq!(jolt.innerBodyLayer, ours.innerBodyLayer);
        // SAFETY: `_Init` returned the empty shape holding one reference, released once here.
        unsafe { JPH_Shape_Destroy(jolt.base.shape.cast_mut()) };
    }

    #[test]
    fn ground_states_convert_and_report_support() {
        let states = [
            (JPH_GroundState_OnGround, GroundState::OnGround, true),
            (
                JPH_GroundState_OnSteepGround,
                GroundState::OnSteepGround,
                true,
            ),
            (
                JPH_GroundState_NotSupported,
                GroundState::NotSupported,
                false,
            ),
            (JPH_GroundState_InAir, GroundState::InAir, false),
        ];
        for (raw, state, supported) in states {
            assert_eq!(GroundState::from_jph(raw), state);
            assert_eq!(state.is_supported(), supported, "{state:?}");
        }
    }
}
