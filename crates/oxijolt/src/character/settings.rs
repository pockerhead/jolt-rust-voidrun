//! Creation and update settings of a character.

use std::fmt;
use std::ptr::null;

use oxijolt_sys::*;

use super::{InnerBody, INNER_BODY_INERTIA_RULE, INVALID_ID};
use crate::body::{has_finite_inverse, mass_properties};
use crate::limits;
use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::{CharacterError, Shape, Vec3};

/// How to create a character, for [`PhysicsWorld::create_character`](crate::PhysicsWorld::create_character). Build it with
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
    pub(super) mass: f32,
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
    pub(super) inner_body: Option<InnerBody<'a>>,
    pub(super) collide_with_characters: bool,
    pub(super) user_data: u64,
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
    /// [`PhysicsWorld::create_character`](crate::PhysicsWorld::create_character) rejects other shapes, because the character moves by
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
    /// [`CharacterMut::set_up`](crate::CharacterMut::set_up).
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
    /// [`PhysicsWorld::update_character`](crate::PhysicsWorld::update_character).
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
    /// [`CharacterRef::max_hits_exceeded`](crate::CharacterRef::max_hits_exceeded).
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
    /// exists, [`PhysicsWorld::remove_body`](crate::PhysicsWorld::remove_body) refuses to remove it.
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
            return invalid(UP_RULE);
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
    pub(super) fn to_jph(&self, id: u32) -> JPH_CharacterVirtualSettings {
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

/// What a character's up must satisfy ([`is_unit`]).
pub(super) const UP_RULE: &str = "up must be a finite unit vector";

/// Whether `v` is finite and of unit length within Jolt's `Vec3::IsNormalized` tolerance
/// (`|length² − 1| <= 1e-5`).
pub(super) fn is_unit(v: Vec3) -> bool {
    v.is_finite() && (v.dot(v) - 1.0).abs() <= 1.0e-5
}

/// What an update does besides moving the character, for [`PhysicsWorld::update_character`](crate::PhysicsWorld::update_character)
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

    pub(super) fn validate(&self) -> Result<(), CharacterError> {
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

    pub(super) fn to_jph(self) -> JPH_ExtendedUpdateSettings {
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
