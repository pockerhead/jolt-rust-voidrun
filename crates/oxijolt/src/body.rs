//! Rigid bodies: ids, creation settings, and read and write access.

use std::fmt;

use oxijolt_sys::*;

use crate::limits::{
    self, is_angular_velocity, is_friction, is_gravity_factor, is_in_frame, is_linear_velocity,
    is_mass,
};
use crate::math::{is_finite_non_negative, ROTATION_RULE};
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyError, ObjectLayer, Quat, RVec3, Shape, Vec3};

mod access;
mod handle;
mod load;
mod lock;

pub use handle::{BodyMut, BodyRef};
pub(crate) use lock::{with_locked_bodies, with_locked_body, with_read_locked_body};

/// Identifies a body in the world that created it.
///
/// Wraps Jolt's `BodyID`: an index into the world's body array and a sequence number that
/// Jolt increments each time the index is reused. Bodies created in the same order get the
/// same ids, so index and sequence are part of a world's deterministic state; digests should
/// hash [`to_raw`](Self::to_raw). The sequence number is 8 bits wide and wraps after 255 reuses
/// of one index, so a very old id can alias a new body (a Jolt limitation).
///
/// A `BodyId` also remembers its world: using it with another world returns
/// [`BodyError::WrongWorld`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyId {
    // Declared first, so ids order by Jolt's id before the world.
    raw: u32,
    pub(crate) world: WorldTag,
}

impl BodyId {
    /// Mask of the index bits (Jolt `BodyID::cMaxBodyIndex`).
    const INDEX_MASK: u32 = 0x007f_ffff;
    /// Position of the sequence number (Jolt `BodyID::cSequenceNumberShift`).
    const SEQUENCE_SHIFT: u32 = 23;

    pub(crate) fn new(raw: u32, world: WorldTag) -> Self {
        Self { raw, world }
    }

    /// The body's index in the world's body array.
    pub fn index(self) -> u32 {
        self.raw & Self::INDEX_MASK
    }

    /// How many times the index had been used before, plus one (wraps after 255).
    pub fn sequence(self) -> u8 {
        (self.raw >> Self::SEQUENCE_SHIFT) as u8
    }

    /// Index and sequence number as Jolt stores them (`BodyID::GetIndexAndSequenceNumber`).
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl fmt::Debug for BodyId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BodyId")
            .field("index", &self.index())
            .field("sequence", &self.sequence())
            .finish()
    }
}

/// How a body moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MotionType {
    /// Never moves; infinite mass.
    Static,
    /// Moved by setting its velocity; not affected by forces or collisions.
    Kinematic,
    /// Moved by forces and collisions.
    Dynamic,
}

impl MotionType {
    pub(crate) fn to_jph(self) -> JPH_MotionType {
        match self {
            Self::Static => JPH_MotionType_Static,
            Self::Kinematic => JPH_MotionType_Kinematic,
            Self::Dynamic => JPH_MotionType_Dynamic,
        }
    }

    pub(crate) fn from_jph(value: JPH_MotionType) -> Self {
        [Self::Static, Self::Kinematic, Self::Dynamic]
            .into_iter()
            .find(|motion_type| motion_type.to_jph() == value)
            // Jolt has exactly these three, asserted equal to joltc's in
            // `native/layout_checks.cpp`. This runs in Rust after the call returned, so a panic
            // here never unwinds into C++.
            .unwrap_or_else(|| unreachable!("Jolt returned unknown motion type {value}"))
    }
}

/// How Jolt integrates a moving body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MotionQuality {
    /// Discrete steps. A fast body can tunnel through thin objects. The cheapest.
    Discrete,
    /// Continuous collision detection along the linear motion of each step, so a fast body
    /// does not pass through thin objects. Rotation is not swept.
    LinearCast,
}

impl MotionQuality {
    fn to_jph(self) -> JPH_MotionQuality {
        match self {
            Self::Discrete => JPH_MotionQuality_Discrete,
            Self::LinearCast => JPH_MotionQuality_LinearCast,
        }
    }
}

/// Whether an operation wakes the body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Activation {
    /// Wake the body (make it active).
    Activate,
    /// Leave the body's sleep state as it is.
    DontActivate,
}

impl Activation {
    pub(crate) fn to_jph(self) -> JPH_Activation {
        match self {
            Self::Activate => JPH_Activation_Activate,
            Self::DontActivate => JPH_Activation_DontActivate,
        }
    }
}

/// How to create a body. Build it from [`new_dynamic`](Self::new_dynamic),
/// [`new_kinematic`](Self::new_kinematic) or [`new_static`](Self::new_static) and the setters.
///
/// The defaults are those of Jolt's `BodyCreationSettings`, except the object layer: Jolt
/// uses layer 0, oxijolt uses [`ObjectLayer::MOVING`] for dynamic and kinematic bodies and
/// [`ObjectLayer::NON_MOVING`] for static ones, matching [`CollisionLayers::default`].
///
/// [`CollisionLayers::default`]: crate::CollisionLayers::default
#[derive(Clone, Debug, PartialEq)]
pub struct BodySettings {
    pub(crate) motion_type: MotionType,
    pub(crate) object_layer: ObjectLayer,
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    friction: f32,
    restitution: f32,
    linear_damping: f32,
    angular_damping: f32,
    pub(crate) mass: Option<f32>,
    motion_quality: MotionQuality,
    gravity_factor: f32,
    allow_sleeping: bool,
    activation: Activation,
    enhanced_internal_edge_removal: bool,
}

impl Default for BodySettings {
    /// A dynamic body; see [`BodySettings`] for the values.
    fn default() -> Self {
        Self {
            motion_type: MotionType::Dynamic,
            object_layer: ObjectLayer::MOVING,
            position: RVec3::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            friction: 0.2,
            restitution: 0.0,
            linear_damping: 0.05,
            angular_damping: 0.05,
            mass: None,
            motion_quality: MotionQuality::Discrete,
            gravity_factor: 1.0,
            allow_sleeping: true,
            activation: Activation::Activate,
            enhanced_internal_edge_removal: false,
        }
    }
}

/// What a restitution must satisfy: `0..=1`.
pub(crate) const RESTITUTION_RULE: &str = "restitution must be between 0 and 1";

/// What a linear damping must satisfy ([`is_finite_non_negative`]).
pub(crate) const LINEAR_DAMPING_RULE: &str = "linear damping must be finite and not negative";

/// What Jolt asks of a shape that `Shape::MustBeStatic` reports.
pub(crate) const STATIC_SHAPE_RULE: &str = "this shape can only be used by static bodies";

impl BodySettings {
    /// A dynamic body in [`ObjectLayer::MOVING`]; the same as [`Default`].
    pub fn new_dynamic() -> Self {
        Self::default()
    }

    /// A kinematic body in [`ObjectLayer::MOVING`].
    pub fn new_kinematic() -> Self {
        Self {
            motion_type: MotionType::Kinematic,
            ..Self::default()
        }
    }

    /// A static body in [`ObjectLayer::NON_MOVING`].
    pub fn new_static() -> Self {
        Self {
            motion_type: MotionType::Static,
            object_layer: ObjectLayer::NON_MOVING,
            ..Self::default()
        }
    }

    /// Initial position of the body origin in metres, every component at most
    /// [`limits::MAX_POSITION`] in absolute value. Default the origin.
    #[must_use]
    pub fn position(mut self, value: RVec3) -> Self {
        self.position = value;
        self
    }

    /// Initial rotation, a unit quaternion. Default identity.
    #[must_use]
    pub fn rotation(mut self, value: Quat) -> Self {
        self.rotation = value;
        self
    }

    /// Initial linear velocity in m/s, finite and at most [`limits::MAX_LINEAR_VELOCITY`] long
    /// (Jolt's own length, Jolt's default maximum). Default zero.
    #[must_use]
    pub fn linear_velocity(mut self, value: Vec3) -> Self {
        self.linear_velocity = value;
        self
    }

    /// Initial angular velocity in rad/s, finite and at most [`limits::MAX_ANGULAR_VELOCITY`]
    /// long (Jolt's own length, Jolt's default maximum). Default zero.
    #[must_use]
    pub fn angular_velocity(mut self, value: Vec3) -> Self {
        self.angular_velocity = value;
        self
    }

    /// The object layer, which must exist in the world's
    /// [`CollisionLayers`](crate::CollisionLayers).
    #[must_use]
    pub fn object_layer(mut self, value: ObjectLayer) -> Self {
        self.object_layer = value;
        self
    }

    /// Friction coefficient, between 0 and [`limits::MAX_FRICTION`]. Default 0.2.
    #[must_use]
    pub fn friction(mut self, value: f32) -> Self {
        self.friction = value;
        self
    }

    /// Restitution (bounciness), between 0 and 1. Default 0.
    #[must_use]
    pub fn restitution(mut self, value: f32) -> Self {
        self.restitution = value;
        self
    }

    /// Linear damping, finite and at least 0: Jolt scales the linear velocity by
    /// `max(0, 1 - c * dt)` every step. Default 0.05.
    #[must_use]
    pub fn linear_damping(mut self, value: f32) -> Self {
        self.linear_damping = value;
        self
    }

    /// Angular damping, finite and at least 0: Jolt scales the angular velocity by
    /// `max(0, 1 - c * dt)` every step. Default 0.05.
    #[must_use]
    pub fn angular_damping(mut self, value: f32) -> Self {
        self.angular_damping = value;
        self
    }

    /// Overrides the mass in kg, between [`limits::MIN_MASS`] and [`limits::MAX_MASS`] (and large
    /// enough that Jolt can invert the scaled inertia, which must also meet the rigid body
    /// inertia floor when it is not diagonal; [`PhysicsWorld::create_body`] checks this).
    /// Without an override, a dynamic body's computed mass must lie in the same range. The
    /// inertia is computed from the shape and scaled to this mass (Jolt
    /// `EOverrideMassProperties::CalculateInertia`).
    /// By default Jolt computes mass and inertia from the shape with a density of 1000 kg/m³.
    ///
    /// [`PhysicsWorld::create_body`]: crate::PhysicsWorld::create_body
    #[must_use]
    pub fn mass(mut self, value: f32) -> Self {
        self.mass = Some(value);
        self
    }

    /// How the body is integrated. Default [`MotionQuality::Discrete`].
    #[must_use]
    pub fn motion_quality(mut self, value: MotionQuality) -> Self {
        self.motion_quality = value;
        self
    }

    /// Multiplier for the world's gravity on this body, at most [`limits::MAX_GRAVITY_FACTOR`] in
    /// absolute value. Default 1.
    #[must_use]
    pub fn gravity_factor(mut self, value: f32) -> Self {
        self.gravity_factor = value;
        self
    }

    /// Whether the body may fall asleep when it comes to rest. Default true.
    #[must_use]
    pub fn allow_sleeping(mut self, value: bool) -> Self {
        self.allow_sleeping = value;
        self
    }

    /// Whether the body starts awake. Default [`Activation::Activate`].
    #[must_use]
    pub fn activation(mut self, value: Activation) -> Self {
        self.activation = value;
        self
    }

    /// Whether Jolt removes ghost contacts of this body against internal edges of one body's
    /// shape: triangle shapes (heightfields, meshes) and touching children of a compound, Jolt's
    /// enhanced internal edge removal. Costs extra CPU per contact. Default false.
    #[must_use]
    pub fn enhanced_internal_edge_removal(mut self, value: bool) -> Self {
        self.enhanced_internal_edge_removal = value;
        self
    }

    fn validate(&self, object_layer_count: u32) -> Result<(), BodyError> {
        if self.object_layer.get() >= object_layer_count {
            return Err(BodyError::UnknownObjectLayer(self.object_layer));
        }
        self.validate_values()
    }

    /// Every check of [`validate`](Self::validate) except the object layer, which needs a world.
    pub(crate) fn validate_values(&self) -> Result<(), BodyError> {
        let invalid = |what| Err(BodyError::InvalidValue(what));
        if !is_in_frame(self.position) {
            return invalid(limits::POSITION_RULE);
        }
        if !self.rotation.is_valid_rotation() {
            return invalid(ROTATION_RULE);
        }
        if !is_linear_velocity(self.linear_velocity) {
            return invalid(limits::LINEAR_VELOCITY_RULE);
        }
        if !is_angular_velocity(self.angular_velocity) {
            return invalid(limits::ANGULAR_VELOCITY_RULE);
        }
        if !is_friction(self.friction) {
            return invalid(limits::FRICTION_RULE);
        }
        if !(0.0..=1.0).contains(&self.restitution) {
            return invalid(RESTITUTION_RULE);
        }
        if !is_finite_non_negative(self.linear_damping) {
            return invalid(LINEAR_DAMPING_RULE);
        }
        if !is_finite_non_negative(self.angular_damping) {
            return invalid("angular damping must be finite and not negative");
        }
        if !is_gravity_factor(self.gravity_factor) {
            return invalid(limits::GRAVITY_FACTOR_RULE);
        }
        if let Some(mass) = self.mass {
            if !is_mass(mass) {
                return invalid(limits::MASS_RULE);
            }
        }
        Ok(())
    }
}

/// What the mass properties of a body that is not static must satisfy ([`has_finite_inverse`]):
/// a finite inverse mass and inertia, and, for an inertia that is not diagonal, the rigid body
/// inertia floor of [`limits`].
pub(crate) const INERTIA_RULE: &str =
    "mass and shape must give a finite inverse mass and a well-conditioned inertia";

/// Zero mass and a zero inertia tensor.
const ZERO_MASS_PROPERTIES: JPH_MassProperties = JPH_MassProperties {
    mass: 0.0,
    inertia: JPH_Mat4 {
        column: [JPH_Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        }; 4],
    },
};

/// The mass and inertia Jolt gives a body made from `shape` (Jolt
/// `BodyCreationSettings::GetMassProperties` with the default inertia multiplier): the shape's
/// own, scaled to `mass` when it is overridden.
pub(crate) fn mass_properties(shape: &Shape, mass: Option<f32>) -> JPH_MassProperties {
    let mut properties = ZERO_MASS_PROPERTIES;
    // SAFETY: `shape` is live for the call; `properties` is a live local that joltc overwrites.
    unsafe { JPH_Shape_GetMassProperties(shape.as_ptr(), &mut properties) };
    if let Some(mass) = mass {
        // SAFETY: `properties` is a live local that joltc reads and overwrites.
        unsafe { JPH_MassProperties_ScaleToMass(&mut properties, mass) };
    }
    properties
}

/// Whether Jolt's `MotionProperties::SetMassProperties` derives a finite inverse mass and
/// inverse inertia from `properties`. A tiny mass, or a shape whose computed mass or inertia
/// underflows, would otherwise give infinite inverses and turn the first force into NaN. A
/// non-diagonal inertia must also pass [`limits::is_rigid_body_inertia`], so that Jolt's
/// decomposition neither asserts nor returns a moment near zero.
pub(crate) fn has_finite_inverse(properties: &JPH_MassProperties) -> bool {
    let inverse_mass = 1.0 / properties.mass;
    // When the inertia is near zero Jolt uses the inertia of a unit sphere, 2.5 / mass.
    if !(properties.mass > 0.0 && inverse_mass.is_finite() && (2.5 * inverse_mass).is_finite()) {
        return false;
    }
    let [x, y, z, _] = properties.inertia.column;
    let tensor = [[x.x, y.x, z.x], [x.y, y.y, z.y], [x.z, y.z, z.z]];
    if !tensor.iter().flatten().all(|value| value.is_finite()) {
        return false;
    }
    let off_diagonal = [x.y, x.z, y.x, y.z, z.x, z.y];
    if off_diagonal.iter().all(|&value| value == 0.0) {
        // Jolt's `EigenValueSymmetric` returns a diagonal tensor unchanged, so the diagonal is
        // exactly what Jolt inverts.
        let diagonal = [x.x, y.y, z.z];
        let length_sq: f32 = diagonal.iter().map(|value| value * value).sum();
        // Jolt `Vec3::IsNearZero` (squared length at most 1e-12) selects the unit-sphere
        // fallback.
        return length_sq <= 1.0e-12 || diagonal.iter().all(|value| (1.0 / value).is_finite());
    }
    limits::is_rigid_body_inertia(tensor.map(|row| row.map(f64::from)))
}

/// Owns a `JPH_BodyCreationSettings`, which holds its own reference to the shape.
pub(crate) struct CreationSettings(Owned<JPH_BodyCreationSettings>);

/// Body creation settings, owned whole by their owner.
impl JoltObject for JPH_BodyCreationSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes; bodies
        // created from them keep their own shape reference.
        unsafe { JPH_BodyCreationSettings_Destroy(ptr) };
    }
}

impl CreationSettings {
    /// Jolt's settings for a body of `shape` made from `settings`, which the caller validated.
    pub(crate) fn new(shape: &Shape, settings: &BodySettings) -> Result<Self, BodyError> {
        let position = settings.position.to_jph();
        let rotation = settings.rotation.to_jph();
        // SAFETY: `shape` is live for the call and the settings take their own reference to
        // it; `position` and `rotation` are live locals. The handle takes over the returned
        // settings.
        let this = unsafe {
            Owned::from_raw(JPH_BodyCreationSettings_Create3(
                shape.as_ptr(),
                &position,
                &rotation,
                settings.motion_type.to_jph(),
                settings.object_layer.get(),
            ))
        }
        .map(Self)
        .ok_or(BodyError::AllocationFailed)?;

        let ptr = this.0.as_ptr();
        let linear_velocity = settings.linear_velocity.to_jph();
        let angular_velocity = settings.angular_velocity.to_jph();
        // SAFETY: `ptr` is the live settings object owned by `this`; every input is a live local
        // or a plain value.
        unsafe {
            JPH_BodyCreationSettings_SetLinearVelocity(ptr, &linear_velocity);
            JPH_BodyCreationSettings_SetAngularVelocity(ptr, &angular_velocity);
            JPH_BodyCreationSettings_SetFriction(ptr, settings.friction);
            JPH_BodyCreationSettings_SetRestitution(ptr, settings.restitution);
            JPH_BodyCreationSettings_SetLinearDamping(ptr, settings.linear_damping);
            JPH_BodyCreationSettings_SetAngularDamping(ptr, settings.angular_damping);
            JPH_BodyCreationSettings_SetMotionQuality(ptr, settings.motion_quality.to_jph());
            JPH_BodyCreationSettings_SetGravityFactor(ptr, settings.gravity_factor);
            JPH_BodyCreationSettings_SetAllowSleeping(ptr, settings.allow_sleeping);
            JPH_BodyCreationSettings_SetEnhancedInternalEdgeRemoval(
                ptr,
                settings.enhanced_internal_edge_removal,
            );
        }
        if let Some(mass) = settings.mass {
            // Jolt ignores the inertia with `CalculateInertia` (`BodyCreationSettings.cpp`).
            let mass_properties = JPH_MassProperties {
                mass,
                ..ZERO_MASS_PROPERTIES
            };
            // SAFETY: `ptr` is live; `mass_properties` is a live local that joltc copies.
            unsafe {
                JPH_BodyCreationSettings_SetOverrideMassProperties(
                    ptr,
                    JPH_OverrideMassProperties_CalculateInertia,
                );
                JPH_BodyCreationSettings_SetMassPropertiesOverride(ptr, &mass_properties);
            }
        }
        Ok(this)
    }

    /// The settings object, still owned by `self`.
    pub(crate) fn as_ptr(&self) -> *mut JPH_BodyCreationSettings {
        self.0.as_ptr()
    }
}

/// Jolt's `BodyID::cInvalidBodyID`, returned by `CreateAndAddBody` when the world is full.
pub(crate) const INVALID_BODY_ID: JPH_BodyID = 0xffff_ffff;

#[cfg(test)]
mod tests;
