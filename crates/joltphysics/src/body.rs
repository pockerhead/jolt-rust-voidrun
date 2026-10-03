//! Rigid bodies: ids, creation settings, and read and write access.

use std::any::Any;
use std::ffi::c_void;
use std::fmt;
use std::marker::PhantomData;
use std::ops::Deref;
use std::panic::{catch_unwind, resume_unwind, AssertUnwindSafe};
use std::ptr::{null, NonNull};

use joltphysics_sys::*;

use crate::limits::{
    self, is_angular_velocity, is_friction, is_gravity_factor, is_in_frame, is_linear_velocity,
    is_mass,
};
use crate::math::is_finite_non_negative;
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyError, ObjectLayer, PhysicsWorld, Quat, RVec3, Shape, Vec3};

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
/// uses layer 0, joltphysics uses [`ObjectLayer::MOVING`] for dynamic and kinematic bodies and
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
    /// enough that Jolt can invert the scaled inertia; [`PhysicsWorld::create_body`] checks
    /// this). Without an override, a dynamic body's computed mass must lie in the same range. The
    /// inertia is computed
    /// from the shape and scaled to this mass (Jolt `EOverrideMassProperties::CalculateInertia`).
    /// By default Jolt computes mass and inertia from the shape with a density of 1000 kg/m³.
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
            return invalid("position must be finite and within limits::MAX_POSITION");
        }
        if !self.rotation.is_valid_rotation() {
            return invalid("rotation must be a finite unit quaternion");
        }
        if !is_linear_velocity(self.linear_velocity) {
            return invalid(LINEAR_VELOCITY_RULE);
        }
        if !is_angular_velocity(self.angular_velocity) {
            return invalid(ANGULAR_VELOCITY_RULE);
        }
        if !is_friction(self.friction) {
            return invalid("friction must be finite and between 0 and limits::MAX_FRICTION");
        }
        if !(0.0..=1.0).contains(&self.restitution) {
            return invalid("restitution must be between 0 and 1");
        }
        if !is_finite_non_negative(self.linear_damping) {
            return invalid("linear damping must be finite and not negative");
        }
        if !is_finite_non_negative(self.angular_damping) {
            return invalid("angular damping must be finite and not negative");
        }
        if !is_gravity_factor(self.gravity_factor) {
            return invalid("gravity factor must be finite and within limits::MAX_GRAVITY_FACTOR");
        }
        if let Some(mass) = self.mass {
            if !is_mass(mass) {
                return invalid(MASS_RULE);
            }
        }
        Ok(())
    }
}

/// What a caller-given linear velocity must satisfy.
pub(crate) const LINEAR_VELOCITY_RULE: &str =
    "linear velocity must be finite and at most limits::MAX_LINEAR_VELOCITY long";
/// What a caller-given angular velocity must satisfy.
pub(crate) const ANGULAR_VELOCITY_RULE: &str =
    "angular velocity must be finite and at most limits::MAX_ANGULAR_VELOCITY long";
/// What a dynamic body's mass must satisfy.
pub(crate) const MASS_RULE: &str = "mass must be between limits::MIN_MASS and limits::MAX_MASS";

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
/// underflows, would otherwise give infinite inverses and turn the first force into NaN.
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
    has_well_conditioned_principal_moments(tensor.map(|row| row.map(f64::from)))
}

/// Whether Jolt can decompose a symmetric, non-diagonal inertia tensor into principal moments
/// that are safe to invert.
///
/// The squared Frobenius norm is the sum of the squared principal moments, so a tensor below
/// Jolt's near-zero limit (`1e-12`, halved to stay clear of f32 rounding) gets Jolt's
/// unit-sphere inertia. Otherwise `det / |I|_F^2` is a lower bound of the smallest principal
/// moment (the product of the other two is at most `|I|_F^2`). Jolt decomposes the tensor in
/// f32, so a principal moment below its rounding error (a few `f32::EPSILON * |I|`) could come
/// back as zero or negative and give an infinite or negative inverse inertia. The floor of
/// `1e-5 * |I|_F` is about 80 times that error; it rejects only extremely slender rotated or
/// offset bodies (aspect ratios of several hundred), which is conservative.
fn has_well_conditioned_principal_moments(tensor: [[f64; 3]; 3]) -> bool {
    let frobenius_sq: f64 = tensor.iter().flatten().map(|value| value * value).sum();
    if frobenius_sq <= 0.5e-12 {
        return true;
    }
    let [[a, b, c], [d, e, f], [g, h, i]] = tensor;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    det > 0.0 && det / frobenius_sq >= 1.0e-5 * frobenius_sq.sqrt()
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
const INVALID_BODY_ID: JPH_BodyID = 0xffff_ffff;

impl PhysicsWorld {
    /// Creates a body from `shape` and adds it to the world. The body keeps its own reference
    /// to the shape, so `shape` may be dropped afterwards.
    ///
    /// Fails with [`BodyError::InvalidValue`] when a setting is out of range, when a dynamic or
    /// kinematic body uses a shape that only static bodies may use (a heightfield, or a
    /// compound that contains one), when a dynamic or kinematic body's mass or inertia
    /// (overridden, or computed from a tiny or very slender shape) is too small for Jolt to
    /// invert, and when a dynamic body's mass (overridden or computed) is outside
    /// [`limits::MIN_MASS`]`..=`[`limits::MAX_MASS`]. Kinematic bodies are exempt from the mass
    /// range: Jolt gives them infinite mass in the solver.
    pub fn create_body(
        &mut self,
        shape: &Shape,
        settings: &BodySettings,
    ) -> Result<BodyId, BodyError> {
        settings.validate(self.object_layer_count)?;
        // Jolt itself never checks this when creating a body.
        if settings.motion_type != MotionType::Static
            // SAFETY: `shape` is live for the call; the getter only reads it.
            && unsafe { JPH_Shape_MustBeStatic(shape.as_ptr()) }
        {
            return Err(BodyError::InvalidValue(
                "this shape can only be used by static bodies",
            ));
        }
        // Jolt computes mass properties for every body that is not static
        // (`BodyCreationSettings::HasMassProperties`).
        if settings.motion_type != MotionType::Static {
            let properties = mass_properties(shape, settings.mass);
            if !has_finite_inverse(&properties) {
                return Err(BodyError::InvalidValue(
                    "mass and shape give an infinite inverse mass or inertia",
                ));
            }
            if settings.motion_type == MotionType::Dynamic && !is_mass(properties.mass) {
                return Err(BodyError::InvalidValue(MASS_RULE));
            }
        }
        let creation = CreationSettings::new(shape, settings)?;
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation`
        // is a fully set up settings object whose layer exists in this world.
        let raw = unsafe {
            JPH_BodyInterface_CreateAndAddBody(
                self.body_interface.as_ptr(),
                creation.as_ptr(),
                settings.activation.to_jph(),
            )
        };
        if raw == INVALID_BODY_ID {
            return Err(BodyError::TooManyBodies);
        }
        Ok(BodyId::new(raw, self.tag))
    }

    /// `Ok` if `id` names a body that is in this world now.
    pub(crate) fn check(&self, id: BodyId) -> Result<(), BodyError> {
        if id.world != self.tag {
            return Err(BodyError::WrongWorld(id));
        }
        // SAFETY: the body interface belongs to this live world. `IsAdded` locks the body and
        // compares index and sequence number, so any id value is safe to pass.
        if unsafe { JPH_BodyInterface_IsAdded(self.body_interface.as_ptr(), id.raw) } {
            Ok(())
        } else {
            Err(BodyError::NotFound(id))
        }
    }

    /// Whether `id` names a body that is in this world now.
    pub fn contains(&self, id: BodyId) -> bool {
        self.check(id).is_ok()
    }

    /// Read access to a body.
    pub fn body(&self, id: BodyId) -> Result<BodyRef<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyRef {
            body_interface: self.body_interface,
            body_lock_interface: self.body_lock_interface,
            id,
            _world: PhantomData,
        })
    }

    /// Read and write access to a body.
    pub fn body_mut(&mut self, id: BodyId) -> Result<BodyMut<'_>, BodyError> {
        self.check(id)?;
        Ok(BodyMut {
            inner: BodyRef {
                body_interface: self.body_interface,
                body_lock_interface: self.body_lock_interface,
                id,
                _world: PhantomData,
            },
            _world: PhantomData,
        })
    }

    /// Removes a body from the world and destroys it.
    ///
    /// Jolt does not wake the bodies around a removed one by itself, so joltphysics wakes every
    /// non-static body whose current bounds overlap (or touch) the removed body's bounds, in
    /// body-id order. The woken set depends only on body poses, not on broad-phase maintenance or
    /// worker threads; a stack whose bottom is removed falls.
    ///
    /// The inner body of a character cannot be removed this way
    /// ([`BodyError::OwnedByCharacter`]); it goes with
    /// [`remove_character`](Self::remove_character). The chassis of a vehicle cannot be removed
    /// while the vehicle exists ([`BodyError::UsedByVehicle`]); remove the vehicle first with
    /// [`remove_vehicle`](Self::remove_vehicle). A part of a ragdoll goes only with its ragdoll
    /// ([`BodyError::OwnedByRagdoll`], [`remove_ragdoll`](Self::remove_ragdoll)).
    pub fn remove_body(&mut self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
        // The character's destructor destroys its inner body, and Jolt does not validate ids in
        // `DestroyBody`: removing it here first would make that a double destroy. Any future
        // API that destroys bodies needs the same check.
        if self.is_inner_body(id) {
            return Err(BodyError::OwnedByCharacter(id));
        }
        // A vehicle keeps a pointer to its chassis and dereferences it on every step. Any future
        // API that destroys bodies or changes their motion type must consult the vehicle bodies
        // the same way.
        if self.is_vehicle_body(id) {
            return Err(BodyError::UsedByVehicle(id));
        }
        // A ragdoll destroys its parts when it is released, and Jolt does not validate ids in
        // `DestroyBody`, so removing a part here would make that a double destroy.
        if self.is_ragdoll_body(id) {
            return Err(BodyError::OwnedByRagdoll(id));
        }
        let mut bounds = JPH_AABox {
            min: Vec3::ZERO.to_jph(),
            max: Vec3::ZERO.to_jph(),
        };
        with_locked_body(self.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure; `bounds` is
            // a live local.
            unsafe { JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut bounds) };
        })
        .ok_or(BodyError::NotFound(id))?;
        // SAFETY: `check` just confirmed the id names a body in this world, and `&mut self`
        // keeps anyone else from removing it in between, so the body is removed exactly once
        // (Jolt does not validate ids in `DestroyBody`). This thread holds no body lock.
        unsafe { JPH_BodyInterface_RemoveAndDestroyBody(self.body_interface.as_ptr(), id.raw) };
        self.wake_bodies_overlapping(&bounds);
        Ok(())
    }

    /// Whether bodies `a` and `b` touched in the last [`step`](Self::step) (Jolt
    /// `PhysicsSystem::WereBodiesInContact`).
    ///
    /// Jolt answers from the contact cache of the last step, and only for pairs of which at
    /// least one body was awake in it; bodies removed since are allowed. Fails with
    /// [`BodyError::WrongWorld`] for an id of another world.
    pub fn were_bodies_in_contact(&self, a: BodyId, b: BodyId) -> Result<bool, BodyError> {
        for id in [a, b] {
            if id.world != self.tag {
                return Err(BodyError::WrongWorld(id));
            }
        }
        // SAFETY: the system is live and no step runs (`step` needs `&mut self`); Jolt looks the
        // pair up in its contact cache and never dereferences a body, so any id is safe.
        Ok(unsafe { JPH_PhysicsSystem_WereBodiesInContact(self.system.as_ptr(), a.raw, b.raw) })
    }

    /// Wakes every non-static body whose world bounds overlap `bounds`, in body-id order.
    ///
    /// Jolt's broad phase keeps widened bounds for moved bodies until its next maintenance, so
    /// which bodies it reports depends on that history (Jolt docs, "Deterministic Simulation").
    /// Here it only proposes candidates; each is kept only when its exact bounds overlap.
    pub(crate) fn wake_bodies_overlapping(&mut self, bounds: &JPH_AABox) {
        let candidates = self.broad_phase_bodies(bounds);
        let woken = self.overlapping_movable_bodies(bounds, &candidates);
        if woken.is_empty() {
            return;
        }
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `woken` holds
        // `woken.len()` ids and lives for the call. This thread holds no body lock:
        // `overlapping_movable_bodies` has released its locks, which `ActivateBodies` takes
        // again (Jolt's body mutexes are not recursive).
        unsafe {
            JPH_BodyInterface_ActivateBodies(
                self.body_interface.as_ptr(),
                woken.as_ptr(),
                woken.len() as u32,
            );
        }
    }

    /// The ids of the bodies whose broad-phase bounds overlap `bounds`, sorted and without
    /// duplicates. The broad phase may keep widened bounds, so callers check exact bounds.
    pub(crate) fn broad_phase_bodies(&self, bounds: &JPH_AABox) -> Vec<JPH_BodyID> {
        let mut hits = BroadPhaseHits {
            ids: Vec::with_capacity(self.body_count() as usize),
            panic: None,
        };
        // SAFETY: the broad-phase query belongs to this live world; a step needs
        // `&mut PhysicsWorld`, so none runs during this `&self` call. `bounds` is live for the
        // call. `hits` is a live local that only the collector touches during the call, as the
        // `BroadPhaseHits` it expects. Null filters select joltc's accept-all defaults.
        unsafe {
            JPH_BroadPhaseQuery_CollideAABox(
                self.broad_phase_query.as_ptr(),
                bounds,
                Some(collect_broad_phase_hit),
                (&mut hits as *mut BroadPhaseHits).cast(),
                null(),
                null(),
            );
        }
        if let Some(payload) = hits.panic {
            resume_unwind(payload);
        }
        let mut candidates = hits.ids;
        // Jolt's `BodyID::operator<` compares the raw value.
        candidates.sort_unstable();
        candidates.dedup();
        candidates
    }

    /// Those of the sorted `candidates` that are non-static bodies whose world bounds overlap
    /// `bounds`, in the same order. Locks all candidates at once and releases them before
    /// returning.
    fn overlapping_movable_bodies(
        &self,
        bounds: &JPH_AABox,
        candidates: &[JPH_BodyID],
    ) -> Vec<JPH_BodyID> {
        let mut woken = Vec::new();
        if candidates.is_empty() {
            return woken;
        }
        // SAFETY: the lock interface belongs to this live world. joltc copies the ids into the
        // lock object, so `candidates` only has to live for the call. The handle takes over the
        // lock and releases it when it goes out of scope.
        let lock = unsafe {
            Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
                self.body_lock_interface.as_ptr(),
                candidates.as_ptr(),
                candidates.len() as u32,
            ))
        };
        let Some(lock) = lock else {
            return woken;
        };
        for (index, &candidate) in candidates.iter().enumerate() {
            // SAFETY: `lock` is live and holds `candidates.len()` ids, so `index` is in range.
            // Jolt returns null unless the id still names a live body.
            let body = unsafe { JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), index as u32) };
            let Some(body) = NonNull::new(body) else {
                continue;
            };
            let mut candidate_bounds = JPH_AABox {
                min: Vec3::ZERO.to_jph(),
                max: Vec3::ZERO.to_jph(),
            };
            // SAFETY: `body` is locked for writing while `lock` lives; the getters only read
            // it, and `candidate_bounds` is a live local.
            let is_static = unsafe {
                JPH_Body_GetWorldSpaceBounds(body.as_ptr(), &mut candidate_bounds);
                JPH_Body_IsStatic(body.as_ptr())
            };
            if !is_static && bounds_overlap(bounds, &candidate_bounds) {
                woken.push(candidate);
            }
        }
        woken
    }
}

/// What the broad-phase collector of `PhysicsWorld::broad_phase_bodies` gathers.
struct BroadPhaseHits {
    /// Candidate ids, at most as many as the capacity reserved before the query.
    ids: Vec<JPH_BodyID>,
    /// The payload of a panic caught in the collector, re-raised after the query.
    panic: Option<Box<dyn Any + Send>>,
}

/// Jolt's `CollisionCollectorTraitsCollideShape::InitialEarlyOutFraction`: keep collecting.
const KEEP_COLLECTING: f32 = f32::MAX;
/// Jolt's `CollisionCollectorTraitsCollideShape::ShouldEarlyOutFraction`: stop the query.
const STOP_COLLECTING: f32 = -f32::MAX;

/// Broad-phase collector of `PhysicsWorld::broad_phase_bodies`: records each candidate id
/// without allocating and never lets a panic unwind into joltc.
///
/// # Safety
/// Called only by joltc during the `JPH_BroadPhaseQuery_CollideAABox` call of
/// `broad_phase_bodies`, with that call's live `*mut BroadPhaseHits` as `user_data`, which
/// nothing else accesses during the call.
unsafe extern "C" fn collect_broad_phase_hit(user_data: *mut c_void, body: JPH_BodyID) -> f32 {
    // SAFETY: guaranteed by the caller (function contract); this is the only reference to the
    // hits during the call.
    let hits = unsafe { &mut *user_data.cast::<BroadPhaseHits>() };
    let ids = &mut hits.ids;
    let collected = catch_unwind(AssertUnwindSafe(|| {
        if ids.len() < ids.capacity() {
            ids.push(body);
        }
    }));
    match collected {
        Ok(()) => KEEP_COLLECTING,
        Err(payload) => {
            hits.panic = Some(payload);
            STOP_COLLECTING
        }
    }
}

/// Whether two boxes overlap, touching included (Jolt `AABox::Overlaps`).
fn bounds_overlap(a: &JPH_AABox, b: &JPH_AABox) -> bool {
    let axis = |a_min: f32, a_max: f32, b_min: f32, b_max: f32| a_min <= b_max && b_min <= a_max;
    axis(a.min.x, a.max.x, b.min.x, b.max.x)
        && axis(a.min.y, a.max.y, b.min.y, b.max.y)
        && axis(a.min.z, a.max.z, b.min.z, b.max.z)
}

/// A body write lock. Destroying it deletes Jolt's `BodyLockMultiWrite`, which unlocks the
/// bodies, and frees the joltc wrapper; as an `Owned` it is released also while unwinding.
impl JoltObject for JPH_BodyLockMultiWrite {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the lock (trait contract), which is released exactly once here.
        unsafe { JPH_BodyLockMultiWrite_Destroy(ptr) };
    }
}

/// Runs `f` with the body locked for writing; `None` if the id no longer resolves.
///
/// The body pointer never leaves `f`. `f` must not call the body interface for the same body:
/// Jolt's body mutexes are not recursive.
pub(crate) fn with_locked_body<R>(
    lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    f: impl FnOnce(NonNull<JPH_Body>) -> R,
) -> Option<R> {
    let raw = id.raw;
    // SAFETY: the lock interface belongs to a live world. joltc copies the one id into the
    // lock object, so `raw` only has to live for the call. The handle takes over the lock.
    let lock = unsafe {
        Owned::from_raw(JPH_BodyLockInterface_LockMultiWrite(
            lock_interface.as_ptr(),
            &raw,
            1,
        ))
    }?;
    // SAFETY: `lock` is live and holds exactly one id, at index 0. Jolt returns null unless
    // index and sequence number both match a live body (`BodyManager::TryGetBody`).
    let body = NonNull::new(unsafe { JPH_BodyLockMultiWrite_GetBody(lock.as_ptr(), 0) })?;
    Some(f(body))
}

/// A body read lock. Destroying it deletes Jolt's `BodyLockMultiRead`, which unlocks the
/// bodies, and frees the joltc wrapper; as an `Owned` it is released also while unwinding.
impl JoltObject for JPH_BodyLockMultiRead {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the lock (trait contract), which is released exactly once here.
        unsafe { JPH_BodyLockMultiRead_Destroy(ptr) };
    }
}

/// Runs `f` with the body locked for reading; `None` if the id no longer resolves.
///
/// The body pointer never leaves `f`, and `f` only reads through it. `f` must not lock the same
/// body for writing.
pub(crate) fn with_read_locked_body<R>(
    lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    f: impl FnOnce(NonNull<JPH_Body>) -> R,
) -> Option<R> {
    let raw = id.raw;
    // SAFETY: the lock interface belongs to a live world. joltc copies the one id into the
    // lock object, so `raw` only has to live for the call. The handle takes over the lock.
    let lock = unsafe {
        Owned::from_raw(JPH_BodyLockInterface_LockMultiRead(
            lock_interface.as_ptr(),
            &raw,
            1,
        ))
    }?;
    // SAFETY: `lock` is live and holds exactly one id, at index 0. Jolt returns null unless
    // index and sequence number both match a live body (`BodyManager::TryGetBody`).
    let body = NonNull::new(unsafe { JPH_BodyLockMultiRead_GetBody(lock.as_ptr(), 0) }.cast_mut())?;
    Some(f(body))
}

/// Read access to one body, borrowed from its world.
///
/// Each method takes Jolt's body lock for the duration of one call and returns a plain value.
/// Not `Send` or `Sync`; share the world instead.
///
/// Positions read back with the same bits they were set with when the shape's centre of mass
/// is at its origin (boxes, spheres, capsules, cylinders), except that the sign of a zero
/// component is not kept: `-0.0` may read back as `+0.0`, because Jolt stores
/// `position + rotation * centre_of_mass`. For other shapes, compounds in particular, the
/// position involves arithmetic.
pub struct BodyRef<'w> {
    body_interface: NonNull<JPH_BodyInterface>,
    body_lock_interface: NonNull<JPH_BodyLockInterface>,
    id: BodyId,
    _world: PhantomData<&'w PhysicsWorld>,
}

impl BodyRef<'_> {
    /// The body's id.
    pub fn id(&self) -> BodyId {
        self.id
    }

    fn interface(&self) -> *mut JPH_BodyInterface {
        self.body_interface.as_ptr()
    }

    /// Position of the body origin (not of the centre of mass) in metres.
    pub fn position(&self) -> RVec3 {
        let mut value = RVec3::ZERO.to_jph();
        // SAFETY: the body interface belongs to the world this view borrows, which holds the
        // body (`check`) and cannot change while the borrow lasts; `value` is a live local.
        unsafe { JPH_BodyInterface_GetPosition(self.interface(), self.id.raw, &mut value) };
        RVec3::from_jph(value)
    }

    /// Rotation.
    pub fn rotation(&self) -> Quat {
        let mut value = Quat::IDENTITY.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetRotation(self.interface(), self.id.raw, &mut value) };
        Quat::from_jph(value)
    }

    /// Linear velocity of the centre of mass in m/s.
    pub fn linear_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetLinearVelocity(self.interface(), self.id.raw, &mut value) };
        Vec3::from_jph(value)
    }

    /// Angular velocity in rad/s.
    pub fn angular_velocity(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_GetAngularVelocity(self.interface(), self.id.raw, &mut value) };
        Vec3::from_jph(value)
    }

    /// How the body moves.
    pub fn motion_type(&self) -> MotionType {
        // SAFETY: as in `position`.
        MotionType::from_jph(unsafe {
            JPH_BodyInterface_GetMotionType(self.interface(), self.id.raw)
        })
    }

    /// Whether the body is awake. Static bodies are never active.
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `position`.
        unsafe { JPH_BodyInterface_IsActive(self.interface(), self.id.raw) }
    }

    /// Whether a body that can move has fallen asleep. Static bodies never sleep.
    pub fn is_sleeping(&self) -> bool {
        self.motion_type() != MotionType::Static && !self.is_active()
    }

    /// Mass in kg of a dynamic body; `None` for static and kinematic bodies, whose mass is
    /// infinite. For the caller's own gravity `g` (m/s²) on a body created with
    /// [`gravity_factor(0.0)`](BodySettings::gravity_factor), add the force `g * mass` every
    /// step with [`BodyMut::add_force`].
    pub fn mass(&self) -> Option<f32> {
        with_read_locked_body(self.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. A dynamic body
            // has motion properties, so the unchecked getter reads a live member; the getters
            // only read.
            unsafe {
                JPH_Body_IsDynamic(body.as_ptr()).then(|| {
                    1.0 / JPH_MotionProperties_GetInverseMassUnchecked(
                        JPH_Body_GetMotionProperties(body.as_ptr()),
                    )
                })
            }
        })
        .flatten()
    }
}

/// Read and write access to one body, borrowed mutably from its world.
///
/// Dereferences to [`BodyRef`] for reads. Setters validate their input before it reaches
/// Jolt. Units: metres, m/s, rad/s, newtons and newton-metres. Static bodies ignore velocity
/// and force writes. Not `Send` or `Sync`.
pub struct BodyMut<'w> {
    inner: BodyRef<'w>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl<'w> Deref for BodyMut<'w> {
    type Target = BodyRef<'w>;

    fn deref(&self) -> &BodyRef<'w> {
        &self.inner
    }
}

/// What a caller-given body position must satisfy.
const POSITION_RULE: &str = "position must be finite and within limits::MAX_POSITION";

/// Bound on each product `lever_i * force_j` of a point force's torque, so that Jolt's `f32`
/// cross product (`Body.inl:127-131`) has finite products and a finite difference even when the
/// two products cancel.
const F32_PRODUCT_HEADROOM: f64 = 1.0e37;

/// What [`BodyMut::check_load`] reads from a dynamic body.
struct LoadState {
    force: Vec3,
    torque: Vec3,
    inverse_mass: f32,
    inverse_inertia: Vec3,
    center_of_mass: RVec3,
}

/// The torque, in `f64`, of `force` at `point` on a body whose centre of mass is
/// `center_of_mass`; an error when Jolt's `f32` arithmetic for it could overflow.
fn point_torque(force: Vec3, point: RVec3, center_of_mass: RVec3) -> Result<[f64; 3], BodyError> {
    // Jolt converts the lever to `f32` (`Vec3(inPosition - mPosition)`). `Real` is `f32`
    // without the `double-precision` feature, so the casts are no-ops there.
    #[allow(clippy::unnecessary_cast)]
    let lever = [
        (point.x - center_of_mass.x) as f32,
        (point.y - center_of_mass.y) as f32,
        (point.z - center_of_mass.z) as f32,
    ];
    require(
        lever.iter().all(|c| c.is_finite()),
        "point is too far from the body's centre of mass",
    )?;
    let lever = lever.map(f64::from);
    let force = [force.x, force.y, force.z].map(f64::from);
    let products_fit = (0..3)
        .all(|i| (0..3).all(|j| i == j || (lever[i] * force[j]).abs() <= F32_PRODUCT_HEADROOM));
    require(
        products_fit,
        "force at this point would overflow Jolt's torque arithmetic",
    )?;
    Ok([
        lever[1] * force[2] - lever[2] * force[1],
        lever[2] * force[0] - lever[0] * force[2],
        lever[0] * force[1] - lever[1] * force[0],
    ])
}

fn require(valid: bool, what: &'static str) -> Result<(), BodyError> {
    if valid {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(what))
    }
}

impl BodyMut<'_> {
    /// Moves the body origin to `position`, every component at most [`limits::MAX_POSITION`] in
    /// absolute value.
    pub fn set_position(
        &mut self,
        position: RVec3,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(is_in_frame(position), POSITION_RULE)?;
        let mut position = position.to_jph();
        // SAFETY: the world is borrowed mutably through this view and holds the body; joltc only
        // reads `position`, a live local.
        unsafe {
            JPH_BodyInterface_SetPosition(
                self.interface(),
                self.id.raw,
                &mut position,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets the rotation, a finite unit quaternion.
    pub fn set_rotation(
        &mut self,
        rotation: Quat,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(
            rotation.is_valid_rotation(),
            "rotation must be a finite unit quaternion",
        )?;
        let mut rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetRotation(
                self.interface(),
                self.id.raw,
                &mut rotation,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets position and rotation together. The position follows the rule of
    /// [`set_position`](Self::set_position).
    pub fn set_position_and_rotation(
        &mut self,
        position: RVec3,
        rotation: Quat,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(is_in_frame(position), POSITION_RULE)?;
        require(
            rotation.is_valid_rotation(),
            "rotation must be a finite unit quaternion",
        )?;
        let position = position.to_jph();
        let rotation = rotation.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetPositionAndRotation(
                self.interface(),
                self.id.raw,
                &position,
                &rotation,
                activation.to_jph(),
            )
        };
        Ok(())
    }

    /// Sets the linear velocity in m/s, finite and at most [`limits::MAX_LINEAR_VELOCITY`] long
    /// (Jolt would clamp a faster one; it is rejected, as at creation). Wakes the body when the
    /// velocity is not near zero.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        require(is_linear_velocity(velocity), LINEAR_VELOCITY_RULE)?;
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_SetLinearVelocity(self.interface(), self.id.raw, &velocity) };
        Ok(())
    }

    /// Sets the angular velocity in rad/s, finite and at most [`limits::MAX_ANGULAR_VELOCITY`]
    /// long (Jolt would clamp a faster one; it is rejected, as at creation). Wakes the body when
    /// the velocity is not near zero.
    pub fn set_angular_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        require(is_angular_velocity(velocity), ANGULAR_VELOCITY_RULE)?;
        let mut velocity = velocity.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_SetAngularVelocity(self.interface(), self.id.raw, &mut velocity)
        };
        Ok(())
    }

    /// Adds a force in newtons at the centre of mass for the next step, and wakes the body.
    ///
    /// Jolt clears accumulated forces after every step, and a body that gets a force every step
    /// never falls asleep.
    ///
    /// The force must be finite, and on a dynamic body the force accumulated this step including
    /// this one may give the body at most [`limits::MAX_ACCELERATION`] (`|F| / mass`); otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes. Static and kinematic bodies
    /// ignore forces.
    pub fn add_force(&mut self, force: Vec3) -> Result<(), BodyError> {
        require(force.is_finite(), "force must be finite")?;
        self.check_load(force, None, Vec3::ZERO)?;
        let mut force = force.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddForce(self.interface(), self.id.raw, &mut force) };
        Ok(())
    }

    /// Adds a force in newtons applied at a world-space `point`, which also adds the matching
    /// torque, and wakes the body.
    ///
    /// The force must be finite and the point within [`limits::MAX_POSITION`]. On a dynamic body
    /// the force accumulated this step including this one may give the body at most
    /// [`limits::MAX_ACCELERATION`], the torque at most [`limits::MAX_ANGULAR_ACCELERATION`]
    /// (`|τ|` times the largest principal inverse inertia), and Jolt's `f32` torque
    /// `(point - centre_of_mass) × force` must not overflow; otherwise
    /// [`BodyError::InvalidValue`] is returned and nothing changes.
    pub fn add_force_at_point(&mut self, force: Vec3, point: RVec3) -> Result<(), BodyError> {
        require(force.is_finite(), "force must be finite")?;
        require(
            is_in_frame(point),
            "point must be finite and within limits::MAX_POSITION",
        )?;
        self.check_load(force, Some(point), Vec3::ZERO)?;
        let mut force = force.to_jph();
        let mut point = point.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_AddForce2(self.interface(), self.id.raw, &mut force, &mut point)
        };
        Ok(())
    }

    /// Adds a torque in newton-metres for the next step, and wakes the body.
    ///
    /// The torque must be finite, and on a dynamic body the torque accumulated this step
    /// including this one may give the body at most [`limits::MAX_ANGULAR_ACCELERATION`] (`|τ|`
    /// times the largest principal inverse inertia); otherwise [`BodyError::InvalidValue`] is
    /// returned and nothing changes.
    pub fn add_torque(&mut self, torque: Vec3) -> Result<(), BodyError> {
        require(torque.is_finite(), "torque must be finite")?;
        self.check_load(Vec3::ZERO, None, torque)?;
        let mut torque = torque.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddTorque(self.interface(), self.id.raw, &mut torque) };
        Ok(())
    }

    /// Checks that adding `force` (at `point`, or at the centre of mass) and `torque` keeps this
    /// step's accumulated load within the acceleration bounds of [`limits`]. Reads the body
    /// under a read lock that is released before the caller adds the load: Jolt's body mutexes
    /// are not recursive.
    fn check_load(&self, force: Vec3, point: Option<RVec3>, torque: Vec3) -> Result<(), BodyError> {
        let state = with_read_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for reading for the duration of the closure. A dynamic
            // body has motion properties, so the unchecked getter reads a live member; the
            // getters only read, and every output is a live local.
            unsafe {
                if !JPH_Body_IsDynamic(body.as_ptr()) {
                    return None;
                }
                let motion = JPH_Body_GetMotionProperties(body.as_ptr());
                let mut accumulated_force = Vec3::ZERO.to_jph();
                let mut accumulated_torque = Vec3::ZERO.to_jph();
                let mut inverse_inertia = Vec3::ZERO.to_jph();
                let mut center_of_mass = RVec3::ZERO.to_jph();
                JPH_Body_GetAccumulatedForce(body.as_ptr(), &mut accumulated_force);
                JPH_Body_GetAccumulatedTorque(body.as_ptr(), &mut accumulated_torque);
                JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
                JPH_Body_GetCenterOfMassPosition(body.as_ptr(), &mut center_of_mass);
                Some(LoadState {
                    force: Vec3::from_jph(accumulated_force),
                    torque: Vec3::from_jph(accumulated_torque),
                    inverse_mass: JPH_MotionProperties_GetInverseMassUnchecked(motion),
                    inverse_inertia: Vec3::from_jph(inverse_inertia),
                    center_of_mass: RVec3::from_jph(center_of_mass),
                })
            }
        })
        .ok_or(BodyError::NotFound(self.id))?;
        // Jolt ignores loads on static and kinematic bodies.
        let Some(state) = state else {
            return Ok(());
        };
        let point_torque = match point {
            Some(point) => point_torque(force, point, state.center_of_mass)?,
            None => [0.0; 3],
        };
        let sum = |a: Vec3, b: Vec3, c: [f64; 3]| {
            let [ax, ay, az] = [a.x, a.y, a.z].map(f64::from);
            let [bx, by, bz] = [b.x, b.y, b.z].map(f64::from);
            [ax + bx + c[0], ay + by + c[1], az + bz + c[2]]
        };
        let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let new_force = sum(state.force, force, [0.0; 3]);
        let new_torque = sum(state.torque, torque, point_torque);
        let largest_inverse_inertia = state
            .inverse_inertia
            .x
            .max(state.inverse_inertia.y)
            .max(state.inverse_inertia.z);
        require(
            length(new_force) * f64::from(state.inverse_mass)
                <= f64::from(limits::MAX_ACCELERATION),
            "accumulated force would exceed limits::MAX_ACCELERATION for this body",
        )?;
        require(
            length(new_torque) * f64::from(largest_inverse_inertia)
                <= f64::from(limits::MAX_ANGULAR_ACCELERATION),
            "accumulated torque would exceed limits::MAX_ANGULAR_ACCELERATION for this body",
        )
    }

    /// Discards the force and torque added since the last step. Jolt clears them after every
    /// step anyway. Does nothing for static and kinematic bodies.
    pub fn reset_forces(&mut self) {
        with_locked_body(self.inner.body_lock_interface, self.id, |body| {
            // SAFETY: `body` is locked for writing for the duration of the closure. Jolt's
            // `ResetForce` and `ResetTorque` need motion properties, which only dynamic bodies
            // are guaranteed to have here, hence the `IsDynamic` check first.
            unsafe {
                if JPH_Body_IsDynamic(body.as_ptr()) {
                    JPH_Body_ResetForce(body.as_ptr());
                    JPH_Body_ResetTorque(body.as_ptr());
                }
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::ensure_initialized;
    use crate::Real;

    #[test]
    fn default_settings_match_jolt() {
        assert!(ensure_initialized());
        // SAFETY: Jolt is initialised, and the handle takes over the new settings.
        let jolt = CreationSettings(
            unsafe { Owned::from_raw(JPH_BodyCreationSettings_Create()) }.unwrap(),
        );
        let ours = BodySettings::default();
        let ptr = jolt.0.as_ptr();
        // SAFETY: `ptr` is the live settings object owned by `jolt`; getters only read it.
        unsafe {
            assert_eq!(JPH_BodyCreationSettings_GetFriction(ptr), ours.friction);
            assert_eq!(
                JPH_BodyCreationSettings_GetRestitution(ptr),
                ours.restitution
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetGravityFactor(ptr),
                ours.gravity_factor
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetAllowSleeping(ptr),
                ours.allow_sleeping
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetMotionQuality(ptr),
                ours.motion_quality.to_jph()
            );
            assert_eq!(
                MotionType::from_jph(JPH_BodyCreationSettings_GetMotionType(ptr)),
                ours.motion_type
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetOverrideMassProperties(ptr),
                JPH_OverrideMassProperties_CalculateMassAndInertia
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetEnhancedInternalEdgeRemoval(ptr),
                ours.enhanced_internal_edge_removal
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetLinearDamping(ptr),
                ours.linear_damping
            );
            assert_eq!(
                JPH_BodyCreationSettings_GetAngularDamping(ptr),
                ours.angular_damping
            );
            assert!(ours.mass.is_none());
            // The one documented difference: Jolt's default layer is 0.
            assert_eq!(JPH_BodyCreationSettings_GetObjectLayer(ptr), 0);
        }
        assert_eq!(ours.object_layer, ObjectLayer::MOVING);
    }

    #[test]
    fn velocity_limits_are_jolts_default_maxima() {
        assert!(ensure_initialized());
        let shape = Shape::new_sphere(0.5).unwrap();
        let creation = CreationSettings::new(&shape, &BodySettings::default()).unwrap();
        // SAFETY: the settings are live and owned by `creation`; the getters only read them.
        let (linear, angular) = unsafe {
            (
                JPH_BodyCreationSettings_GetMaxLinearVelocity(creation.as_ptr()),
                JPH_BodyCreationSettings_GetMaxAngularVelocity(creation.as_ptr()),
            )
        };
        assert_eq!(linear.to_bits(), limits::MAX_LINEAR_VELOCITY.to_bits());
        assert_eq!(angular.to_bits(), limits::MAX_ANGULAR_VELOCITY.to_bits());
    }

    #[test]
    fn point_torque_rejects_overflowing_products_even_when_they_cancel() {
        let center = RVec3::ZERO;
        let torque = point_torque(Vec3::new(0.0, 2.0, 0.0), RVec3::new(3.0, 0.0, 0.0), center);
        assert_eq!(torque, Ok([0.0, 0.0, 6.0]));
        // Lever and force are parallel: the cross product is zero in f64, but Jolt's f32
        // products `lever_x * force_y` and `lever_y * force_x` are infinite.
        let parallel = point_torque(
            Vec3::new(1.0e20, 1.0e20, 0.0),
            RVec3::new(1.0e20, 1.0e20, 0.0),
            center,
        );
        assert!(matches!(parallel, Err(BodyError::InvalidValue(_))));
        let far = RVec3::new(Real::MAX, 0.0, 0.0);
        let lever_overflows = point_torque(
            Vec3::new(0.0, 1.0, 0.0),
            far,
            RVec3::new(-Real::MAX, 0.0, 0.0),
        );
        assert!(matches!(lever_overflows, Err(BodyError::InvalidValue(_))));
    }

    #[test]
    fn enhanced_internal_edge_removal_reaches_the_body() {
        let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
        let shape = Shape::new_sphere(0.5).unwrap();
        for value in [true, false] {
            let id = world
                .create_body(
                    &shape,
                    &BodySettings::new_dynamic().enhanced_internal_edge_removal(value),
                )
                .unwrap();
            let stored = with_locked_body(world.body_lock_interface, id, |body| {
                // SAFETY: `body` is locked for the duration of the closure; the getter only
                // reads it.
                unsafe { JPH_Body_GetEnhancedInternalEdgeRemoval(body.as_ptr()) }
            });
            assert_eq!(stored, Some(value));
        }
    }

    #[test]
    fn damping_reaches_the_body() {
        let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
        let shape = Shape::new_sphere(0.5).unwrap();
        let settings = BodySettings::new_dynamic()
            .linear_damping(0.3)
            .angular_damping(0.7);
        let id = world.create_body(&shape, &settings).unwrap();
        let stored = with_read_locked_body(world.body_lock_interface, id, |body| {
            // SAFETY: `body` is locked for the duration of the closure and dynamic, so it has
            // motion properties; the getters only read.
            unsafe {
                let motion = JPH_Body_GetMotionProperties(body.as_ptr());
                (
                    JPH_MotionProperties_GetLinearDamping(motion),
                    JPH_MotionProperties_GetAngularDamping(motion),
                )
            }
        });
        assert_eq!(stored, Some((0.3, 0.7)));
    }

    #[test]
    fn invalid_damping_is_rejected() {
        let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
        let shape = Shape::new_sphere(0.5).unwrap();
        for value in [-0.1, f32::NAN, f32::INFINITY] {
            for settings in [
                BodySettings::new_dynamic().linear_damping(value),
                BodySettings::new_dynamic().angular_damping(value),
            ] {
                assert!(matches!(
                    world.create_body(&shape, &settings),
                    Err(BodyError::InvalidValue(_))
                ));
            }
        }
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn mass_is_reported_for_dynamic_bodies_only() {
        let mut world = PhysicsWorld::new(crate::WorldSettings::default()).unwrap();
        let shape = Shape::new_sphere(0.5).unwrap();
        let dynamic = world
            .create_body(&shape, &BodySettings::new_dynamic().mass(12.5))
            .unwrap();
        let mass = world.body(dynamic).unwrap().mass().unwrap();
        assert!((mass - 12.5).abs() <= 12.5 * 1.0e-6, "{mass}");
        for settings in [BodySettings::new_static(), BodySettings::new_kinematic()] {
            let id = world.create_body(&shape, &settings).unwrap();
            assert_eq!(world.body(id).unwrap().mass(), None);
        }
    }

    /// Mass 1 with the inertia `R * diag(moments) * R^T`, `R` a rotation of 30 degrees about Z.
    fn rotated_inertia(moments: [f32; 3]) -> JPH_MassProperties {
        let (sin, cos) = 30.0_f32.to_radians().sin_cos();
        let rotation = [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]];
        let mut properties = JPH_MassProperties {
            mass: 1.0,
            ..ZERO_MASS_PROPERTIES
        };
        for (column, out) in properties.inertia.column.iter_mut().take(3).enumerate() {
            let entry = |row: usize| -> f32 {
                (0..3)
                    .map(|k| rotation[row][k] * moments[k] * rotation[column][k])
                    .sum()
            };
            *out = JPH_Vec4 {
                x: entry(0),
                y: entry(1),
                z: entry(2),
                w: 0.0,
            };
        }
        properties
    }

    #[test]
    fn rotated_inertia_is_accepted() {
        let properties = rotated_inertia([1.0, 2.0, 3.0]);
        assert_ne!(
            properties.inertia.column[0].y, 0.0,
            "the tensor is not diagonal"
        );
        assert!(has_finite_inverse(&properties));
    }

    #[test]
    fn non_finite_inertia_is_rejected() {
        let mut properties = rotated_inertia([1.0, 2.0, 3.0]);
        properties.inertia.column[1].x = f32::NAN;
        assert!(!has_finite_inverse(&properties));
    }

    #[test]
    fn ill_conditioned_rotated_inertia_is_rejected() {
        assert!(!has_finite_inverse(&rotated_inertia([1.0, 1.0e-9, 1.0])));
    }

    fn aabox(min: [f32; 3], max: [f32; 3]) -> JPH_AABox {
        JPH_AABox {
            min: Vec3::from(min).to_jph(),
            max: Vec3::from(max).to_jph(),
        }
    }

    #[test]
    fn bounds_overlap_counts_touching_and_containment() {
        let unit = aabox([0.0; 3], [1.0; 3]);
        assert!(bounds_overlap(&unit, &aabox([0.5; 3], [2.0; 3])));
        assert!(bounds_overlap(
            &unit,
            &aabox([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])
        ));
        assert!(bounds_overlap(&unit, &aabox([0.25; 3], [0.75; 3])));
        assert!(bounds_overlap(&aabox([0.25; 3], [0.75; 3]), &unit));
    }

    #[test]
    fn bounds_separated_on_any_axis_do_not_overlap() {
        let unit = aabox([0.0; 3], [1.0; 3]);
        for axis in 0..3 {
            let mut min = [0.0; 3];
            let mut max = [1.0; 3];
            min[axis] = 1.5;
            max[axis] = 2.5;
            let apart = aabox(min, max);
            assert!(!bounds_overlap(&unit, &apart), "axis {axis}");
            assert!(!bounds_overlap(&apart, &unit), "axis {axis}");
        }
    }

    #[test]
    fn tiny_rotated_inertia_uses_the_unit_sphere() {
        assert!(has_finite_inverse(&rotated_inertia([
            1.0e-7, 2.0e-7, 3.0e-7
        ])));
    }
}
