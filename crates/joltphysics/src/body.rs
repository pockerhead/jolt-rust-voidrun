//! Rigid bodies: ids, creation settings, and read and write access.

use std::fmt;
use std::marker::PhantomData;
use std::ops::Deref;
use std::ptr::NonNull;

use joltphysics_sys::*;

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
    world: WorldTag,
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
    fn to_jph(self) -> JPH_MotionType {
        match self {
            Self::Static => JPH_MotionType_Static,
            Self::Kinematic => JPH_MotionType_Kinematic,
            Self::Dynamic => JPH_MotionType_Dynamic,
        }
    }

    fn from_jph(value: JPH_MotionType) -> Self {
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
    fn to_jph(self) -> JPH_Activation {
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
    motion_type: MotionType,
    object_layer: ObjectLayer,
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    friction: f32,
    restitution: f32,
    mass: Option<f32>,
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

    /// Initial position of the body origin in metres. Default the origin.
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

    /// Initial linear velocity in m/s. Default zero.
    #[must_use]
    pub fn linear_velocity(mut self, value: Vec3) -> Self {
        self.linear_velocity = value;
        self
    }

    /// Initial angular velocity in rad/s. Default zero.
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

    /// Friction coefficient, finite and at least 0. Default 0.2.
    #[must_use]
    pub fn friction(mut self, value: f32) -> Self {
        self.friction = value;
        self
    }

    /// Restitution (bounciness), finite and at least 0; usually at most 1. Default 0.
    #[must_use]
    pub fn restitution(mut self, value: f32) -> Self {
        self.restitution = value;
        self
    }

    /// Overrides the mass in kg (finite, positive and large enough that Jolt can invert it and
    /// the scaled inertia; [`PhysicsWorld::create_body`] checks this). The inertia is computed
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

    /// Multiplier for the world's gravity on this body, finite. Default 1.
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

    /// Whether Jolt removes ghost contacts of this body against internal edges of triangle
    /// shapes (heightfields, meshes), Jolt's enhanced internal edge removal. Costs extra CPU per
    /// contact. Default false.
    #[must_use]
    pub fn enhanced_internal_edge_removal(mut self, value: bool) -> Self {
        self.enhanced_internal_edge_removal = value;
        self
    }

    fn validate(&self, object_layer_count: u32) -> Result<(), BodyError> {
        if self.object_layer.get() >= object_layer_count {
            return Err(BodyError::UnknownObjectLayer(self.object_layer));
        }
        let invalid = |what| Err(BodyError::InvalidValue(what));
        if !self.position.is_finite() {
            return invalid("position must be finite");
        }
        if !(self.rotation.is_finite() && self.rotation.is_normalized()) {
            return invalid("rotation must be a finite unit quaternion");
        }
        if !self.linear_velocity.is_finite() {
            return invalid("linear velocity must be finite");
        }
        if !self.angular_velocity.is_finite() {
            return invalid("angular velocity must be finite");
        }
        if !(self.friction.is_finite() && self.friction >= 0.0) {
            return invalid("friction must be finite and not negative");
        }
        if !(self.restitution.is_finite() && self.restitution >= 0.0) {
            return invalid("restitution must be finite and not negative");
        }
        if !self.gravity_factor.is_finite() {
            return invalid("gravity factor must be finite");
        }
        if let Some(mass) = self.mass {
            if !(mass.is_finite() && mass > 0.0) {
                return invalid("mass must be finite and positive");
            }
        }
        Ok(())
    }
}

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
fn mass_properties(shape: &Shape, mass: Option<f32>) -> JPH_MassProperties {
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
fn has_finite_inverse(properties: &JPH_MassProperties) -> bool {
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
struct CreationSettings(Owned<JPH_BodyCreationSettings>);

/// Body creation settings, owned whole by their owner.
impl JoltObject for JPH_BodyCreationSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner owns the settings (trait contract), which joltc deletes; bodies
        // created from them keep their own shape reference.
        unsafe { JPH_BodyCreationSettings_Destroy(ptr) };
    }
}

impl CreationSettings {
    fn new(shape: &Shape, settings: &BodySettings) -> Result<Self, BodyError> {
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
}

/// Jolt's `BodyID::cInvalidBodyID`, returned by `CreateAndAddBody` when the world is full.
const INVALID_BODY_ID: JPH_BodyID = 0xffff_ffff;

impl PhysicsWorld {
    /// Creates a body from `shape` and adds it to the world. The body keeps its own reference
    /// to the shape, so `shape` may be dropped afterwards.
    ///
    /// Fails with [`BodyError::InvalidValue`] when a setting is out of range, when a dynamic or
    /// kinematic body uses a shape that only static bodies may use (a heightfield, or a
    /// compound that contains one), and when a dynamic or kinematic body's mass or inertia
    /// (overridden, or computed from a tiny or very slender shape) is too small for Jolt to
    /// invert.
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
        if settings.motion_type != MotionType::Static
            && !has_finite_inverse(&mass_properties(shape, settings.mass))
        {
            return Err(BodyError::InvalidValue(
                "mass and shape give an infinite inverse mass or inertia",
            ));
        }
        let creation = CreationSettings::new(shape, settings)?;
        // SAFETY: the body interface belongs to this live world, borrowed mutably; `creation`
        // is a fully set up settings object whose layer exists in this world.
        let raw = unsafe {
            JPH_BodyInterface_CreateAndAddBody(
                self.body_interface.as_ptr(),
                creation.0.as_ptr(),
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
                id,
                _world: PhantomData,
            },
            body_lock_interface: self.body_lock_interface,
            _world: PhantomData,
        })
    }

    /// Removes a body from the world and destroys it.
    ///
    /// Jolt does not wake the bodies around a removed one by itself, so joltphysics wakes every body
    /// whose bounds overlap the removed body's bounds; a stack whose bottom is removed falls.
    pub fn remove_body(&mut self, id: BodyId) -> Result<(), BodyError> {
        self.check(id)?;
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
        // (Jolt does not validate ids in `DestroyBody`). `bounds` is a live local and null
        // filters select Jolt's accept-all defaults.
        unsafe {
            JPH_BodyInterface_RemoveAndDestroyBody(self.body_interface.as_ptr(), id.raw);
            JPH_BodyInterface_ActivateBodiesInAABox(
                self.body_interface.as_ptr(),
                &bounds,
                std::ptr::null(),
                std::ptr::null(),
            );
        }
        Ok(())
    }
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
fn with_locked_body<R>(
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
}

/// Read and write access to one body, borrowed mutably from its world.
///
/// Dereferences to [`BodyRef`] for reads. Setters validate their input before it reaches
/// Jolt. Units: metres, m/s, rad/s, newtons and newton-metres. Static bodies ignore velocity
/// and force writes. Not `Send` or `Sync`.
pub struct BodyMut<'w> {
    inner: BodyRef<'w>,
    body_lock_interface: NonNull<JPH_BodyLockInterface>,
    _world: PhantomData<&'w mut PhysicsWorld>,
}

impl<'w> Deref for BodyMut<'w> {
    type Target = BodyRef<'w>;

    fn deref(&self) -> &BodyRef<'w> {
        &self.inner
    }
}

fn require(valid: bool, what: &'static str) -> Result<(), BodyError> {
    if valid {
        Ok(())
    } else {
        Err(BodyError::InvalidValue(what))
    }
}

impl BodyMut<'_> {
    /// Moves the body origin to `position`.
    pub fn set_position(
        &mut self,
        position: RVec3,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(position.is_finite(), "position must be finite")?;
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
            rotation.is_finite() && rotation.is_normalized(),
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

    /// Sets position and rotation together.
    pub fn set_position_and_rotation(
        &mut self,
        position: RVec3,
        rotation: Quat,
        activation: Activation,
    ) -> Result<(), BodyError> {
        require(position.is_finite(), "position must be finite")?;
        require(
            rotation.is_finite() && rotation.is_normalized(),
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

    /// Sets the linear velocity in m/s. Wakes the body when the velocity is not near zero.
    pub fn set_linear_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        require(velocity.is_finite(), "linear velocity must be finite")?;
        let velocity = velocity.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_SetLinearVelocity(self.interface(), self.id.raw, &velocity) };
        Ok(())
    }

    /// Sets the angular velocity in rad/s. Wakes the body when the velocity is not near zero.
    pub fn set_angular_velocity(&mut self, velocity: Vec3) -> Result<(), BodyError> {
        require(velocity.is_finite(), "angular velocity must be finite")?;
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
    pub fn add_force(&mut self, force: Vec3) -> Result<(), BodyError> {
        require(force.is_finite(), "force must be finite")?;
        let mut force = force.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddForce(self.interface(), self.id.raw, &mut force) };
        Ok(())
    }

    /// Adds a force in newtons applied at a world-space `point`, which also adds the matching
    /// torque, and wakes the body.
    pub fn add_force_at_point(&mut self, force: Vec3, point: RVec3) -> Result<(), BodyError> {
        require(force.is_finite(), "force must be finite")?;
        require(point.is_finite(), "point must be finite")?;
        let mut force = force.to_jph();
        let mut point = point.to_jph();
        // SAFETY: as in `set_position`.
        unsafe {
            JPH_BodyInterface_AddForce2(self.interface(), self.id.raw, &mut force, &mut point)
        };
        Ok(())
    }

    /// Adds a torque in newton-metres for the next step, and wakes the body.
    pub fn add_torque(&mut self, torque: Vec3) -> Result<(), BodyError> {
        require(torque.is_finite(), "torque must be finite")?;
        let mut torque = torque.to_jph();
        // SAFETY: as in `set_position`.
        unsafe { JPH_BodyInterface_AddTorque(self.interface(), self.id.raw, &mut torque) };
        Ok(())
    }

    /// Discards the force and torque added since the last step. Jolt clears them after every
    /// step anyway. Does nothing for static and kinematic bodies.
    pub fn reset_forces(&mut self) {
        with_locked_body(self.body_lock_interface, self.id, |body| {
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
            assert!(ours.mass.is_none());
            // The one documented difference: Jolt's default layer is 0.
            assert_eq!(JPH_BodyCreationSettings_GetObjectLayer(ptr), 0);
        }
        assert_eq!(ours.object_layer, ObjectLayer::MOVING);
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

    #[test]
    fn tiny_rotated_inertia_uses_the_unit_sphere() {
        assert!(has_finite_inverse(&rotated_inertia([
            1.0e-7, 2.0e-7, 3.0e-7
        ])));
    }
}
