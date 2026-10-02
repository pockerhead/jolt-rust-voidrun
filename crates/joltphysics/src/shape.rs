//! Collision shapes.

use std::fmt;
use std::ptr::{null, null_mut, NonNull};

use joltphysics_sys::*;

use crate::math::{is_finite_non_negative, is_finite_positive};
use crate::owned::{JoltObject, Owned};
use crate::world::ensure_initialized;
use crate::{Quat, ShapeError, Vec3};

/// A collision shape that bodies are created from.
///
/// Owns one Jolt reference. Every body created from it holds its own reference, so the shape
/// may be dropped while bodies use it. Jolt shapes cannot change after construction, so one
/// shape may serve any number of bodies in any number of worlds.
pub struct Shape(Owned<JPH_Shape>);

// SAFETY: Jolt shapes are immutable after construction and `RefTarget` counts references
// atomically, so a shape may be used and released from any thread
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#memory-management).
unsafe impl Send for Shape {}
// SAFETY: as for `Send`; `&Shape` only lets bodies take further references and read the
// immutable shape.
unsafe impl Sync for Shape {}

/// Jolt's `SubShapeID`: the path from a body's root shape to the leaf shape that was hit (a
/// compound child, a heightfield triangle).
///
/// It is meaningful only together with the root shape it came from. Equal shapes built the same
/// way give equal ids, so ids may be hashed and compared.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub struct SubShapeId(u32);

impl SubShapeId {
    pub(crate) fn new(raw: u32) -> Self {
        Self(raw)
    }

    /// The id as Jolt stores it (`SubShapeID::GetValue`).
    pub fn to_raw(self) -> u32 {
        self.0
    }
}

/// One child of a compound shape, for [`Shape::new_compound`].
///
/// The pose is relative to the compound's origin (Jolt `CompoundShapeSettings::AddShape`).
/// `user_data` is free for the caller, for example a collision group. It belongs to the
/// compound child, not to the leaf shape (Jolt `CompoundShape::SubShape::mUserData`), so one
/// leaf [`Shape`] may serve several children with different values.
#[derive(Clone, Copy)]
pub struct CompoundChild<'a> {
    /// The child's shape.
    pub shape: &'a Shape,
    /// Position of the child's origin in the compound, metres; finite.
    pub position: Vec3,
    /// Rotation of the child in the compound; a finite unit quaternion.
    pub rotation: Quat,
    /// The caller's value for this child, readable from hits.
    pub user_data: u32,
}

impl fmt::Debug for CompoundChild<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CompoundChild")
            .field("position", &self.position)
            .field("rotation", &self.rotation)
            .field("user_data", &self.user_data)
            .finish_non_exhaustive()
    }
}

/// The compound child a [`SubShapeId`] leads to, from [`Shape::compound_sub_shape`] or
/// [`PhysicsWorld::compound_sub_shape`](crate::PhysicsWorld::compound_sub_shape).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CompoundSubShape {
    /// Position of the child in the `children` slice given to [`Shape::new_compound`].
    pub index: u32,
    /// The child's [`CompoundChild::user_data`].
    pub user_data: u32,
}

/// Owns a `JPH_ShapeSettings`: the one reference every `JPH_*ShapeSettings_Create` returns
/// (joltc calls `AddRef` on the new settings).
///
/// Dropping it releases that reference, which deletes the settings and with them their
/// cached `ShapeResult` reference. A shape returned by a `*_CreateShape` or `*_Create` call
/// carries its own reference (joltc calls `AddRef` before returning it), so the caller keeps
/// exactly one reference to the shape.
struct ShapeSettings(Owned<JPH_ShapeSettings>);

/// Shape settings, of which the owner holds one Jolt reference.
impl JoltObject for JPH_ShapeSettings {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference to the settings (trait contract), released here
        // once. Shapes created from them hold their own, and another Jolt object may still
        // hold further ones, so joltc only calls `Release`, never `delete`.
        unsafe { JPH_ShapeSettings_Destroy(ptr) };
    }
}

impl ShapeSettings {
    /// Takes over settings returned by a `JPH_*ShapeSettings_Create` call.
    ///
    /// # Safety
    /// `ptr` is null or points to live shape settings holding one reference that the caller
    /// hands over.
    unsafe fn from_raw(ptr: *mut JPH_ShapeSettings) -> Result<Self, ShapeError> {
        // SAFETY: the caller hands over one reference to live settings, or null.
        unsafe { Owned::from_raw(ptr) }
            .map(Self)
            .ok_or(ShapeError::AllocationFailed)
    }

    /// The settings as one of joltc's typed settings pointers. joltc's settings types are
    /// `reinterpret_cast`s of Jolt classes with single inheritance from `ShapeSettings`, the
    /// convention joltc itself uses, so the caller picks the type the settings were created as.
    fn as_ptr<T>(&self) -> *mut T {
        self.0.as_ptr().cast()
    }
}

/// Jolt heightfield settings holding `samples` and `settings`.
///
/// # Safety
/// Jolt is initialised, `samples` holds `sample_count^2` finite values and `settings` passed
/// `validate_layout(sample_count)` and `validate_extents`.
unsafe fn height_field_settings(
    sample_count: u32,
    samples: &[f32],
    settings: &HeightFieldSettings,
) -> Result<ShapeSettings, ShapeError> {
    let offset = settings.offset.to_jph();
    let scale = settings.scale.to_jph();
    // SAFETY: the caller guarantees initialisation and that `samples` holds `sample_count^2`
    // floats, which Jolt copies; `offset` and `scale` are live locals. Null material indices
    // are allowed. The returned settings hold one reference, which the guard takes over.
    let jolt_settings = unsafe {
        ShapeSettings::from_raw(
            JPH_HeightFieldShapeSettings_Create(
                samples.as_ptr(),
                &offset,
                &scale,
                sample_count,
                null(),
            )
            .cast(),
        )
    }?;
    let ptr = jolt_settings.as_ptr();
    // SAFETY: the settings are live, owned by the guard and were created as heightfield
    // settings; the caller validated every value.
    unsafe {
        JPH_HeightFieldShapeSettings_SetBlockSize(ptr, settings.block_size);
        JPH_HeightFieldShapeSettings_SetBitsPerSample(ptr, settings.bits_per_sample);
        JPH_HeightFieldShapeSettings_SetActiveEdgeCosThresholdAngle(
            ptr,
            settings.active_edge_cos_threshold_angle,
        );
    }
    Ok(jolt_settings)
}

/// Runs `JPH_Init` once, mapping failure to [`ShapeError::InitFailed`].
fn initialize() -> Result<(), ShapeError> {
    if ensure_initialized() {
        Ok(())
    } else {
        Err(ShapeError::InitFailed)
    }
}

/// Settings of [`Shape::new_height_field`] other than the samples. The defaults are Jolt's
/// (`HeightFieldShapeSettings`).
#[derive(Clone, Debug, PartialEq)]
pub struct HeightFieldSettings {
    offset: Vec3,
    scale: Vec3,
    block_size: u32,
    bits_per_sample: u32,
    active_edge_cos_threshold_angle: f32,
}

impl Default for HeightFieldSettings {
    fn default() -> Self {
        Self {
            offset: Vec3::ZERO,
            scale: Vec3::new(1.0, 1.0, 1.0),
            block_size: 2,
            bits_per_sample: 8,
            active_edge_cos_threshold_angle: 0.996195,
        }
    }
}

impl HeightFieldSettings {
    /// Shape-local position of sample (0, 0) at height 0, metres; finite. Default zero.
    #[must_use]
    pub fn offset(mut self, value: Vec3) -> Self {
        self.offset = value;
        self
    }

    /// Metres per sample step along X and Z, and the factor applied to the samples along Y;
    /// each finite and positive. Default `(1, 1, 1)`.
    #[must_use]
    pub fn scale(mut self, value: Vec3) -> Self {
        self.scale = value;
        self
    }

    /// Side length of the square blocks Jolt groups samples into, in `2..=8`. Larger blocks use
    /// less memory and make queries slower. Default 2.
    #[must_use]
    pub fn block_size(mut self, value: u32) -> Self {
        self.block_size = value;
        self
    }

    /// Bits Jolt stores per sample, in `1..=16`, each relative to the height range of its
    /// block. More bits follow the samples more closely and use more memory. Default 8.
    #[must_use]
    pub fn bits_per_sample(mut self, value: u32) -> Self {
        self.bits_per_sample = value;
        self
    }

    /// Cosine of the angle between two triangles above which their shared edge counts as
    /// active, in `[0, 1]`; concave edges are never active. Smaller values give more ghost
    /// collisions with edges, larger ones slower depenetration (Jolt's wording). Default
    /// `0.996195`, the cosine of 5 degrees.
    #[must_use]
    pub fn active_edge_cos_threshold_angle(mut self, value: f32) -> Self {
        self.active_edge_cos_threshold_angle = value;
        self
    }

    /// Checks the settings Jolt relies on before it looks at the samples and returns the
    /// padded sample count; `sample_count` is at least 2.
    fn validate_layout(&self, sample_count: u32) -> Result<u64, ShapeError> {
        let invalid = |what| Err(ShapeError::InvalidSettings(what));
        if !(2..=8).contains(&self.block_size) {
            return invalid("block_size must be between 2 and 8");
        }
        if !(1..=16).contains(&self.bits_per_sample) {
            return invalid("bits_per_sample must be between 1 and 16");
        }
        let threshold = self.active_edge_cos_threshold_angle;
        if !(threshold.is_finite() && (0.0..=1.0).contains(&threshold)) {
            return invalid("active_edge_cos_threshold_angle must be between 0 and 1");
        }
        let padded = padded_sample_count(sample_count, self.block_size);
        if padded / u64::from(self.block_size) < 2 {
            return invalid("sample_count must be larger than block_size");
        }
        // Jolt needs `2 * bits(padded - 1) + 1` sub-shape id bits, at most 32.
        if padded > 32768 {
            return invalid("sample_count is too large");
        }
        Ok(padded)
    }

    /// Checks that offset, scale and the field's extents are finite, given the padded sample
    /// count and the range of the samples that are not holes.
    fn validate_extents(&self, padded: u64, heights: Option<(f32, f32)>) -> Result<(), ShapeError> {
        if !self.offset.is_finite() {
            return Err(ShapeError::InvalidDimensions(
                "height field offset must be finite",
            ));
        }
        let scale = [self.scale.x, self.scale.y, self.scale.z];
        if !scale.into_iter().all(is_finite_positive) {
            return Err(ShapeError::InvalidDimensions(
                "height field scale must be finite and positive",
            ));
        }
        let last = (padded - 1) as f32;
        let far_x = self.offset.x + self.scale.x * last;
        let far_z = self.offset.z + self.scale.z * last;
        if !(far_x.is_finite() && far_z.is_finite()) {
            return Err(ShapeError::InvalidDimensions(
                "height field extent along x or z must be finite",
            ));
        }
        if let Some((min, max)) = heights {
            let (offset, scale) = (self.offset.y, self.scale.y);
            if !((offset + scale * min).is_finite() && (offset + scale * max).is_finite()) {
                return Err(ShapeError::InvalidDimensions(
                    "height field extent along y must be finite",
                ));
            }
        }
        Ok(())
    }
}

/// `sample_count` rounded up to a multiple of `block_size`, as Jolt stores it.
fn padded_sample_count(sample_count: u32, block_size: u32) -> u64 {
    u64::from(sample_count).div_ceil(u64::from(block_size)) * u64::from(block_size)
}

/// Lowest and highest sample that is not a hole (`f32::MAX`, Jolt's `cNoCollisionValue`), or
/// `None` when every sample is a hole.
fn height_range(samples: &[f32]) -> Option<(f32, f32)> {
    samples
        .iter()
        .filter(|&&sample| sample != f32::MAX)
        .fold(None, |range, &sample| match range {
            None => Some((sample, sample)),
            Some((min, max)) => Some((sample.min(min), sample.max(max))),
        })
}

impl Shape {
    /// A box with the given half extents in metres (each finite and positive) and Jolt's
    /// default convex radius of 0.05 m; see [`new_box_with_convex_radius`].
    ///
    /// [`new_box_with_convex_radius`]: Self::new_box_with_convex_radius
    pub fn new_box(half_extent: Vec3) -> Result<Self, ShapeError> {
        Self::new_box_with_convex_radius(half_extent, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A box with the given half extents in metres (each finite and positive) and convex
    /// radius in metres (finite and not negative).
    ///
    /// Jolt shrinks the box by the convex radius and inflates it again, so the faces stay where
    /// they are while edges and corners are rounded for contacts and shape casts. Jolt clamps
    /// the radius to the smallest half extent (`BoxShape.h`), and contacts and shape casts use
    /// at most 0.05 m of it (`ScaleHelpers::ScaleConvexRadius` caps it at Jolt's default), so a
    /// larger radius collides like 0.05. A radius of 0 gives sharp edges; collision detection
    /// is then somewhat slower, because Jolt falls back to EPA more often. Ray casts always see
    /// the sharp box, whatever the radius (`BoxShape::CastRay` tests the half extents only).
    pub fn new_box_with_convex_radius(
        half_extent: Vec3,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        let components = [half_extent.x, half_extent.y, half_extent.z];
        if !components.into_iter().all(is_finite_positive) {
            return Err(ShapeError::InvalidDimensions(
                "box half extents must be finite and positive",
            ));
        }
        if !is_finite_non_negative(convex_radius) {
            return Err(ShapeError::InvalidDimensions(
                "convex radius must be finite and not negative",
            ));
        }
        initialize()?;
        let half_extent = half_extent.to_jph();
        // SAFETY: Jolt is initialised, `half_extent` is a live local and both inputs were
        // checked against Jolt's assertions. The returned box holds one reference, which
        // `Self` takes over.
        unsafe { Self::from_raw(JPH_BoxShape_Create(&half_extent, convex_radius).cast()) }
    }

    /// A sphere with the given radius in metres (finite and positive).
    pub fn new_sphere(radius: f32) -> Result<Self, ShapeError> {
        if !is_finite_positive(radius) {
            return Err(ShapeError::InvalidDimensions(
                "sphere radius must be finite and positive",
            ));
        }
        initialize()?;
        // SAFETY: Jolt is initialised. The returned sphere holds one reference, which `Self`
        // takes over.
        unsafe { Self::from_raw(JPH_SphereShape_Create(radius).cast()) }
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high,
    /// with Jolt's default convex radius of 0.05 m; see
    /// [`new_cylinder_with_convex_radius`].
    ///
    /// [`new_cylinder_with_convex_radius`]: Self::new_cylinder_with_convex_radius
    pub fn new_cylinder(half_height: f32, radius: f32) -> Result<Self, ShapeError> {
        Self::new_cylinder_with_convex_radius(half_height, radius, JPH_DEFAULT_CONVEX_RADIUS as f32)
    }

    /// A cylinder along the local Y axis, centred on the origin, `2 * half_height` metres high.
    ///
    /// Half height and radius must be finite and positive, the convex radius finite and not
    /// negative. Like a box's, the convex radius rounds the edges for contacts and shape casts;
    /// Jolt clamps it to `min(half_height, radius)` (`CylinderShape.cpp`) and uses at most
    /// 0.05 m of it there (`ScaleHelpers::ScaleConvexRadius`).
    pub fn new_cylinder_with_convex_radius(
        half_height: f32,
        radius: f32,
        convex_radius: f32,
    ) -> Result<Self, ShapeError> {
        if !(is_finite_positive(half_height) && is_finite_positive(radius)) {
            return Err(ShapeError::InvalidDimensions(
                "cylinder half height and radius must be finite and positive",
            ));
        }
        if !is_finite_non_negative(convex_radius) {
            return Err(ShapeError::InvalidDimensions(
                "convex radius must be finite and not negative",
            ));
        }
        initialize()?;
        // `JPH_CylinderShape_Create` would ignore the convex radius (joltc passes 0), so the
        // cylinder is built through its settings.
        // SAFETY: Jolt is initialised; the returned settings hold one reference, which the
        // guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(
                JPH_CylinderShapeSettings_Create(half_height, radius, convex_radius).cast(),
            )
        }?;
        // SAFETY: the settings are live, owned by the guard and were created as cylinder
        // settings. The returned shape holds one reference, which `Self` takes over.
        unsafe {
            Self::from_created(JPH_CylinderShapeSettings_CreateShape(settings.as_ptr()).cast())
        }
    }

    /// A capsule along the local Y axis, centred on the origin: a cylinder
    /// `2 * half_height_of_cylinder` metres high with a hemisphere of `radius` at each end, so
    /// `2 * (half_height_of_cylinder + radius)` metres high in total. Both values must be finite
    /// and positive.
    pub fn new_capsule(half_height_of_cylinder: f32, radius: f32) -> Result<Self, ShapeError> {
        if !(is_finite_positive(half_height_of_cylinder) && is_finite_positive(radius)) {
            return Err(ShapeError::InvalidDimensions(
                "capsule half height and radius must be finite and positive",
            ));
        }
        initialize()?;
        // SAFETY: Jolt is initialised and both values are positive, as Jolt asserts
        // (`CapsuleShape.h`). The returned capsule holds one reference, which `Self` takes
        // over.
        unsafe { Self::from_raw(JPH_CapsuleShape_Create(half_height_of_cylinder, radius).cast()) }
    }

    /// A heightfield of `sample_count` x `sample_count` height samples, in metres before
    /// scaling.
    ///
    /// # Layout
    /// The surface passes through `offset + scale * (x, samples[y * n + x], y)` for `x, y` in
    /// `0..n`, `n = sample_count`. `y` runs along +Z, so row `y` of `samples` holds the heights
    /// at `z = offset.z + y * scale.z` (Jolt's row-major order). A column-major source
    /// `heights[x * n + z]` must be transposed first:
    ///
    /// ```text
    /// samples[z * n + x] = heights[x * n + z]
    /// ```
    ///
    /// Each cell is split into two triangles along the diagonal from sample `(x, y)` to sample
    /// `(x + 1, y + 1)`. A sample of `f32::MAX` is a hole: the cells touching it have no
    /// collision. Every other sample must be finite.
    ///
    /// # Block size
    /// Jolt rounds `n` up to a multiple of the block size and fills the extra rows and columns
    /// with holes, so the cells touching them have no collision. With the default block size 2,
    /// n = 33 is stored as 34 x 34 and the surface covers exactly the 32 x 32 cells given.
    /// `n / block_size`, rounded up, must be at least 2.
    ///
    /// # Precision
    /// Jolt first quantises the heights to 16 bits over the height range of the whole field,
    /// then to [`HeightFieldSettings::bits_per_sample`] bits within the height range of each
    /// block. [`height_field_position`](Self::height_field_position) reads back the stored
    /// heights.
    ///
    /// # Static only
    /// [`PhysicsWorld::create_body`](crate::PhysicsWorld::create_body) refuses heightfields for
    /// dynamic and kinematic bodies.
    pub fn new_height_field(
        sample_count: u32,
        samples: &[f32],
        settings: &HeightFieldSettings,
    ) -> Result<Self, ShapeError> {
        if sample_count < 2 {
            return Err(ShapeError::InvalidDimensions(
                "sample_count must be at least 2",
            ));
        }
        // Jolt divides by the block size before it checks it, so the settings are checked
        // first.
        let padded = settings.validate_layout(sample_count)?;
        let count = sample_count as usize;
        if count.checked_mul(count) != Some(samples.len()) {
            return Err(ShapeError::InvalidDimensions(
                "samples must hold sample_count^2 values",
            ));
        }
        if !samples.iter().all(|sample| sample.is_finite()) {
            return Err(ShapeError::InvalidDimensions(
                "height samples must be finite",
            ));
        }
        let heights = height_range(samples);
        // Jolt quantises with `65534 / (max - min)`, so the range must be finite too.
        if heights.is_some_and(|(min, max)| !(max - min).is_finite()) {
            return Err(ShapeError::InvalidDimensions(
                "height sample range must be finite",
            ));
        }
        settings.validate_extents(padded, heights)?;
        initialize()?;
        // SAFETY: Jolt is initialised and `samples` and `settings` were validated above.
        let jolt_settings = unsafe { height_field_settings(sample_count, samples, settings) }?;
        // SAFETY: the settings are live, owned by the guard and were created as heightfield
        // settings. The returned shape holds one reference, which `Self` takes over.
        unsafe {
            Self::from_created(
                JPH_HeightFieldShapeSettings_CreateShape(jolt_settings.as_ptr()).cast(),
            )
        }
    }

    /// The stored surface point of heightfield sample `(x, y)` in shape-local space, after
    /// Jolt's quantisation. `None` when the shape is not a heightfield, when `(x, y)` lies
    /// outside the stored (padded) grid, or when the sample is a hole, including the padding
    /// Jolt adds.
    pub fn height_field_position(&self, x: u32, y: u32) -> Option<Vec3> {
        if self.sub_type() != JPH_ShapeSubType_HeightField {
            return None;
        }
        let shape: *const JPH_HeightFieldShape = self.as_ptr().cast();
        // SAFETY: the shape is live and a heightfield (checked above); the getter only reads it.
        let sample_count = unsafe { JPH_HeightFieldShape_GetSampleCount(shape) };
        if x >= sample_count || y >= sample_count {
            return None;
        }
        // SAFETY: as above, and `x` and `y` are inside the stored grid, as Jolt asserts.
        if unsafe { JPH_HeightFieldShape_IsNoCollision(shape, x, y) } {
            return None;
        }
        let mut position = Vec3::ZERO.to_jph();
        // SAFETY: as above; `position` is a live local.
        unsafe { JPH_HeightFieldShape_GetPosition(shape, x, y, &mut position) };
        Some(Vec3::from_jph(position))
    }

    /// A compound of `children`, each with its own pose and user data.
    ///
    /// The children may be dropped afterwards: the compound holds its own references. Child
    /// order is part of the shape and so of a deterministic state. Jolt moves the compound's
    /// centre of mass to the children's mass-weighted centre, so body positions of compounds
    /// read back through arithmetic.
    ///
    /// Two or more children make a Jolt `StaticCompoundShape`. A single child makes a
    /// `MutableCompoundShape`, because Jolt's static compound replaces a lone child by the
    /// child itself (or a `RotatedTranslatedShape`) and drops its user data
    /// (`StaticCompoundShape.cpp`); joltphysics never changes it after construction.
    pub fn new_compound(children: &[CompoundChild<'_>]) -> Result<Self, ShapeError> {
        let invalid = |what| Err(ShapeError::InvalidSettings(what));
        if children.is_empty() {
            return invalid("a compound needs at least one child");
        }
        if u32::try_from(children.len()).is_err() {
            return invalid("a compound has at most u32::MAX children");
        }
        for child in children {
            if !child.position.is_finite() {
                return invalid("compound child position must be finite");
            }
            if !child.rotation.is_valid_rotation() {
                return invalid("compound child rotation must be a finite unit quaternion");
            }
        }
        initialize()?;
        let single = children.len() == 1;
        // SAFETY: Jolt is initialised. The returned settings hold one reference, which the
        // guard takes over.
        let settings = unsafe {
            ShapeSettings::from_raw(if single {
                JPH_MutableCompoundShapeSettings_Create().cast()
            } else {
                JPH_StaticCompoundShapeSettings_Create().cast()
            })
        }?;
        for child in children {
            let position = child.position.to_jph();
            let rotation = child.rotation.to_jph();
            // SAFETY: the settings are live and owned by the guard; both compound settings
            // types derive from `CompoundShapeSettings` with single inheritance. The child shape
            // is live, and the settings store their own `RefConst` to it. `position` and
            // `rotation` are live locals.
            unsafe {
                JPH_CompoundShapeSettings_AddShape2(
                    settings.as_ptr(),
                    &position,
                    &rotation,
                    child.shape.as_ptr(),
                    child.user_data,
                );
            }
        }
        // SAFETY: the settings are live, owned by the guard and of the type each call expects.
        // Both calls run Jolt's `Create` and return a shape holding one reference, which `Self`
        // takes over; null means Jolt refused the settings (for example a hierarchy that needs
        // more than 32 sub-shape id bits).
        unsafe {
            Self::from_created(if single {
                JPH_MutableCompoundShape_Create(settings.as_ptr()).cast()
            } else {
                JPH_StaticCompoundShape_Create(settings.as_ptr()).cast()
            })
        }
    }

    /// The child of this compound that `id` leads to.
    ///
    /// `None` for shapes that are not compounds. Only the root level is decoded, so `id` may be
    /// a hit's full path or a partial path below this shape. An id that came from another
    /// shape gives `None` or an arbitrary valid child: the bits cannot prove where they came
    /// from.
    pub fn compound_sub_shape(&self, id: SubShapeId) -> Option<CompoundSubShape> {
        // SAFETY: `self` keeps the shape alive for the call.
        unsafe { compound_sub_shape_of(self.0.as_non_null(), id) }
    }

    /// Takes over a shape returned by a shape `Create` call, where null means joltc could not
    /// create it.
    ///
    /// # Safety
    /// `ptr` is null or a live shape holding one reference that the caller hands over.
    unsafe fn from_raw(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        // SAFETY: the caller hands over one reference to a live shape, or null.
        unsafe { Owned::from_raw(ptr) }
            .map(Self)
            .ok_or(ShapeError::AllocationFailed)
    }

    /// Takes over a shape returned by a settings `Create` call, where null means Jolt refused
    /// the settings.
    ///
    /// # Safety
    /// `ptr` is null or a live shape holding one reference that the caller hands over.
    unsafe fn from_created(ptr: *mut JPH_Shape) -> Result<Self, ShapeError> {
        // SAFETY: the caller hands over one reference to a live shape, or null.
        unsafe { Owned::from_raw(ptr) }
            .map(Self)
            .ok_or(ShapeError::Rejected)
    }

    pub(crate) fn as_ptr(&self) -> *const JPH_Shape {
        self.0.as_ptr()
    }

    /// Jolt's concrete shape type.
    pub(crate) fn sub_type(&self) -> JPH_ShapeSubType {
        // SAFETY: the shape is live for the call; the getter only reads it.
        unsafe { JPH_Shape_GetSubType(self.as_ptr()) }
    }

    /// Whether Jolt allows this shape only on static bodies (heightfields and compounds that
    /// contain one).
    pub(crate) fn must_be_static(&self) -> bool {
        // SAFETY: the shape is live for the call; the getter only reads it.
        unsafe { JPH_Shape_MustBeStatic(self.as_ptr()) }
    }

    /// The centre of mass in shape-local space, metres.
    pub(crate) fn center_of_mass(&self) -> Vec3 {
        let mut center = Vec3::ZERO.to_jph();
        // SAFETY: the shape is live for the call; the getter only reads it, and `center` is a
        // live local.
        unsafe { JPH_Shape_GetCenterOfMass(self.as_ptr(), &mut center) };
        Vec3::from_jph(center)
    }

    /// This sphere or capsule with its radius grown by `by` metres; `Ok(None)` for every other
    /// shape type.
    ///
    /// For spheres and capsules Jolt's default support function is the core (a point or a
    /// segment) plus a convex radius equal to the radius (`SphereShape.cpp`, `CapsuleShape.cpp`
    /// `GetSupportFunction`), and a shape cast adds `ShapeCastSettings::mExtraConvexRadius` to
    /// that same radius (`ConvexShape.cpp`, `CastSphereVsTriangles.cpp`,
    /// `CastConvexVsTriangles.cpp`). Casting the grown shape is therefore the cast Jolt does
    /// with an extra convex radius of `by`.
    pub(crate) fn inflated(&self, by: f32) -> Result<Option<Shape>, ShapeError> {
        let sub_type = self.sub_type();
        if sub_type == JPH_ShapeSubType_Sphere {
            // SAFETY: the shape is live and a sphere (checked above); the getter only reads it.
            let radius = unsafe { JPH_SphereShape_GetRadius(self.as_ptr().cast()) };
            Shape::new_sphere(radius + by).map(Some)
        } else if sub_type == JPH_ShapeSubType_Capsule {
            let capsule: *const JPH_CapsuleShape = self.as_ptr().cast();
            // SAFETY: the shape is live and a capsule (checked above); the getters only read it.
            let (half_height, radius) = unsafe {
                (
                    JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule),
                    JPH_CapsuleShape_GetRadius(capsule),
                )
            };
            Shape::new_capsule(half_height, radius + by).map(Some)
        } else {
            Ok(None)
        }
    }
}

/// The child of the compound `root` that `id` leads to; `None` unless `root` is a compound and
/// `id` names one of its children.
///
/// # Safety
/// `root` points to a live shape for the duration of the call.
pub(crate) unsafe fn compound_sub_shape_of(
    root: NonNull<JPH_Shape>,
    id: SubShapeId,
) -> Option<CompoundSubShape> {
    let root = root.as_ptr();
    // SAFETY: `root` is live (caller contract); the getter only reads it.
    let sub_type = unsafe { JPH_Shape_GetSubType(root) };
    if sub_type != JPH_ShapeSubType_StaticCompound && sub_type != JPH_ShapeSubType_MutableCompound {
        return None;
    }
    let compound: *const JPH_CompoundShape = root.cast();
    // SAFETY: `root` is live and a compound (checked above); the getter only reads it.
    let count = unsafe { JPH_CompoundShape_GetNumSubShapes(compound) };
    if count == 0 {
        return None;
    }
    // Jolt `CompoundShape::GetSubShapeIDBits`: enough bits for the indices `0..count`.
    let bits = 32 - (count - 1).leading_zeros();
    let mask = ((1_u64 << bits) - 1) as u32;
    // Rejecting out-of-range indices here keeps Jolt's index assertion unreachable.
    if id.to_raw() & mask >= count {
        return None;
    }
    let mut remainder: JPH_SubShapeID = 0;
    // SAFETY: as above; `remainder` is a live local and the index is in range.
    let index =
        unsafe { JPH_CompoundShape_GetSubShapeIndexFromID(compound, id.to_raw(), &mut remainder) };
    // Jolt indexes its child array without a check in `GetSubShape`.
    if index >= count {
        return None;
    }
    let mut user_data = 0;
    // SAFETY: as above, `index < count`, and joltc writes only the outputs that are not null.
    unsafe {
        JPH_CompoundShape_GetSubShape(
            compound,
            index,
            null_mut(),
            null_mut(),
            null_mut(),
            &mut user_data,
        );
    }
    Some(CompoundSubShape { index, user_data })
}

/// A shape, of which the owner holds one Jolt reference: joltc returns created shapes holding
/// one reference, and `JPH_Shape_Destroy` releases it. Bodies, body creation settings and
/// compounds hold their own.
impl JoltObject for JPH_Shape {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference to the shape (trait contract), released here
        // once. Bodies, creation settings and compounds keep their own.
        unsafe { JPH_Shape_Destroy(ptr) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn many_shapes_are_created_and_released() {
        for i in 0..1000 {
            let size = 0.1 + i as f32 * 0.001;
            drop(Shape::new_box(Vec3::new(size, size, size)).unwrap());
            drop(Shape::new_sphere(size).unwrap());
            drop(Shape::new_cylinder(size, size).unwrap());
            drop(Shape::new_capsule(size, size).unwrap());
        }
    }

    #[test]
    fn invalid_dimensions_are_rejected() {
        let invalid = |result: Result<Shape, ShapeError>| {
            matches!(result, Err(ShapeError::InvalidDimensions(_)))
        };
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            assert!(invalid(Shape::new_box(Vec3::new(1.0, bad, 1.0))));
            assert!(invalid(Shape::new_sphere(bad)));
            assert!(invalid(Shape::new_cylinder(bad, 1.0)));
            assert!(invalid(Shape::new_cylinder(1.0, bad)));
            assert!(invalid(Shape::new_capsule(bad, 1.0)));
            assert!(invalid(Shape::new_capsule(1.0, bad)));
        }
        for bad in [-0.1, f32::NAN, f32::INFINITY] {
            assert!(invalid(Shape::new_box_with_convex_radius(
                Vec3::new(1.0, 1.0, 1.0),
                bad
            )));
            assert!(invalid(Shape::new_cylinder_with_convex_radius(
                1.0, 1.0, bad
            )));
        }
        assert!(Shape::new_box_with_convex_radius(Vec3::new(1.0, 1.0, 1.0), 0.0).is_ok());
        assert!(Shape::new_cylinder_with_convex_radius(1.0, 1.0, 0.0).is_ok());
    }

    fn box_convex_radius(shape: &Shape) -> f32 {
        assert_eq!(shape.sub_type(), JPH_ShapeSubType_Box);
        // SAFETY: the shape is live and a box (checked above); the getter only reads it.
        unsafe { JPH_BoxShape_GetConvexRadius(shape.as_ptr().cast()) }
    }

    #[test]
    fn box_convex_radius_reaches_jolt() {
        let unit = Vec3::new(1.0, 1.0, 1.0);
        let sharp = Shape::new_box_with_convex_radius(unit, 0.0).unwrap();
        assert_eq!(box_convex_radius(&sharp), 0.0);
        let default = Shape::new_box(unit).unwrap();
        assert_eq!(box_convex_radius(&default), 0.05);
        let clamped = Shape::new_box_with_convex_radius(Vec3::new(0.1, 1.0, 1.0), 0.5).unwrap();
        assert_eq!(box_convex_radius(&clamped), 0.1);
    }

    #[test]
    fn cylinder_and_capsule_dimensions_reach_jolt() {
        let cylinder = Shape::new_cylinder(0.75, 0.3).unwrap();
        let capsule = Shape::new_capsule(0.70845, 0.4).unwrap();
        // SAFETY: both shapes are live and of the type each getter expects (their subtypes are
        // checked in `new_shapes_have_jolt_subtypes`); the getters only read them.
        unsafe {
            assert_eq!(
                JPH_CylinderShape_GetHalfHeight(cylinder.as_ptr().cast()),
                0.75
            );
            assert_eq!(JPH_CylinderShape_GetRadius(cylinder.as_ptr().cast()), 0.3);
            assert_eq!(
                JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
                0.70845
            );
            assert_eq!(JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()), 0.4);
        }
    }

    #[test]
    fn inflated_grows_spheres_and_capsules_only() {
        let sphere = Shape::new_sphere(0.05)
            .unwrap()
            .inflated(0.1)
            .unwrap()
            .unwrap();
        assert_eq!(sphere.sub_type(), JPH_ShapeSubType_Sphere);
        // SAFETY: the shape is live and a sphere; the getter only reads it.
        let radius = unsafe { JPH_SphereShape_GetRadius(sphere.as_ptr().cast()) };
        assert!((radius - 0.15).abs() < 1e-6);

        let capsule = Shape::new_capsule(0.7, 0.4)
            .unwrap()
            .inflated(0.1)
            .unwrap()
            .unwrap();
        // SAFETY: the shape is live and a capsule; the getters only read it.
        unsafe {
            assert_eq!(
                JPH_CapsuleShape_GetHalfHeightOfCylinder(capsule.as_ptr().cast()),
                0.7
            );
            assert!((JPH_CapsuleShape_GetRadius(capsule.as_ptr().cast()) - 0.5).abs() < 1e-6);
        }

        let unit_box = unit_box();
        assert!(unit_box.inflated(0.1).unwrap().is_none());
        let huge = Shape::new_capsule(1.0, f32::MAX).unwrap();
        assert!(huge.inflated(f32::MAX).is_err());
    }

    #[test]
    fn static_only_and_center_of_mass_are_read_from_jolt() {
        assert!(!unit_box().must_be_static());
        let terrain = Shape::new_height_field(3, &[0.0; 9], &HeightFieldSettings::default());
        assert!(terrain.unwrap().must_be_static());
        assert_eq!(unit_box().center_of_mass(), Vec3::ZERO);
        let unit_box = unit_box();
        let pair =
            Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
        assert_eq!(pair.center_of_mass(), Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn new_shapes_have_jolt_subtypes() {
        let unit = Vec3::new(1.0, 1.0, 1.0);
        assert_eq!(
            Shape::new_box(unit).unwrap().sub_type(),
            JPH_ShapeSubType_Box
        );
        assert_eq!(
            Shape::new_cylinder(1.0, 0.5).unwrap().sub_type(),
            JPH_ShapeSubType_Cylinder
        );
        assert_eq!(
            Shape::new_capsule(1.0, 0.5).unwrap().sub_type(),
            JPH_ShapeSubType_Capsule
        );
        assert_eq!(
            Shape::new_sphere(1.0).unwrap().sub_type(),
            JPH_ShapeSubType_Sphere
        );
    }

    #[test]
    fn height_field_settings_reach_jolt() {
        let settings = HeightFieldSettings::default()
            .block_size(4)
            .bits_per_sample(12)
            .active_edge_cos_threshold_angle(0.5);
        assert_ne!(settings, HeightFieldSettings::default());
        let samples = [0.0; 81];
        assert_eq!(settings.validate_layout(9), Ok(12));
        assert!(ensure_initialized());
        // SAFETY: Jolt is initialised and the inputs are valid (checked above).
        let jolt_settings = unsafe { height_field_settings(9, &samples, &settings) }.unwrap();
        let ptr = jolt_settings.as_ptr();
        // SAFETY: the settings are live and were created as heightfield settings; getters only
        // read them.
        unsafe {
            assert_eq!(JPH_HeightFieldShapeSettings_GetBlockSize(ptr), 4);
            assert_eq!(JPH_HeightFieldShapeSettings_GetBitsPerSample(ptr), 12);
            assert_eq!(
                JPH_HeightFieldShapeSettings_GetActiveEdgeCosThresholdAngle(ptr),
                0.5
            );
        }
        let shape = Shape::new_height_field(9, &samples, &settings).unwrap();
        // SAFETY: the shape is live and a heightfield; the getter only reads it.
        let block_size = unsafe { JPH_HeightFieldShape_GetBlockSize(shape.as_ptr().cast()) };
        assert_eq!(block_size, 4);
    }

    fn unit_box() -> Shape {
        Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap()
    }

    fn child(shape: &Shape, x: f32, user_data: u32) -> CompoundChild<'_> {
        CompoundChild {
            shape,
            position: Vec3::new(x, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            user_data,
        }
    }

    #[test]
    fn compound_subtypes_depend_on_the_child_count() {
        let unit_box = unit_box();
        let single = Shape::new_compound(&[child(&unit_box, 1.0, 7)]).unwrap();
        assert_eq!(single.sub_type(), JPH_ShapeSubType_MutableCompound);
        let pair =
            Shape::new_compound(&[child(&unit_box, 0.0, 1), child(&unit_box, 2.0, 2)]).unwrap();
        assert_eq!(pair.sub_type(), JPH_ShapeSubType_StaticCompound);
    }

    #[test]
    fn out_of_range_sub_shape_ids_are_rejected() {
        let unit_box = unit_box();
        let children = [
            child(&unit_box, 0.0, 10),
            child(&unit_box, 2.0, 11),
            child(&unit_box, 4.0, 12),
        ];
        let compound = Shape::new_compound(&children).unwrap();
        // Three children need two bits; the remaining bits are the path below the child.
        for index in 0..3 {
            let id = SubShapeId::new(0xffff_fffc | index);
            assert_eq!(
                compound.compound_sub_shape(id),
                Some(CompoundSubShape {
                    index,
                    user_data: 10 + index
                })
            );
        }
        for raw in [3, 7, u32::MAX] {
            assert_eq!(compound.compound_sub_shape(SubShapeId::new(raw)), None);
        }
        assert_eq!(unit_box.compound_sub_shape(SubShapeId::new(0)), None);
    }
}
