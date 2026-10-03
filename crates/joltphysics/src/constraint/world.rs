//! Constraints owned by a [`PhysicsWorld`]: typed ids, the world's registry, and read and write
//! views.

use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::fmt;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;
use std::ptr::NonNull;

use joltphysics_sys::*;

use super::SpringSettings;
use crate::body::{with_locked_bodies, with_read_locked_body};
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyError, BodyId, ConstraintError, PhysicsWorld};

/// The kinds of constraint a world can hold.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum ConstraintType {
    /// [`FixedConstraint`].
    Fixed,
    /// [`PointConstraint`].
    Point,
    /// [`DistanceConstraint`].
    Distance,
    /// [`HingeConstraint`].
    Hinge,
    /// [`SliderConstraint`].
    Slider,
    /// [`ConeConstraint`].
    Cone,
    /// [`SwingTwistConstraint`].
    SwingTwist,
    /// [`SixDofConstraint`].
    SixDof,
    /// [`GearConstraint`].
    Gear,
    /// [`RackAndPinionConstraint`].
    RackAndPinion,
    /// [`PulleyConstraint`].
    Pulley,
    /// [`PathConstraint`].
    Path,
}

/// The traits users can name but not implement or call: the constraint kinds and settings are
/// the crate's own.
pub(crate) mod sealed {
    use super::*;

    /// A constraint kind marker.
    pub trait Kind {
        /// The kind as a value.
        const TYPE: ConstraintType;
        /// Jolt's subtype of the kind's constraint.
        const SUB_TYPE: JPH_ConstraintSubType;
    }

    /// Settings a world constraint is created from.
    pub trait Settings {
        /// What Jolt asserts or silently rewrites, checked before anything is created.
        fn validate(&self) -> Result<(), &'static str>;

        /// Every spring whose frequency-mode coefficients depend on the bodies' effective mass.
        fn springs(&self) -> Vec<SpringSettings>;

        /// The constraints this one references (a gear's hinges, a rack and pinion's hinge and
        /// slider).
        fn references(&self) -> [Option<AnyConstraintId>; 2] {
            [None, None]
        }

        /// Creates the Jolt constraint between the two bodies and returns it holding the one
        /// reference joltc returns, or null when joltc could not allocate it.
        ///
        /// # Safety
        /// Both bodies are distinct, live bodies of one world, locked for writing for the call,
        /// and `validate` returned `Ok`.
        unsafe fn create(
            &self,
            body1: NonNull<JPH_Body>,
            body2: NonNull<JPH_Body>,
        ) -> *mut JPH_Constraint;
    }
}

/// A kind of world constraint, named by the zero-sized markers such as [`HingeConstraint`]. It
/// selects the methods of [`ConstraintRef`] and [`ConstraintMut`].
pub trait ConstraintKind: sealed::Kind {}

/// Settings a world constraint is created from with [`PhysicsWorld::create_constraint`].
pub trait ConstraintSettings: sealed::Settings {
    /// The kind of constraint these settings create.
    type Kind: ConstraintKind;
}

macro_rules! constraint_kinds {
    ($($(#[$doc:meta])* $name:ident => $kind:ident, $sub_type:ident;)*) => {
        $(
            $(#[$doc])*
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
            pub enum $name {}

            impl sealed::Kind for $name {
                const TYPE: ConstraintType = ConstraintType::$kind;
                const SUB_TYPE: JPH_ConstraintSubType = $sub_type;
            }

            impl ConstraintKind for $name {}
        )*
    };
}

constraint_kinds! {
    /// A fixed constraint (Jolt `FixedConstraint`): the bodies move as one.
    FixedConstraint => Fixed, JPH_ConstraintSubType_Fixed;
    /// A point constraint (Jolt `PointConstraint`): a ball joint.
    PointConstraint => Point, JPH_ConstraintSubType_Point;
    /// A distance constraint (Jolt `DistanceConstraint`): a rod or rope between two points.
    DistanceConstraint => Distance, JPH_ConstraintSubType_Distance;
    /// A hinge (Jolt `HingeConstraint`).
    HingeConstraint => Hinge, JPH_ConstraintSubType_Hinge;
    /// A slider (Jolt `SliderConstraint`): translation along one axis.
    SliderConstraint => Slider, JPH_ConstraintSubType_Slider;
    /// A cone constraint (Jolt `ConeConstraint`): a ball joint whose twist axis stays in a cone.
    ConeConstraint => Cone, JPH_ConstraintSubType_Cone;
    /// A swing-twist constraint (Jolt `SwingTwistConstraint`).
    SwingTwistConstraint => SwingTwist, JPH_ConstraintSubType_SwingTwist;
    /// A six-degree-of-freedom constraint (Jolt `SixDOFConstraint`).
    SixDofConstraint => SixDof, JPH_ConstraintSubType_SixDOF;
    /// A gear (Jolt `GearConstraint`): couples the rotation of two bodies.
    GearConstraint => Gear, JPH_ConstraintSubType_Gear;
    /// A rack and pinion (Jolt `RackAndPinionConstraint`): couples a rotation and a translation.
    RackAndPinionConstraint => RackAndPinion, JPH_ConstraintSubType_RackAndPinion;
    /// A pulley (Jolt `PulleyConstraint`): a rope over two fixed points.
    PulleyConstraint => Pulley, JPH_ConstraintSubType_Pulley;
    /// A path constraint (Jolt `PathConstraint`): body 2 moves along a path fixed to body 1.
    PathConstraint => Path, JPH_ConstraintSubType_Path;
}

/// Identifies a constraint of kind `K` in the world that created it.
///
/// The raw value is 1 for the first constraint of a world, then 2, 3 and so on, whatever the
/// kind; ids are never reused within a world, so the same creation history gives the same ids.
/// Ids order by creation, then by world. Using an id with another world returns
/// [`ConstraintError::WrongWorld`].
pub struct ConstraintId<K> {
    raw: u32,
    world: WorldTag,
    kind: PhantomData<fn() -> K>,
}

impl<K: ConstraintKind> ConstraintId<K> {
    fn new(raw: u32, world: WorldTag) -> Self {
        Self {
            raw,
            world,
            kind: PhantomData,
        }
    }
}

impl<K> ConstraintId<K> {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl<K> Clone for ConstraintId<K> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<K> Copy for ConstraintId<K> {}

impl<K> PartialEq for ConstraintId<K> {
    fn eq(&self, other: &Self) -> bool {
        (self.raw, self.world) == (other.raw, other.world)
    }
}

impl<K> Eq for ConstraintId<K> {}

impl<K> PartialOrd for ConstraintId<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K> Ord for ConstraintId<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.raw, self.world).cmp(&(other.raw, other.world))
    }
}

impl<K> Hash for ConstraintId<K> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (self.raw, self.world).hash(state);
    }
}

impl<K: ConstraintKind> fmt::Debug for ConstraintId<K> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ConstraintId")
            .field(&K::TYPE)
            .field(&self.raw)
            .finish()
    }
}

/// A constraint id of any kind, for listing and removing constraints.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct AnyConstraintId {
    // Declared first, so ids order by creation before the world.
    raw: u32,
    world: WorldTag,
    kind: ConstraintType,
}

impl AnyConstraintId {
    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }

    /// The kind of constraint the id names.
    pub fn kind(self) -> ConstraintType {
        self.kind
    }

    /// The typed id, if the constraint is of kind `K`.
    pub fn downcast<K: ConstraintKind>(self) -> Option<ConstraintId<K>> {
        (self.kind == K::TYPE).then(|| ConstraintId::new(self.raw, self.world))
    }
}

impl<K: ConstraintKind> From<ConstraintId<K>> for AnyConstraintId {
    fn from(id: ConstraintId<K>) -> Self {
        Self {
            raw: id.raw,
            world: id.world,
            kind: K::TYPE,
        }
    }
}

impl fmt::Debug for AnyConstraintId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("ConstraintId")
            .field(&self.kind)
            .field(&self.raw)
            .finish()
    }
}

/// The state of a constraint motor (Jolt `EMotorState`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum MotorState {
    /// The motor is off. The default.
    #[default]
    Off,
    /// The motor drives to a target velocity.
    Velocity,
    /// The motor drives to a target position or angle.
    Position,
}

impl MotorState {
    pub(crate) fn to_jph(self) -> JPH_MotorState {
        match self {
            Self::Off => JPH_MotorState_Off,
            Self::Velocity => JPH_MotorState_Velocity,
            Self::Position => JPH_MotorState_Position,
        }
    }

    pub(crate) fn from_jph(state: JPH_MotorState) -> Self {
        if state == JPH_MotorState_Velocity {
            Self::Velocity
        } else if state == JPH_MotorState_Position {
            Self::Position
        } else {
            Self::Off
        }
    }
}

/// A world constraint, of which the world holds the one reference `JPH_*Constraint_Create`
/// returns (joltc `AddRef`s the new object). `JPH_PhysicsSystem_AddConstraint` adds the system's
/// own reference and `JPH_PhysicsSystem_RemoveConstraint` drops it; a gear or rack and pinion
/// holds a `RefConst` on each constraint it references.
impl JoltObject for JPH_Constraint {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), which `Release` drops; the
        // world removes the constraint from the system first.
        unsafe { JPH_Constraint_Destroy(ptr) };
    }
}

/// One constraint of a world.
pub(crate) struct ConstraintEntry {
    pub(crate) constraint: Owned<JPH_Constraint>,
    pub(crate) kind: ConstraintType,
    pub(crate) bodies: [BodyId; 2],
    /// An upper bound of the effective mass (kg) or inertia (kg·m²) any part of the constraint
    /// sees, from the bodies at creation; frequency-mode springs are checked against it.
    pub(crate) effective_mass_bound: f64,
    /// Raw ids of the constraints this one references.
    pub(crate) references: [Option<u32>; 2],
    /// Raw ids of the constraints that reference this one.
    pub(crate) referenced_by: BTreeSet<u32>,
}

/// Read access to one constraint, borrowed from its world.
pub struct ConstraintRef<'w, K> {
    id: ConstraintId<K>,
    entry: &'w ConstraintEntry,
}

impl<K: ConstraintKind> ConstraintRef<'_, K> {
    /// The constraint as the joltc handle `T` of its kind, which is the same object
    /// (joltc casts the most derived object).
    pub(crate) fn ptr<T>(&self) -> *mut T {
        self.entry.constraint.as_ptr().cast()
    }

    /// The constraint's id.
    pub fn id(&self) -> ConstraintId<K> {
        self.id
    }

    /// Its two bodies, body 1 and body 2.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.entry.bodies
    }

    /// Whether the constraint is enabled.
    pub fn is_enabled(&self) -> bool {
        // SAFETY: the world borrowed here owns the constraint; `GetEnabled` reads a member, and
        // writes need `&mut PhysicsWorld`.
        unsafe { JPH_Constraint_GetEnabled(self.ptr()) }
    }

    /// Whether Jolt simulates the constraint: enabled and with at least one awake body that can
    /// move (Jolt `IsActive`).
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `is_enabled`; `IsActive` reads the constraint's flag and its bodies'
        // motion and activation state, which Jolt writes only in `step` and the body and
        // constraint setters, all behind `&mut PhysicsWorld`.
        unsafe { JPH_Constraint_IsActive(self.ptr()) }
    }
}

/// Write access to one constraint, borrowed mutably from its world.
///
/// Read the constraint through [`PhysicsWorld::constraint`] once this borrow ends.
pub struct ConstraintMut<'w, K> {
    world: &'w mut PhysicsWorld,
    id: ConstraintId<K>,
}

impl<K: ConstraintKind> ConstraintMut<'_, K> {
    pub(crate) fn entry(&self) -> &ConstraintEntry {
        self.world
            .constraints
            .get(&self.id.raw)
            .unwrap_or_else(|| unreachable!("checked when the view was made"))
    }

    /// The constraint as the joltc handle `T` of its kind.
    pub(crate) fn ptr<T>(&self) -> *mut T {
        self.entry().constraint.as_ptr().cast()
    }

    /// The constraint's id.
    pub fn id(&self) -> ConstraintId<K> {
        self.id
    }

    /// Its two bodies, body 1 and body 2.
    pub fn bodies(&self) -> [BodyId; 2] {
        self.entry().bodies
    }

    /// Whether the constraint is enabled.
    pub fn is_enabled(&self) -> bool {
        // SAFETY: the world is borrowed through this view and owns the constraint; the getter
        // reads a member.
        unsafe { JPH_Constraint_GetEnabled(self.ptr()) }
    }

    /// Whether Jolt simulates the constraint (Jolt `IsActive`).
    pub fn is_active(&self) -> bool {
        // SAFETY: as in `is_enabled`; the getter reads the constraint and its bodies' state.
        unsafe { JPH_Constraint_IsActive(self.ptr()) }
    }

    /// Enables or disables the constraint and wakes its bodies that can move, so a sleeping body
    /// starts or stops following the constraint at the next step.
    pub fn set_enabled(&mut self, enabled: bool) {
        // SAFETY: the world is borrowed mutably through this view and owns the constraint; no
        // step runs. The setter writes a member.
        unsafe { JPH_Constraint_SetEnabled(self.ptr(), enabled) };
        self.wake_bodies();
    }

    /// Wakes the constraint's bodies that can move, in body id order.
    pub(crate) fn wake_bodies(&mut self) {
        let bodies = self.entry().bodies;
        self.world.wake_constraint_bodies(bodies);
    }

    /// The bound springs set on this constraint are checked against.
    pub(crate) fn effective_mass_bound(&self) -> f64 {
        self.entry().effective_mass_bound
    }
}

/// `Err(InvalidValue)` unless `spring` is valid and fits `bound`.
pub(crate) fn check_spring(spring: SpringSettings, bound: f64) -> Result<(), ConstraintError> {
    spring.validate().map_err(ConstraintError::InvalidValue)?;
    if spring.fits_effective_mass(bound) {
        Ok(())
    } else {
        Err(ConstraintError::InvalidValue(SPRING_BOUND_RULE))
    }
}

/// What every frequency-mode spring of a world constraint must satisfy.
const SPRING_BOUND_RULE: &str = "spring stiffness and damping derived from the bodies' effective mass must be at most limits::MAX_SPRING_COEFFICIENT";

impl PhysicsWorld {
    /// Creates a constraint between `body1` and `body2` and returns its typed id.
    ///
    /// Frames given in [`ConstraintSpace::WorldSpace`](crate::ConstraintSpace::WorldSpace) are
    /// world space at the time the constraint is created; Jolt turns them into each body's own
    /// frame then. Both bodies must be bodies of this world, different, and neither the inner
    /// body of a character nor a part of a ragdoll ([`ConstraintError::Body`]). One of them may
    /// be static, which anchors the constraint to the world. Jolt still lets the two bodies
    /// collide with each other where their shapes touch. While the constraint exists,
    /// [`remove_body`](Self::remove_body) refuses its bodies ([`BodyError::UsedByConstraint`]).
    ///
    /// Frequency-mode springs ([`SpringSettings::FrequencyAndDamping`]) become a stiffness and
    /// damping that grow with the bodies' effective mass, so they are checked against an upper
    /// bound of it, taken from the dynamic bodies' mass and inertia now: `max(mass, largest
    /// principal moment)` of each. The public API cannot change a body's mass or shape
    /// afterwards, and only ragdoll parts, which are refused here, can turn kinematic or
    /// static bodies dynamic, so the bound stays valid for the constraint's life.
    ///
    /// Fails with [`ConstraintError::InvalidValue`] when a setting is out of range, with
    /// [`ConstraintError::NotFound`] or [`ConstraintError::WrongWorld`] for a referenced
    /// constraint that is not in this world, and with [`ConstraintError::TooManyConstraints`]
    /// when the world has run out of ids. Nothing changes on failure.
    ///
    /// # Example
    /// ```
    /// use joltphysics::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let post = Shape::new_box(Vec3::new(0.1, 1.0, 0.1))?;
    /// let frame = world.create_body(&post, &BodySettings::new_static())?;
    /// let panel = Shape::new_box(Vec3::new(0.5, 1.0, 0.05))?;
    /// // Jolt lets the two bodies of a constraint collide, so the door leaves a gap to the post.
    /// let door = world.create_body(
    ///     &panel,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.7, 0.0, 0.0)),
    /// )?;
    ///
    /// // A vertical hinge at the door's edge, driven to a quarter turn.
    /// let up = Vec3::new(0.0, 1.0, 0.0);
    /// let side = Vec3::new(1.0, 0.0, 0.0);
    /// let hinge = world.create_constraint(
    ///     frame,
    ///     door,
    ///     &HingeConstraintSettings::new(RVec3::new(0.15, 0.0, 0.0), up, side),
    /// )?;
    /// let mut motor = world.constraint_mut(hinge)?;
    /// motor.set_motor_state(MotorState::Position);
    /// motor.set_target_angle(std::f32::consts::FRAC_PI_2)?;
    /// for _ in 0..120 {
    ///     world.step(1.0 / 60.0)?;
    /// }
    /// let angle = world.constraint(hinge)?.current_angle();
    /// assert!((angle - std::f32::consts::FRAC_PI_2).abs() < 0.05);
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_constraint<S: ConstraintSettings>(
        &mut self,
        body1: BodyId,
        body2: BodyId,
        settings: &S,
    ) -> Result<ConstraintId<S::Kind>, ConstraintError> {
        sealed::Settings::validate(settings).map_err(ConstraintError::InvalidValue)?;
        for body in [body1, body2] {
            self.check(body).map_err(ConstraintError::Body)?;
        }
        if body1 == body2 {
            return Err(ConstraintError::InvalidValue(
                "a constraint joins two different bodies",
            ));
        }
        for body in [body1, body2] {
            // The character and the ragdoll destroy these bodies themselves, behind the body
            // guard, which the constraint would outlive.
            if self.is_inner_body(body) {
                return Err(ConstraintError::Body(BodyError::OwnedByCharacter(body)));
            }
            if self.is_ragdoll_body(body) {
                return Err(ConstraintError::Body(BodyError::OwnedByRagdoll(body)));
            }
        }
        let references = settings.references();
        for reference in references.into_iter().flatten() {
            self.constraint_entry(reference)?;
        }
        let raw = self.next_constraint_id;
        if raw == u32::MAX {
            return Err(ConstraintError::TooManyConstraints);
        }
        let effective_mass_bound = self.effective_mass_bound([body1, body2]);
        for spring in settings.springs() {
            check_spring(spring, effective_mass_bound)?;
        }

        let constraint =
            with_locked_bodies(self.body_lock_interface, [body1, body2], |first, second| {
                // SAFETY: both bodies are distinct live bodies of this world (checked above),
                // locked for writing for the call, and the settings are validated. Jolt bodies
                // stay at their address until destroyed, and `remove_body` refuses them while
                // the constraint exists. The handle takes over the one reference joltc returns.
                unsafe { Owned::from_raw(sealed::Settings::create(settings, first, second)) }
            })
            .unwrap_or_else(|| unreachable!("`check` found both bodies and `&mut self` keeps them"))
            // A failed native allocation is not a recoverable error here, as everywhere joltc `new`s.
            .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
        debug_assert_eq!(
            // SAFETY: the constraint is live; the getter is a virtual call that only returns the type.
            unsafe { JPH_Constraint_GetSubType(constraint.as_ptr()) },
            <S::Kind as sealed::Kind>::SUB_TYPE
        );

        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs. The system takes its own reference; `remove_constraint` and
        // `remove_all_constraints` remove it before the world releases its own.
        unsafe { JPH_PhysicsSystem_AddConstraint(self.system.as_ptr(), constraint.as_ptr()) };
        let references = references.map(|reference| reference.map(|id| id.raw));
        for reference in references.into_iter().flatten() {
            if let Some(entry) = self.constraints.get_mut(&reference) {
                entry.referenced_by.insert(raw);
            }
        }
        for body in [body1, body2] {
            self.constraint_bodies
                .entry(body.to_raw())
                .or_default()
                .insert(raw);
        }
        self.constraints.insert(
            raw,
            ConstraintEntry {
                constraint,
                kind: <S::Kind as sealed::Kind>::TYPE,
                bodies: [body1, body2],
                effective_mass_bound,
                references,
                referenced_by: BTreeSet::new(),
            },
        );
        self.next_constraint_id += 1;
        Ok(ConstraintId::new(raw, self.tag))
    }

    /// An upper bound of the effective mass or inertia a constraint between `bodies` sees: over
    /// the dynamic bodies, the largest of each one's mass and largest principal moment of
    /// inertia; 0 without a dynamic body.
    ///
    /// The inverse effective mass of a translation part is at least the inverse mass of a
    /// dynamic body it connects, and that of a rotation part at least that body's smallest
    /// principal inverse inertia.
    fn effective_mass_bound(&self, bodies: [BodyId; 2]) -> f64 {
        let bound_of = |id| {
            with_read_locked_body(self.body_lock_interface, id, |body| {
                // SAFETY: `body` is locked for reading for the duration of the closure. A
                // dynamic body has motion properties, so the unchecked getter reads a live
                // member; the getters only read, and `inverse_inertia` is a live local.
                unsafe {
                    if !JPH_Body_IsDynamic(body.as_ptr()) {
                        return 0.0;
                    }
                    let motion = JPH_Body_GetMotionProperties(body.as_ptr());
                    let mut inverse_inertia = crate::Vec3::ZERO.to_jph();
                    JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
                    let inverse_mass = JPH_MotionProperties_GetInverseMassUnchecked(motion);
                    let smallest = inverse_inertia
                        .x
                        .min(inverse_inertia.y)
                        .min(inverse_inertia.z);
                    // A zero inverse inertia (a locked rotation) would make the inertia
                    // unbounded; shapes give every body a finite one, so this is defensive.
                    let inertia = if smallest > 0.0 {
                        1.0 / f64::from(smallest)
                    } else {
                        f64::INFINITY
                    };
                    (1.0 / f64::from(inverse_mass)).max(inertia)
                }
            })
            .unwrap_or_else(|| unreachable!("checked by the caller"))
        };
        bodies.into_iter().map(bound_of).fold(0.0, f64::max)
    }

    /// The entry of `id`, if it names a constraint of this world.
    pub(crate) fn constraint_entry(
        &self,
        id: impl Into<AnyConstraintId>,
    ) -> Result<&ConstraintEntry, ConstraintError> {
        let id = id.into();
        if id.world != self.tag {
            return Err(ConstraintError::WrongWorld(id));
        }
        self.constraints
            .get(&id.raw)
            .ok_or(ConstraintError::NotFound(id))
    }

    /// Removes a constraint and wakes its bodies that can move, in body id order, so a body the
    /// constraint held does not stay asleep where it was.
    ///
    /// A hinge or slider that a gear or rack and pinion references cannot be removed while the
    /// coupling exists ([`ConstraintError::UsedByConstraint`]); remove the coupling first.
    pub fn remove_constraint(
        &mut self,
        id: impl Into<AnyConstraintId>,
    ) -> Result<(), ConstraintError> {
        let id = id.into();
        let entry = self.constraint_entry(id)?;
        if let Some(&dependent) = entry.referenced_by.first() {
            let dependent = AnyConstraintId {
                raw: dependent,
                world: self.tag,
                kind: self.constraints[&dependent].kind,
            };
            return Err(ConstraintError::UsedByConstraint(dependent));
        }
        let entry = self
            .constraints
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        let bodies = entry.bodies;
        self.unregister_constraint(id.raw, entry);
        self.wake_constraint_bodies(bodies);
        Ok(())
    }

    /// Takes the constraint out of the system and of the world's indexes, then releases the
    /// world's reference.
    fn unregister_constraint(&mut self, raw: u32, entry: ConstraintEntry) {
        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs; `create_constraint` added the constraint.
        unsafe {
            JPH_PhysicsSystem_RemoveConstraint(self.system.as_ptr(), entry.constraint.as_ptr())
        };
        for body in entry.bodies {
            if let Some(ids) = self.constraint_bodies.get_mut(&body.to_raw()) {
                ids.remove(&raw);
                if ids.is_empty() {
                    self.constraint_bodies.remove(&body.to_raw());
                }
            }
        }
        for reference in entry.references.into_iter().flatten() {
            if let Some(referenced) = self.constraints.get_mut(&reference) {
                referenced.referenced_by.remove(&raw);
            }
        }
        drop(entry);
    }

    /// Removes every constraint, newest first. Runs when the world is dropped, before anything
    /// else is released. A coupling is always newer than the constraints it references, so
    /// dependents go first; the coupling's own references would keep a referenced hinge alive
    /// in any order.
    pub(crate) fn remove_all_constraints(&mut self) {
        while let Some((raw, entry)) = self.constraints.pop_last() {
            self.unregister_constraint(raw, entry);
        }
    }

    /// Wakes `bodies` that can move, in body id order; Jolt skips static bodies.
    pub(crate) fn wake_constraint_bodies(&mut self, bodies: [BodyId; 2]) {
        let mut ids = bodies.map(BodyId::to_raw);
        ids.sort_unstable();
        // SAFETY: the body interface belongs to this live world, borrowed mutably; both ids name
        // bodies of this world, which `remove_body` keeps while a constraint uses them, and
        // `ids` lives for the call. This thread holds no body lock.
        unsafe { JPH_BodyInterface_ActivateBodies(self.body_interface.as_ptr(), ids.as_ptr(), 2) };
    }

    /// Read access to a constraint.
    pub fn constraint<K: ConstraintKind>(
        &self,
        id: ConstraintId<K>,
    ) -> Result<ConstraintRef<'_, K>, ConstraintError> {
        let entry = self.constraint_entry(id)?;
        debug_assert_eq!(entry.kind, K::TYPE, "ids are typed by their kind");
        Ok(ConstraintRef { id, entry })
    }

    /// Write access to a constraint.
    pub fn constraint_mut<K: ConstraintKind>(
        &mut self,
        id: ConstraintId<K>,
    ) -> Result<ConstraintMut<'_, K>, ConstraintError> {
        let entry = self.constraint_entry(id)?;
        debug_assert_eq!(entry.kind, K::TYPE, "ids are typed by their kind");
        Ok(ConstraintMut { world: self, id })
    }

    /// The ids of the world's constraints, in id order (creation order).
    pub fn constraint_ids(&self) -> impl Iterator<Item = AnyConstraintId> + '_ {
        self.constraints
            .iter()
            .map(|(&raw, entry)| AnyConstraintId {
                raw,
                world: self.tag,
                kind: entry.kind,
            })
    }

    /// The ids of the constraints that use `body`, in id order.
    pub fn constraints_of_body(&self, body: BodyId) -> Vec<AnyConstraintId> {
        if body.world != self.tag {
            return Vec::new();
        }
        self.constraint_bodies
            .get(&body.to_raw())
            .into_iter()
            .flatten()
            .map(|&raw| AnyConstraintId {
                raw,
                world: self.tag,
                kind: self.constraints[&raw].kind,
            })
            .collect()
    }

    /// Number of constraints in the world.
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }

    /// Whether a constraint of this world uses `body`.
    pub(crate) fn is_constraint_body(&self, body: BodyId) -> bool {
        body.world == self.tag && self.constraint_bodies.contains_key(&body.to_raw())
    }
}
