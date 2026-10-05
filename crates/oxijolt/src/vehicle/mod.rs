//! Vehicles: Jolt's `VehicleConstraint` with the wheeled or the tracked controller, owned by a
//! [`PhysicsWorld`] and attached to a dynamic chassis body the caller created. The kind of a
//! vehicle is part of its id ([`VehicleId<K>`](VehicleId)).
//!
//! # Vehicles and the step
//! A vehicle is a constraint and a step listener of the world's Jolt system. During
//! [`PhysicsWorld::step`] Jolt runs it before the collision step: it casts every wheel against
//! the scene, applies gravity (or the override of [`VehicleMut::set_gravity`]), then the engine,
//! brakes and tire friction act through the constraint. No Rust code runs inside the step, and
//! no vehicle can be read or changed during it, because the step takes `&mut PhysicsWorld`.
//!
//! The chassis stays an ordinary body owned by the caller: its pose and velocities come from
//! [`PhysicsWorld::body`], and it cannot be removed while a vehicle uses it. For a vehicle under
//! the caller's own (for example radial) gravity, create the chassis with
//! [`BodySettings::allow_sleeping(false)`](crate::BodySettings::allow_sleeping) and
//! [`gravity_factor(0.0)`](crate::BodySettings::gravity_factor), and set the gravity at the
//! vehicle every tick.

mod control;
mod id;
mod readout;
mod settings;
mod tracked;

use std::marker::PhantomData;
use std::ops::Range;

use oxijolt_sys::*;

use crate::body::with_locked_body;
use crate::constraint::constraint_base;
use crate::math::is_unit;
use crate::owned::{JoltObject, Owned};
use crate::{AllowedDofs, BodyError, BodyId, MotionType, PhysicsWorld, Vec3, VehicleError};

pub use control::{DriverInput, VehicleMut};
pub use id::{
    AnyVehicleId, Motorcycle, TrackedVehicle, VehicleId, VehicleKind, VehicleType, WheeledVehicle,
};
pub use readout::{VehicleRef, WheelContact, WheelState};
use settings::{BuiltSettings, WheelGeometry};
pub use settings::{
    SuspensionSpring, TrackedVehicleSettings, TrackedWheelSettings, VehicleAntiRollBar,
    VehicleCollisionTester, VehicleDifferentialSettings, VehicleEngineSettings, VehicleSettings,
    VehicleTrackSettings, VehicleTransmissionSettings, WheelSettings, DEFAULT_LATERAL_FRICTION,
    DEFAULT_LONGITUDINAL_FRICTION, DEFAULT_NORMALIZED_TORQUE,
};
pub use tracked::{TrackSide, TrackState, TrackedDriverInput};

/// A vehicle constraint, of which the world holds the one reference
/// `JPH_VehicleConstraint_Create` returns. The physics system holds another while the vehicle is
/// registered as a constraint, and a plain pointer while it is a step listener, so the world
/// removes it from both before releasing its reference.
impl JoltObject for JPH_VehicleConstraint {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract) and has removed the vehicle
        // from the system's step listeners and constraints (`remove_vehicle`,
        // `remove_all_vehicles`). The `Constraint` base is the object `Release` acts on.
        unsafe { JPH_Constraint_Destroy(JPH_VehicleConstraint_AsConstraint(ptr)) };
    }
}

/// One vehicle of a world.
pub(crate) struct VehicleEntry {
    constraint: Owned<JPH_VehicleConstraint>,
    pub(crate) body: BodyId,
    pub(crate) collision_tester: VehicleCollisionTester,
    wheels: Vec<WheelGeometry>,
    kind: VehicleType,
    /// The vehicle wheel indices of a tracked vehicle's left and right track.
    tracks: Option<[Range<u32>; 2]>,
}

/// The gravity override of the vehicle, or `None` while it uses the world's gravity.
fn gravity_override(entry: &VehicleEntry) -> Option<Vec3> {
    let constraint = entry.constraint.as_ptr();
    // SAFETY: the caller borrows the world that owns the constraint, and changing it needs
    // `&mut PhysicsWorld`; the getters read members, and `value` is a live local.
    unsafe {
        JPH_VehicleConstraint_IsGravityOverridden(constraint).then(|| {
            let mut value = Vec3::ZERO.to_jph();
            JPH_VehicleConstraint_GetGravityOverride(constraint, &mut value);
            Vec3::from_jph(value)
        })
    }
}

/// The engine and transmission of the vehicle's controller, members of the controller that
/// live as long as the constraint. The only place that casts a controller without a typed id:
/// it matches on the kind the entry was created with.
fn engine_and_transmission(
    entry: &VehicleEntry,
) -> (*const JPH_VehicleEngine, *const JPH_VehicleTransmission) {
    // SAFETY: the caller borrows the world that owns the constraint; the getter returns a
    // member.
    let controller = unsafe { JPH_VehicleConstraint_GetController(entry.constraint.as_ptr()) };
    match entry.kind {
        // SAFETY: a wheeled vehicle's and a motorcycle's controller derive from
        // `WheeledVehicleController` with single inheritance; the getters return members.
        VehicleType::Wheeled | VehicleType::Motorcycle => unsafe {
            let controller: *const JPH_WheeledVehicleController = controller.cast();
            (
                JPH_WheeledVehicleController_GetEngine(controller),
                JPH_WheeledVehicleController_GetTransmission(controller),
            )
        },
        // SAFETY: a tracked vehicle's controller derives from `VehicleController` with single
        // inheritance; the getters return members.
        VehicleType::Tracked => unsafe {
            let controller: *const JPH_TrackedVehicleController = controller.cast();
            (
                JPH_TrackedVehicleController_GetEngine(controller),
                JPH_TrackedVehicleController_GetTransmission(controller),
            )
        },
    }
}

/// Gives the vehicle a new Jolt tester built from the validated `tester` and records it.
fn install_tester(entry: &mut VehicleEntry, tester: VehicleCollisionTester) {
    let jolt_tester = tester.create(entry.body.to_raw());
    // SAFETY: the world is borrowed mutably (the caller holds the entry mutably) and owns the
    // constraint; no step runs. The constraint takes its own reference to the tester and
    // releases the one to the tester it replaces; the guard releases ours afterwards.
    unsafe {
        JPH_VehicleConstraint_SetVehicleCollisionTester(
            entry.constraint.as_ptr(),
            jolt_tester.as_ptr(),
        )
    };
    entry.collision_tester = tester;
}

impl PhysicsWorld {
    /// Attaches a wheeled vehicle to the dynamic body `body`, its chassis, and returns its id.
    ///
    /// The chassis' shape gives the vehicle its mass and centre of mass; a low centre of mass
    /// ([`Shape::new_offset_center_of_mass`](crate::Shape::new_offset_center_of_mass)) keeps it
    /// from rolling over.
    ///
    /// # Vehicles and the step
    /// From now on the vehicle runs inside every [`step`](Self::step), as a constraint and a
    /// step listener of the world's Jolt system: before the collision step it casts every wheel
    /// against the scene and applies gravity (or the override of [`VehicleMut::set_gravity`]),
    /// then engine, brakes and tire friction act through the constraint. No Rust code runs
    /// inside the step, and no vehicle can be read or changed during it, because the step takes
    /// `&mut PhysicsWorld`.
    ///
    /// The chassis stays an ordinary body owned by the caller: its pose and velocities come from
    /// [`body`](Self::body), and [`remove_body`](Self::remove_body) refuses it while the vehicle
    /// exists. For a vehicle under the caller's own (for example radial) gravity, create the
    /// chassis with [`allow_sleeping(false)`](crate::BodySettings::allow_sleeping) and
    /// [`gravity_factor(0.0)`](crate::BodySettings::gravity_factor), and set the gravity at the
    /// vehicle every tick.
    ///
    /// Fails with [`VehicleError::InvalidValue`] when a setting is out of range (see the setters
    /// of [`VehicleSettings`] and the types it holds), with [`VehicleError::Body`] when `body`
    /// is not in this world, is the inner body of a character, a ragdoll part or a soft body, or
    /// has fewer than six degrees of freedom, with
    /// [`VehicleError::NotDynamic`], with [`VehicleError::AlreadyHasVehicle`] when the body
    /// carries a vehicle already, and with [`VehicleError::TooManyVehicles`] when the world has
    /// run out of ids. Nothing is created on failure.
    ///
    /// # Example
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let floor = Shape::new_box(Vec3::new(50.0, 1.0, 50.0))?;
    /// world.create_body(&floor, &BodySettings::new_static().position(RVec3::new(0.0, -1.0, 0.0)))?;
    ///
    /// let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
    /// let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0))?;
    /// let chassis = world.create_body(
    ///     &chassis_shape,
    ///     &BodySettings::new_dynamic()
    ///         .position(RVec3::new(0.0, 1.0, 0.0))
    ///         .mass(1500.0)
    ///         .allow_sleeping(false),
    /// )?;
    /// let wheel = |x: f32, z: f32| {
    ///     WheelSettings::new(Vec3::new(x, -0.1, z)).radius(0.35).width(0.2)
    /// };
    /// let settings = VehicleSettings::new(
    ///     vec![wheel(0.9, 1.4), wheel(-0.9, 1.4), wheel(0.9, -1.4), wheel(-0.9, -1.4)],
    ///     vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
    ///     VehicleCollisionTester::ray(ObjectLayer::MOVING),
    /// );
    /// let car = world.create_vehicle(chassis, &settings)?;
    ///
    /// world.vehicle_mut(car)?.set_driver_input(DriverInput { forward: 1.0, ..DriverInput::default() })?;
    /// for _ in 0..60 {
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// let vehicle = world.vehicle(car)?;
    /// assert!(vehicle.wheels().iter().all(|wheel| wheel.contact.is_some()));
    /// assert!(world.body(chassis)?.position().z > 0.0);
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_vehicle(
        &mut self,
        body: BodyId,
        settings: &VehicleSettings,
    ) -> Result<VehicleId, VehicleError> {
        settings.validate(self.object_layer_count)?;
        self.check_chassis(body)?;
        let id = self.attach_vehicle(body, settings.build())?;
        Ok(VehicleId::new(id.raw, self.tag))
    }

    /// Checks that `body` can carry a vehicle: a dynamic rigid body of this world with all six
    /// degrees of freedom, not owned by a character or a ragdoll, without a vehicle yet.
    fn check_chassis(&self, body: BodyId) -> Result<(), VehicleError> {
        self.check(body).map_err(VehicleError::Body)?;
        if self.is_inner_body(body) {
            return Err(VehicleError::Body(BodyError::OwnedByCharacter(body)));
        }
        // A ragdoll part is destroyed with its ragdoll, which the vehicle would outlive.
        if self.is_ragdoll_body(body) {
            return Err(VehicleError::Body(BodyError::OwnedByRagdoll(body)));
        }
        // A vehicle is a constraint, and Jolt's constraints cannot operate on soft bodies
        // (`Docs/Architecture.md:462`).
        if self.body(body).map_err(VehicleError::Body)?.is_soft_body() {
            return Err(VehicleError::Body(BodyError::SoftBody(body)));
        }
        if self.body(body).map_err(VehicleError::Body)?.motion_type() != MotionType::Dynamic {
            return Err(VehicleError::NotDynamic(body));
        }
        // The wheel and gravity bounds assume an unmasked chassis.
        if self.body(body).map_err(VehicleError::Body)?.allowed_dofs() != AllowedDofs::ALL {
            return Err(VehicleError::Body(BodyError::RestrictedDofs(body)));
        }
        // One vehicle per chassis: two vehicles would write the same body's force accumulator
        // from different step listener jobs.
        if self.vehicle_bodies.contains_key(&body.to_raw()) {
            return Err(VehicleError::AlreadyHasVehicle(body));
        }
        Ok(())
    }

    /// Creates the vehicle constraint from `built` on `body`, which
    /// [`check_chassis`](Self::check_chassis) accepted, installs its tester and registers it with
    /// the system. Fails only when the world has run out of ids, before anything is created.
    fn attach_vehicle(
        &mut self,
        body: BodyId,
        built: BuiltSettings,
    ) -> Result<AnyVehicleId, VehicleError> {
        let raw = self.next_vehicle_id;
        if raw == u32::MAX {
            return Err(VehicleError::TooManyVehicles);
        }
        let mut wheel_pointers: Vec<*mut JPH_WheelSettings> =
            built.wheels.iter().map(|wheel| wheel.as_base()).collect();
        let kind = built.controller.kind();
        let jolt_settings = JPH_VehicleConstraintSettings {
            base: constraint_base(),
            up: built.frame.up.to_jph(),
            forward: built.frame.forward.to_jph(),
            maxPitchRollAngle: built.frame.max_pitch_roll_angle,
            // The settings' validation bounds the counts to `u32`.
            wheelsCount: wheel_pointers.len() as u32,
            wheels: wheel_pointers.as_mut_ptr(),
            antiRollBarsCount: built.anti_roll_bars.len() as u32,
            antiRollBars: built.anti_roll_bars.as_ptr(),
            controller: built.controller.as_base(),
        };
        let constraint = with_locked_body(self.body_lock_interface, body, |chassis| {
            // SAFETY: the chassis is locked for writing and dynamic. The constructor stores the
            // body pointer and reads its id; Jolt bodies stay at their address until destroyed,
            // and `remove_body` refuses a chassis while the vehicle exists. The settings, the
            // wheel and controller settings (owned by `built`, which lives to the end of this
            // function) and the anti-roll bars are live and validated; the wheels are of the kind
            // the controller expects. The constraint keeps its own references to the wheel
            // settings and copies the rest. The handle takes over the one reference joltc
            // returns.
            unsafe {
                Owned::from_raw(JPH_VehicleConstraint_Create(
                    chassis.as_ptr(),
                    &jolt_settings,
                ))
            }
        })
        .unwrap_or_else(|| unreachable!("`check_chassis` found the body and `&mut self` keeps it"))
        .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
        let mut entry = VehicleEntry {
            constraint,
            body,
            collision_tester: built.collision_tester,
            wheels: built.geometry,
            kind,
            tracks: built.tracks,
        };
        install_tester(&mut entry, built.collision_tester);
        self.note_structure_change();
        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs. The system takes its own reference as a constraint and keeps a pointer as a
        // step listener; `remove_vehicle` and `remove_all_vehicles` take the vehicle out of both
        // before the world releases its reference. The tester is set, as the first step needs.
        unsafe {
            let constraint = entry.constraint.as_ptr();
            JPH_PhysicsSystem_AddConstraint(
                self.system.as_ptr(),
                JPH_VehicleConstraint_AsConstraint(constraint),
            );
            JPH_PhysicsSystem_AddStepListener(
                self.system.as_ptr(),
                JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
            );
        }
        self.vehicles.insert(raw, entry);
        self.vehicle_bodies.insert(body.to_raw(), raw);
        self.next_vehicle_id += 1;
        Ok(AnyVehicleId {
            raw,
            world: self.tag,
            kind,
        })
    }

    /// The entry of `id`, if it names a vehicle of this world.
    fn vehicle_entry(&self, id: AnyVehicleId) -> Result<&VehicleEntry, VehicleError> {
        if id.world != self.tag {
            return Err(VehicleError::WrongWorld(id));
        }
        let entry = self
            .vehicles
            .get(&id.raw)
            .ok_or(VehicleError::NotFound(id))?;
        // Ids are typed with the kind the world created them with.
        debug_assert_eq!(entry.kind, id.kind, "a vehicle id has its vehicle's kind");
        Ok(entry)
    }

    /// Removes a vehicle of any kind. Its chassis body stays in the world, with whatever gravity
    /// factor the vehicle left it.
    pub fn remove_vehicle(&mut self, id: impl Into<AnyVehicleId>) -> Result<(), VehicleError> {
        let id = id.into();
        self.vehicle_entry(id)?;
        self.note_structure_change();
        let entry = self
            .vehicles
            .remove(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        self.vehicle_bodies.remove(&entry.body.to_raw());
        self.unregister_vehicle(entry);
        Ok(())
    }

    /// Takes the vehicle out of the system's step listeners and constraints, then releases the
    /// world's reference.
    fn unregister_vehicle(&mut self, entry: VehicleEntry) {
        // SAFETY: the system and the constraint are live, the system is borrowed mutably and no
        // step runs; the vehicle was registered in both lists by `create_vehicle`.
        unsafe {
            let constraint = entry.constraint.as_ptr();
            JPH_PhysicsSystem_RemoveStepListener(
                self.system.as_ptr(),
                JPH_VehicleConstraint_AsPhysicsStepListener(constraint),
            );
            JPH_PhysicsSystem_RemoveConstraint(
                self.system.as_ptr(),
                JPH_VehicleConstraint_AsConstraint(constraint),
            );
        }
        drop(entry);
    }

    /// Removes every vehicle, in id order. Runs when the world is dropped, so no vehicle is
    /// released while the system still lists it.
    pub(crate) fn remove_all_vehicles(&mut self) {
        while let Some((_, entry)) = self.vehicles.pop_first() {
            self.vehicle_bodies.remove(&entry.body.to_raw());
            self.unregister_vehicle(entry);
        }
    }

    /// Read access to a vehicle.
    pub fn vehicle<K: VehicleKind>(
        &self,
        id: VehicleId<K>,
    ) -> Result<VehicleRef<'_, K>, VehicleError> {
        let entry = self.vehicle_entry(id.into())?;
        Ok(VehicleRef {
            world: self,
            id,
            entry,
        })
    }

    /// Write access to a vehicle.
    pub fn vehicle_mut<K: VehicleKind>(
        &mut self,
        id: VehicleId<K>,
    ) -> Result<VehicleMut<'_, K>, VehicleError> {
        self.vehicle_entry(id.into())?;
        let object_layer_count = self.object_layer_count;
        let entry = self
            .vehicles
            .get_mut(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        Ok(VehicleMut {
            entry,
            object_layer_count,
            kind: PhantomData,
        })
    }

    /// The ids of the world's vehicles of every kind, in id order (creation order).
    pub fn vehicle_ids(&self) -> impl Iterator<Item = AnyVehicleId> + '_ {
        self.vehicles.iter().map(|(&raw, entry)| AnyVehicleId {
            raw,
            world: self.tag,
            kind: entry.kind,
        })
    }

    /// The vehicle whose chassis is `body`, if any.
    pub fn vehicle_of_body(&self, body: BodyId) -> Option<AnyVehicleId> {
        if body.world != self.tag {
            return None;
        }
        let raw = *self.vehicle_bodies.get(&body.to_raw())?;
        let entry = self
            .vehicles
            .get(&raw)
            .unwrap_or_else(|| unreachable!("every chassis names a vehicle"));
        Some(AnyVehicleId {
            raw,
            world: self.tag,
            kind: entry.kind,
        })
    }

    /// Whether `body` is the chassis of a vehicle of this world.
    pub(crate) fn is_vehicle_body(&self, body: BodyId) -> bool {
        self.vehicle_of_body(body).is_some()
    }
}

/// A vehicle's gravity override and collision tester in the new frame of a rotating
/// [`PhysicsWorld::rebase`], computed before the rebase writes anything.
pub(crate) struct VehicleRebase {
    raw: u32,
    gravity: Option<Vec3>,
    collision_tester: VehicleCollisionTester,
}

impl PhysicsWorld {
    /// Every vehicle's gravity override and tester rotated by `rotate`, in id order; an error
    /// names the value that would not be valid in the new frame.
    pub(crate) fn rotated_vehicles(
        &self,
        rotate: impl Fn(Vec3) -> Vec3,
    ) -> Result<Vec<VehicleRebase>, &'static str> {
        let mut rebased = Vec::with_capacity(self.vehicles.len());
        for (&raw, entry) in &self.vehicles {
            // A rotation keeps the override's length, which `set_gravity` bounds.
            let gravity = gravity_override(entry).map(&rotate);
            let collision_tester = match entry.collision_tester.up() {
                Some(up) => {
                    let up = rotate(up).normalized_or_zero();
                    if !is_unit(up) {
                        return Err("rebase would give a vehicle tester an up that is not unit");
                    }
                    entry.collision_tester.with_up(up)
                }
                None => entry.collision_tester,
            };
            rebased.push(VehicleRebase {
                raw,
                gravity,
                collision_tester,
            });
        }
        Ok(rebased)
    }

    /// Writes what [`rotated_vehicles`](Self::rotated_vehicles) computed.
    pub(crate) fn apply_vehicle_rebase(&mut self, rebased: Vec<VehicleRebase>) {
        for vehicle in rebased {
            let entry = self
                .vehicles
                .get_mut(&vehicle.raw)
                .unwrap_or_else(|| unreachable!("computed from this world's vehicles"));
            if let Some(gravity) = vehicle.gravity {
                let gravity = gravity.to_jph();
                // SAFETY: the world is borrowed mutably and owns the constraint; no step runs.
                // `gravity` is a live local.
                unsafe {
                    JPH_VehicleConstraint_OverrideGravity(entry.constraint.as_ptr(), &gravity)
                };
            }
            if vehicle.collision_tester != entry.collision_tester {
                install_tester(entry, vehicle.collision_tester);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorldSettings;

    #[test]
    fn listener_batching_matches_the_fleet_gate() {
        let world = PhysicsWorld::new(WorldSettings::default()).unwrap();
        // SAFETY: an all-zero `JPH_PhysicsSettings` is valid: integers, floats and `false`.
        let mut settings: JPH_PhysicsSettings = unsafe { std::mem::zeroed() };
        // SAFETY: the system is live; `settings` is a live local that joltc fills.
        unsafe { JPH_PhysicsSystem_GetPhysicsSettings(world.system.as_ptr(), &mut settings) };
        // The vehicle determinism gate sizes its fleet from these: Jolt runs step listeners in
        // `listeners / batch size / batches per job` jobs, capped by the job system.
        assert_eq!(settings.stepListenersBatchSize, 8);
        assert_eq!(settings.stepListenerBatchesPerJob, 1);
    }
}
