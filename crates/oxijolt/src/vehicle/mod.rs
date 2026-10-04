//! Wheeled vehicles: Jolt's `VehicleConstraint` with the wheeled controller, owned by a
//! [`PhysicsWorld`] and attached to a dynamic chassis body the caller created.
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

mod settings;

use std::fmt;

use oxijolt_sys::*;

use crate::body::with_locked_body;
use crate::constraint::constraint_base;
use crate::limits;
use crate::math::is_unit;
use crate::owned::{JoltObject, Owned};
use crate::world::WorldTag;
use crate::{BodyError, BodyId, MotionType, PhysicsWorld, RVec3, SubShapeId, Vec3, VehicleError};

pub use settings::{
    SuspensionSpring, VehicleAntiRollBar, VehicleCollisionTester, VehicleDifferentialSettings,
    VehicleEngineSettings, VehicleSettings, VehicleTransmissionSettings, WheelSettings,
    DEFAULT_LATERAL_FRICTION, DEFAULT_LONGITUDINAL_FRICTION, DEFAULT_NORMALIZED_TORQUE,
};
use settings::{WheelGeometry, MAX_PITCH_ROLL_RULE};

/// Jolt's invalid `BodyID` value.
const INVALID_BODY_ID: u32 = 0xffff_ffff;

/// Identifies a vehicle in the world that created it.
///
/// The raw value is 1 for the first vehicle of a world, then 2, 3 and so on; ids are never
/// reused within a world, so the same creation history gives the same ids. Using an id with
/// another world returns [`VehicleError::WrongWorld`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct VehicleId {
    // Declared first, so ids order by creation before the world.
    raw: u32,
    pub(crate) world: WorldTag,
}

impl VehicleId {
    pub(crate) fn new(raw: u32, world: WorldTag) -> Self {
        Self { raw, world }
    }

    /// The id's number within its world.
    pub fn to_raw(self) -> u32 {
        self.raw
    }
}

impl fmt::Debug for VehicleId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_tuple("VehicleId").field(&self.raw).finish()
    }
}

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
}

/// What the driver asks of a vehicle (Jolt `WheeledVehicleController::SetDriverInput`).
///
/// The default is no input: no throttle, straight ahead, no brakes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DriverInput {
    /// Throttle in `[-1, 1]`; negative drives backwards. Pressing against the direction of
    /// travel brakes first.
    pub forward: f32,
    /// Steering in `[-1, 1]`; 1 steers fully right, −1 fully left.
    pub right: f32,
    /// Brake in `[0, 1]`.
    pub brake: f32,
    /// Hand brake in `[0, 1]`.
    pub hand_brake: f32,
}

impl DriverInput {
    fn validate(&self) -> Result<(), VehicleError> {
        let within = |value: f32, low: f32| value.is_finite() && (low..=1.0).contains(&value);
        let valid = within(self.forward, -1.0)
            && within(self.right, -1.0)
            && within(self.brake, 0.0)
            && within(self.hand_brake, 0.0);
        if valid {
            Ok(())
        } else {
            Err(VehicleError::InvalidValue(
                "forward and right must be in [-1, 1], brake and hand brake in [0, 1]",
            ))
        }
    }
}

/// Where a wheel touches the ground.
///
/// Computed by the wheel test at the start of the last step, at the chassis pose before that
/// step moved it; all values are in world space. Normals are the ground's outward surface
/// normal, pointing toward the wheel, the convention of the scene queries.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct WheelContact {
    /// The body under the wheel. It may have been removed since the step.
    pub body: BodyId,
    /// The leaf of that body's shape under the wheel.
    pub sub_shape_id: SubShapeId,
    /// The contact point, metres.
    pub position: RVec3,
    /// The ground's surface normal at the contact, a unit vector.
    pub normal: Vec3,
    /// Velocity of the ground at the contact point, m/s.
    pub point_velocity: Vec3,
    /// Direction along the wheel's rolling direction on the ground, a unit vector.
    pub longitudinal: Vec3,
    /// Sideways direction of the wheel on the ground, a unit vector.
    pub lateral: Vec3,
}

/// The state of one wheel after the last step.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct WheelState {
    /// The ground contact, `None` while the wheel hangs free.
    pub contact: Option<WheelContact>,
    /// Suspension length from the attachment point, metres: the maximum length without
    /// contact. A wheel whose test started inside the ground reports 0 and
    /// [`hit_hard_point`](Self::hit_hard_point), never a penetration depth; for that depth run
    /// [`PhysicsWorld::collide_shape`] with the tester's object layer.
    pub suspension_length: f32,
    /// Whether the suspension is pressed past its minimum length onto its hard stop.
    pub hit_hard_point: bool,
    /// Rotation speed of the wheel, rad/s; positive when it rolls the vehicle forward.
    pub angular_velocity: f32,
    /// Rotation angle of the wheel, radians. Jolt wraps it with `fmod` by 2π, so while the angle
    /// and angular velocity are finite it stays within `(−2π, 2π)`; it goes negative when the
    /// wheel turns backwards past 0.
    pub rotation_angle: f32,
    /// Steering angle, radians; positive steers left.
    pub steer_angle: f32,
    /// Impulse the suspension applied in the last step, N·s.
    pub suspension_lambda: f32,
    /// Impulse the tire applied along its rolling direction in the last step, N·s.
    pub longitudinal_lambda: f32,
    /// Impulse the tire applied sideways in the last step, N·s.
    pub lateral_lambda: f32,
}

/// Read access to one vehicle, borrowed from its world.
///
/// Each method reads the state the last step left. The chassis pose and velocities come from
/// [`PhysicsWorld::body`] with [`body`](Self::body).
pub struct VehicleRef<'w> {
    world: &'w PhysicsWorld,
    id: VehicleId,
    entry: &'w VehicleEntry,
}

impl VehicleRef<'_> {
    fn ptr(&self) -> *mut JPH_VehicleConstraint {
        self.entry.constraint.as_ptr()
    }

    fn controller(&self) -> *mut JPH_WheeledVehicleController {
        // SAFETY: the world borrowed here owns the constraint, and changing it needs
        // `&mut PhysicsWorld`; the getter returns a member. The world creates only wheeled
        // controllers, which derive from `VehicleController` with single inheritance.
        unsafe { JPH_VehicleConstraint_GetController(self.ptr()) }.cast()
    }

    /// The vehicle's id.
    pub fn id(&self) -> VehicleId {
        self.id
    }

    /// The chassis body.
    pub fn body(&self) -> BodyId {
        self.entry.body
    }

    /// Number of wheels.
    pub fn wheel_count(&self) -> u32 {
        self.entry.wheels.len() as u32
    }

    /// The state of wheel `index`, `None` when there is no such wheel.
    pub fn wheel(&self, index: u32) -> Option<WheelState> {
        if index >= self.wheel_count() {
            return None;
        }
        // SAFETY: the world borrowed here owns the constraint; `index` is in range, as Jolt
        // asserts. The wheel lives as long as the constraint.
        let wheel = unsafe { JPH_VehicleConstraint_GetWheel(self.ptr(), index) };
        // SAFETY: the wheel is live and only read; its contact getters run only when
        // `HasContact` holds, as Jolt asserts. Every output is a live local.
        unsafe {
            let contact = JPH_Wheel_HasContact(wheel).then(|| {
                let mut position = RVec3::ZERO.to_jph();
                let mut normal = Vec3::ZERO.to_jph();
                let mut point_velocity = Vec3::ZERO.to_jph();
                let mut longitudinal = Vec3::ZERO.to_jph();
                let mut lateral = Vec3::ZERO.to_jph();
                JPH_Wheel_GetContactPosition(wheel, &mut position);
                JPH_Wheel_GetContactNormal(wheel, &mut normal);
                JPH_Wheel_GetContactPointVelocity(wheel, &mut point_velocity);
                JPH_Wheel_GetContactLongitudinal(wheel, &mut longitudinal);
                JPH_Wheel_GetContactLateral(wheel, &mut lateral);
                let body = JPH_Wheel_GetContactBodyID(wheel);
                debug_assert_ne!(body, INVALID_BODY_ID, "a contact names a body");
                WheelContact {
                    body: BodyId::new(body, self.world.tag),
                    sub_shape_id: SubShapeId::new(JPH_Wheel_GetContactSubShapeID(wheel)),
                    position: RVec3::from_jph(position),
                    normal: Vec3::from_jph(normal),
                    point_velocity: Vec3::from_jph(point_velocity),
                    longitudinal: Vec3::from_jph(longitudinal),
                    lateral: Vec3::from_jph(lateral),
                }
            });
            Some(WheelState {
                contact,
                suspension_length: JPH_Wheel_GetSuspensionLength(wheel),
                hit_hard_point: JPH_Wheel_HasHitHardPoint(wheel),
                angular_velocity: JPH_Wheel_GetAngularVelocity(wheel),
                rotation_angle: JPH_Wheel_GetRotationAngle(wheel),
                steer_angle: JPH_Wheel_GetSteerAngle(wheel),
                suspension_lambda: JPH_Wheel_GetSuspensionLambda(wheel),
                longitudinal_lambda: JPH_Wheel_GetLongitudinalLambda(wheel),
                lateral_lambda: JPH_Wheel_GetLateralLambda(wheel),
            })
        }
    }

    /// The states of all wheels, in wheel order.
    pub fn wheels(&self) -> Vec<WheelState> {
        (0..self.wheel_count())
            .filter_map(|index| self.wheel(index))
            .collect()
    }

    /// The driver input last set.
    pub fn driver_input(&self) -> DriverInput {
        let controller = self.controller();
        // SAFETY: the controller is live and owned by the constraint; the getters read members.
        unsafe {
            DriverInput {
                forward: JPH_WheeledVehicleController_GetForwardInput(controller),
                right: JPH_WheeledVehicleController_GetRightInput(controller),
                brake: JPH_WheeledVehicleController_GetBrakeInput(controller),
                hand_brake: JPH_WheeledVehicleController_GetHandBrakeInput(controller),
            }
        }
    }

    /// The gravity override of [`VehicleMut::set_gravity`], m/s², or `None` while the vehicle
    /// uses the world's gravity.
    pub fn gravity(&self) -> Option<Vec3> {
        // SAFETY: as in `controller`; the getters read members, and `value` is a live local.
        unsafe {
            JPH_VehicleConstraint_IsGravityOverridden(self.ptr()).then(|| {
                let mut value = Vec3::ZERO.to_jph();
                JPH_VehicleConstraint_GetGravityOverride(self.ptr(), &mut value);
                Vec3::from_jph(value)
            })
        }
    }

    /// The world up of the last step: the opposite of the gravity the vehicle used, normalized.
    /// The pitch and roll limit keeps the vehicle's up within its angle of this direction. A step
    /// in zero gravity keeps the previous world up; it is not part of
    /// [`WorldState`](crate::WorldState).
    pub fn world_up(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: as in `gravity`.
        unsafe { JPH_VehicleConstraint_GetWorldUp(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Engine speed, rpm.
    pub fn engine_rpm(&self) -> f32 {
        // SAFETY: the controller is live and owned by the constraint; the engine is a member of
        // it, read only.
        unsafe {
            let engine = JPH_WheeledVehicleController_GetEngine(self.controller());
            JPH_VehicleEngine_GetCurrentRPM(engine)
        }
    }

    /// Current gear: −1 reverse, 0 neutral, 1 first gear and so on.
    pub fn current_gear(&self) -> i32 {
        // SAFETY: as in `engine_rpm`, for the transmission.
        unsafe {
            let transmission = JPH_WheeledVehicleController_GetTransmission(self.controller());
            JPH_VehicleTransmission_GetCurrentGear(transmission)
        }
    }

    /// The collision tester the wheels use.
    pub fn collision_tester(&self) -> &VehicleCollisionTester {
        &self.entry.collision_tester
    }
}

/// Write access to one vehicle, borrowed mutably from its world.
///
/// Read the vehicle through [`PhysicsWorld::vehicle`] once this borrow ends.
pub struct VehicleMut<'w> {
    entry: &'w mut VehicleEntry,
    object_layer_count: u32,
}

impl VehicleMut<'_> {
    fn ptr(&self) -> *mut JPH_VehicleConstraint {
        self.entry.constraint.as_ptr()
    }

    /// Sets what the driver asks for the next steps; see [`DriverInput`] for the ranges.
    pub fn set_driver_input(&mut self, input: DriverInput) -> Result<(), VehicleError> {
        input.validate()?;
        // SAFETY: the world is borrowed mutably through this view and owns the constraint. The
        // world creates only wheeled controllers, which derive from `VehicleController` with
        // single inheritance; the setter writes members.
        unsafe {
            let controller: *mut JPH_WheeledVehicleController =
                JPH_VehicleConstraint_GetController(self.ptr()).cast();
            JPH_WheeledVehicleController_SetDriverInput(
                controller,
                input.forward,
                input.right,
                input.brake,
                input.hand_brake,
            );
        }
        Ok(())
    }

    /// Replaces the world's gravity for this vehicle by `gravity`, m/s² (Jolt
    /// `VehicleConstraint::OverrideGravity`). It must be finite and at most
    /// [`limits::MAX_ACCELERATION`] long. The chassis is a dynamic body, so its mass is at most
    /// [`limits::MAX_MASS`] and the force Jolt derives from the override stays finite.
    ///
    /// On every step while the chassis is awake, Jolt sets the chassis' gravity factor to 0 and
    /// adds the force `gravity / inverse mass` at its centre of mass; a sleeping chassis gets no
    /// force. The opposite of `gravity` also becomes the world up of the pitch and roll limit;
    /// a zero `gravity` keeps the last world up, which a restore does not undo (see
    /// [`WorldState`](crate::WorldState)).
    /// The override stays until it is set again; there is no reset, because Jolt's reset writes
    /// gravity factor 1 to the chassis. For radial gravity, set it every tick.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_gravity(&mut self, gravity: Vec3) -> Result<(), VehicleError> {
        if !limits::is_acceleration(gravity) {
            return Err(VehicleError::InvalidValue(limits::GRAVITY_RULE));
        }
        let gravity = gravity.to_jph();
        // SAFETY: as in `set_driver_input`; `gravity` is a live local.
        unsafe { JPH_VehicleConstraint_OverrideGravity(self.ptr(), &gravity) };
        Ok(())
    }

    /// Sets the largest pitch and roll angle, radians in `[0, π]`; π turns the limit off. See
    /// [`VehicleSettings::max_pitch_roll_angle`].
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_max_pitch_roll_angle(&mut self, radians: f32) -> Result<(), VehicleError> {
        if !(radians.is_finite() && (0.0..=std::f32::consts::PI).contains(&radians)) {
            return Err(VehicleError::InvalidValue(MAX_PITCH_ROLL_RULE));
        }
        // SAFETY: as in `set_driver_input`.
        unsafe { JPH_VehicleConstraint_SetMaxPitchRollAngle(self.ptr(), radians) };
        Ok(())
    }

    /// Replaces the collision tester, checked against this vehicle's wheels as at creation. The
    /// next step tests the wheels with it.
    /// Not part of [`WorldState`](crate::WorldState):
    /// [`PhysicsWorld::restore_state`](crate::PhysicsWorld::restore_state) does not undo it.
    pub fn set_collision_tester(
        &mut self,
        tester: VehicleCollisionTester,
    ) -> Result<(), VehicleError> {
        tester.validate(self.object_layer_count, self.entry.wheels.iter().copied())?;
        install_tester(self.entry, tester);
        Ok(())
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
    /// is not in this world, is the inner body of a character, a ragdoll part or a soft body, with
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
    ///     world.step(1.0 / 60.0)?;
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
        // One vehicle per chassis: two vehicles would write the same body's force accumulator
        // from different step listener jobs.
        if self.vehicle_bodies.contains_key(&body.to_raw()) {
            return Err(VehicleError::AlreadyHasVehicle(body));
        }
        let raw = self.next_vehicle_id;
        if raw == u32::MAX {
            return Err(VehicleError::TooManyVehicles);
        }

        let wheels = settings.create_wheels();
        let mut wheel_pointers: Vec<*mut JPH_WheelSettings> =
            wheels.iter().map(|wheel| wheel.as_ptr().cast()).collect();
        let controller = settings.create_controller();
        let anti_roll_bars = settings.anti_roll_bars_to_jph();
        let jolt_settings = JPH_VehicleConstraintSettings {
            base: constraint_base(),
            up: settings.up_to_jph(),
            forward: settings.forward_to_jph(),
            maxPitchRollAngle: settings.max_pitch_roll_angle_value(),
            // `validate` bounds the counts to `u32`.
            wheelsCount: wheel_pointers.len() as u32,
            wheels: wheel_pointers.as_mut_ptr(),
            antiRollBarsCount: anti_roll_bars.len() as u32,
            antiRollBars: anti_roll_bars.as_ptr(),
            controller: controller.as_ptr().cast(),
        };
        let constraint = with_locked_body(self.body_lock_interface, body, |chassis| {
            // SAFETY: the chassis is locked for writing and dynamic. The constructor stores the
            // body pointer and reads its id; Jolt bodies stay at their address until destroyed,
            // and `remove_body` refuses a chassis while the vehicle exists. The settings, the
            // wheel and controller settings and the anti-roll bars are live and validated; the
            // constraint keeps its own references to the wheel settings and copies the rest.
            // The handle takes over the one reference joltc returns.
            unsafe {
                Owned::from_raw(JPH_VehicleConstraint_Create(
                    chassis.as_ptr(),
                    &jolt_settings,
                ))
            }
        })
        .unwrap_or_else(|| unreachable!("`check` found the body and `&mut self` keeps it"))
        .unwrap_or_else(|| unreachable!("joltc `new`s the constraint"));
        let mut entry = VehicleEntry {
            constraint,
            body,
            collision_tester: settings.collision_tester,
            wheels: settings.wheels.iter().map(WheelGeometry::of).collect(),
        };
        install_tester(&mut entry, settings.collision_tester);
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
        Ok(VehicleId::new(raw, self.tag))
    }

    /// The entry of `id`, if it names a vehicle of this world.
    fn vehicle_entry(&self, id: VehicleId) -> Result<&VehicleEntry, VehicleError> {
        if id.world != self.tag {
            return Err(VehicleError::WrongWorld(id));
        }
        self.vehicles.get(&id.raw).ok_or(VehicleError::NotFound(id))
    }

    /// Removes a vehicle. Its chassis body stays in the world, with whatever gravity factor the
    /// vehicle left it.
    pub fn remove_vehicle(&mut self, id: VehicleId) -> Result<(), VehicleError> {
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
    pub fn vehicle(&self, id: VehicleId) -> Result<VehicleRef<'_>, VehicleError> {
        let entry = self.vehicle_entry(id)?;
        Ok(VehicleRef {
            world: self,
            id,
            entry,
        })
    }

    /// Write access to a vehicle.
    pub fn vehicle_mut(&mut self, id: VehicleId) -> Result<VehicleMut<'_>, VehicleError> {
        self.vehicle_entry(id)?;
        let object_layer_count = self.object_layer_count;
        let entry = self
            .vehicles
            .get_mut(&id.raw)
            .unwrap_or_else(|| unreachable!("checked above"));
        Ok(VehicleMut {
            entry,
            object_layer_count,
        })
    }

    /// The ids of the world's vehicles, in id order (creation order).
    pub fn vehicle_ids(&self) -> impl Iterator<Item = VehicleId> + '_ {
        self.vehicles
            .keys()
            .map(|&raw| VehicleId::new(raw, self.tag))
    }

    /// The vehicle whose chassis is `body`, if any.
    pub fn vehicle_of_body(&self, body: BodyId) -> Option<VehicleId> {
        if body.world != self.tag {
            return None;
        }
        self.vehicle_bodies
            .get(&body.to_raw())
            .map(|&raw| VehicleId::new(raw, self.tag))
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
            let vehicle = VehicleRef {
                world: self,
                id: VehicleId::new(raw, self.tag),
                entry,
            };
            // A rotation keeps the override's length, which `set_gravity` bounds.
            let gravity = vehicle.gravity().map(&rotate);
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
