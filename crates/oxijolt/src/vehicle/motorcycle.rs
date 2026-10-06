//! Motorcycles: Jolt's `VehicleConstraint` with the motorcycle controller.

use oxijolt_sys::*;

use super::{DriverInput, Motorcycle, MotorcycleSettings, VehicleId};
use crate::body::with_read_locked_body;
use crate::{limits, BodyId, PhysicsWorld, Quat, Vec3, VehicleError, VehicleMut, VehicleRef};

/// A motorcycle's lean after the last step.
///
/// Angles are in radians about the chassis' forward, measured from the world up of the last
/// step ([`VehicleRef::world_up`]); positive leans to the right. An angle is 0 when its
/// direction or the world up lies along the forward axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct MotorcycleLean {
    /// The direction the lean controller steers the chassis' up toward, in world space (Jolt's
    /// target lean, part of [`WorldState`](crate::WorldState)): the zero vector until the first
    /// step, the world up after a step with the lean controller off.
    pub target: Vec3,
    /// The lean angle of [`target`](Self::target).
    pub target_angle: f32,
    /// The lean angle of the chassis' up.
    pub angle: f32,
}

impl VehicleRef<'_, Motorcycle> {
    fn motorcycle_controller(&self) -> *mut JPH_MotorcycleController {
        // A motorcycle's controller is a `MotorcycleController`, which derives from
        // `VehicleController` with single inheritance.
        self.controller().cast()
    }

    /// The driver input last set.
    pub fn driver_input(&self) -> DriverInput {
        // SAFETY: a `MotorcycleController` is a `WheeledVehicleController` with single
        // inheritance; the world borrowed here keeps it alive.
        unsafe { DriverInput::of(self.motorcycle_controller().cast()) }
    }

    /// The lean controller's target and the lean of the chassis.
    pub fn lean(&self) -> MotorcycleLean {
        let target = target_lean(self.controller());
        let constraint = self.ptr();
        let mut local_up = Vec3::ZERO.to_jph();
        let mut local_forward = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getters read members into
        // live locals.
        unsafe {
            JPH_VehicleConstraint_GetLocalUp(constraint, &mut local_up);
            JPH_VehicleConstraint_GetLocalForward(constraint, &mut local_forward);
        }
        let rotation = self
            .world
            .body(self.entry.body)
            .unwrap_or_else(|_| {
                unreachable!("the chassis cannot be removed while the vehicle exists")
            })
            .rotation();
        let forward = rotate(rotation, wide(Vec3::from_jph(local_forward)));
        let up = rotate(rotation, wide(Vec3::from_jph(local_up)));
        let world_up = wide(self.world_up());
        MotorcycleLean {
            target,
            target_angle: lean_angle(world_up, wide(target), forward),
            angle: lean_angle(world_up, up, forward),
        }
    }

    /// Whether the lean controller runs; see
    /// [`MotorcycleSettings::lean_controller`].
    pub fn is_lean_controller_enabled(&self) -> bool {
        // SAFETY: as in `driver_input`; the getter reads a member.
        unsafe { JPH_MotorcycleController_IsLeanControllerEnabled(self.motorcycle_controller()) }
    }

    /// Whether the steering angle is limited by speed; see
    /// [`MotorcycleSettings::lean_steering_limit`].
    pub fn is_lean_steering_limit_enabled(&self) -> bool {
        // SAFETY: as in `driver_input`; the getter reads a member.
        unsafe { JPH_MotorcycleController_IsLeanSteeringLimitEnabled(self.motorcycle_controller()) }
    }
}

impl VehicleMut<'_, Motorcycle> {
    /// Sets what the driver asks for the next steps; see [`DriverInput`] for the ranges.
    pub fn set_driver_input(&mut self, input: DriverInput) -> Result<(), VehicleError> {
        input.validate()?;
        // SAFETY: a `MotorcycleController` is a `WheeledVehicleController` with single
        // inheritance; the world is borrowed mutably through this view.
        unsafe { input.apply(self.controller().cast()) };
        Ok(())
    }
}

/// The target lean of a motorcycle's controller.
pub(super) fn target_lean(controller: *mut JPH_VehicleController) -> Vec3 {
    let mut value = Vec3::ZERO.to_jph();
    // SAFETY: the caller passes the live controller of a vehicle its world owns; the extension
    // checks the controller's kind and writes `value`, a live local, only for a motorcycle.
    let found = unsafe { JPH_MotorcycleController_GetTargetLean(controller, &mut value) };
    assert!(found, "a motorcycle's controller is a MotorcycleController");
    Vec3::from_jph(value)
}

/// Replaces the target lean of a motorcycle's controller.
///
/// # Safety
/// `controller` is the live controller of a motorcycle, which nothing else uses during the
/// call, and no step runs.
pub(super) unsafe fn set_target_lean(controller: *mut JPH_VehicleController, value: Vec3) {
    let value = value.to_jph();
    // SAFETY: the controller is live and unshared (contract); `value` is a live local.
    let stored = unsafe { JPH_MotorcycleController_SetTargetLean(controller, &value) };
    assert!(
        stored,
        "a motorcycle's controller is a MotorcycleController"
    );
}

type Wide = [f64; 3];

fn wide(v: Vec3) -> Wide {
    <[f32; 3]>::from(v).map(f64::from)
}

fn dot(a: Wide, b: Wide) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Wide, b: Wide) -> Wide {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `v` rotated by the unit quaternion `q`, in `f64`.
fn rotate(q: Quat, v: Wide) -> Wide {
    let [x, y, z, w] = <[f32; 4]>::from(q).map(f64::from);
    let u = [x, y, z];
    let t = cross(u, v).map(|c| 2.0 * c);
    let ut = cross(u, t);
    std::array::from_fn(|i| v[i] + w * t[i] + ut[i])
}

/// `v` without its component along the unit `axis`, normalised, or `None` when nothing is left.
fn perpendicular(v: Wide, axis: Wide) -> Option<Wide> {
    let along = dot(v, axis);
    let rest: Wide = std::array::from_fn(|i| v[i] - along * axis[i]);
    let length = dot(rest, rest).sqrt();
    (length > 0.0 && length.is_finite()).then(|| rest.map(|c| c / length))
}

/// The angle about `forward` from `world_up` to `direction`, both projected off `forward`;
/// positive leans right (Jolt's sign of the lean angle). 0 when a projection is empty.
fn lean_angle(world_up: Wide, direction: Wide, forward: Wide) -> f32 {
    let length = dot(forward, forward).sqrt();
    let forward = forward.map(|c| c / length);
    match (
        perpendicular(world_up, forward),
        perpendicular(direction, forward),
    ) {
        (Some(up), Some(direction)) => {
            dot(cross(up, direction), forward).atan2(dot(up, direction)) as f32
        }
        _ => 0.0,
    }
}

impl PhysicsWorld {
    /// Attaches a motorcycle to the dynamic body `body`, its chassis, and returns its id.
    ///
    /// The chassis works as for [`create_wheeled_vehicle`](Self::create_wheeled_vehicle), with the same rules
    /// and errors. The settings are checked as their setters state, and the lean spring against
    /// this chassis' largest principal inverse inertia: [`VehicleError::InvalidValue`] when it
    /// could exceed [`limits::MAX_ANGULAR_ACCELERATION`], and
    /// [`VehicleError::LeanSpringIntegrationNotSaved`] for an integration coefficient other than
    /// 0. Nothing is created on failure.
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
    /// let frame = Shape::new_box(Vec3::new(0.2, 0.3, 0.4))?;
    /// let chassis_shape = Shape::new_offset_center_of_mass(&frame, Vec3::new(0.0, -0.3, 0.0))?;
    /// let chassis = world.create_body(
    ///     &chassis_shape,
    ///     &BodySettings::new_dynamic()
    ///         .position(RVec3::new(0.0, 1.0, 0.0))
    ///         .mass(240.0)
    ///         .allow_sleeping(false),
    /// )?;
    /// let wheel = |z: f32| WheelSettings::new(Vec3::new(0.0, -0.27, z)).radius(0.31).width(0.05);
    /// let front = wheel(0.75).max_steer_angle(30.0_f32.to_radians());
    /// let rear = wheel(-0.75).max_steer_angle(0.0);
    /// let bike = WheeledVehicleSettings::new(
    ///     vec![front, rear],
    ///     vec![VehicleDifferentialSettings::new(None, Some(1))],
    ///     VehicleCollisionTester::cast_cylinder(ObjectLayer::MOVING),
    /// );
    /// let motorcycle = world.create_motorcycle(chassis, &MotorcycleSettings::new(bike))?;
    ///
    /// world
    ///     .vehicle_mut(motorcycle)?
    ///     .set_driver_input(DriverInput { forward: 0.4, ..DriverInput::default() })?;
    /// for _ in 0..90 {
    ///     assert!(world.step(1.0 / 60.0)?.is_complete());
    /// }
    /// assert!(world.body(chassis)?.position().z > 1.0);
    /// assert!(world.vehicle(motorcycle)?.lean().angle.abs() < 0.1);
    /// # Ok(())
    /// # }
    /// ```
    pub fn create_motorcycle(
        &mut self,
        body: BodyId,
        settings: &MotorcycleSettings,
    ) -> Result<VehicleId<Motorcycle>, VehicleError> {
        settings.validate(self.object_layer_count)?;
        self.check_chassis(body)?;
        let largest_inverse_inertia =
            with_read_locked_body(self.body_lock_interface, body, |chassis| {
                let mut inverse_inertia = Vec3::ZERO.to_jph();
                // SAFETY: the chassis is locked for reading and dynamic (`check_chassis`), so it has
                // motion properties; the getter writes a live local.
                unsafe {
                    let motion = JPH_Body_GetMotionProperties(chassis.as_ptr());
                    JPH_MotionProperties_GetInverseInertiaDiagonal(motion, &mut inverse_inertia);
                }
                let inverse_inertia = Vec3::from_jph(inverse_inertia);
                inverse_inertia
                    .x
                    .max(inverse_inertia.y)
                    .max(inverse_inertia.z)
            })
            .unwrap_or_else(|| {
                unreachable!("`check_chassis` found the body and `&mut self` keeps it")
            });
        if !limits::is_lean_spring(
            settings.lean_spring_constant,
            settings.lean_spring_damping,
            largest_inverse_inertia,
        ) {
            return Err(VehicleError::InvalidValue(limits::LEAN_SPRING_RULE));
        }
        let id = self.attach_vehicle(body, settings.build())?;
        let entry = self
            .vehicles
            .get_mut(&id.raw)
            .unwrap_or_else(|| unreachable!("just attached"));
        // SAFETY: the vehicle was just created with a motorcycle controller, which derives from
        // `VehicleController` with single inheritance; the world is borrowed mutably and no step
        // has run yet. The setters write members.
        unsafe {
            let controller: *mut JPH_MotorcycleController =
                JPH_VehicleConstraint_GetController(entry.constraint.as_ptr()).cast();
            JPH_MotorcycleController_EnableLeanController(controller, settings.lean_controller);
            JPH_MotorcycleController_EnableLeanSteeringLimit(
                controller,
                settings.lean_steering_limit,
            );
        }
        Ok(VehicleId::new(id.raw, self.tag))
    }
}
