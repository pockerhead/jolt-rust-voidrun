//! Reading a vehicle of any kind: wheels, gravity, engine and transmission.

use oxijolt_sys::*;

use super::{engine_and_transmission, VehicleCollisionTester, VehicleEntry, VehicleKind};
use crate::{BodyId, PhysicsWorld, RVec3, SubShapeId, Vec3, VehicleId};

/// Jolt's invalid `BodyID` value.
const INVALID_BODY_ID: u32 = 0xffff_ffff;

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
    /// Rotation speed of the wheel, rad/s; positive when it rolls the vehicle forward. A tracked
    /// vehicle's wheel turns with its track.
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

/// Read access to one vehicle of kind `K`, borrowed from its world.
///
/// Each method reads the state the last step left. The chassis pose and velocities come from
/// [`PhysicsWorld::body`] with [`body`](Self::body).
pub struct VehicleRef<'w, K: VehicleKind = super::WheeledVehicle> {
    pub(super) world: &'w PhysicsWorld,
    pub(super) id: VehicleId<K>,
    pub(super) entry: &'w VehicleEntry,
}

impl<K: VehicleKind> VehicleRef<'_, K> {
    pub(super) fn ptr(&self) -> *mut JPH_VehicleConstraint {
        self.entry.constraint.as_ptr()
    }

    /// The vehicle's controller; the world borrowed here owns the constraint, which owns the
    /// controller.
    pub(super) fn controller(&self) -> *mut JPH_VehicleController {
        // SAFETY: the world borrowed here owns the constraint, and changing it needs
        // `&mut PhysicsWorld`; the getter returns a member.
        unsafe { JPH_VehicleConstraint_GetController(self.ptr()) }
    }

    /// The vehicle's id.
    pub fn id(&self) -> VehicleId<K> {
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

    /// The gravity override of [`VehicleMut::set_gravity`](crate::VehicleMut::set_gravity),
    /// m/s², or `None` while the vehicle uses the world's gravity.
    pub fn gravity(&self) -> Option<Vec3> {
        super::gravity_override(self.entry)
    }

    /// The world up of the last step: the opposite of the gravity the vehicle used, normalized.
    /// The pitch and roll limit keeps the vehicle's up within its angle of this direction. A step
    /// in zero gravity keeps the previous world up; it is not part of
    /// [`WorldState`](crate::WorldState).
    pub fn world_up(&self) -> Vec3 {
        let mut value = Vec3::ZERO.to_jph();
        // SAFETY: the world borrowed here owns the constraint; the getter reads a member into
        // `value`, a live local.
        unsafe { JPH_VehicleConstraint_GetWorldUp(self.ptr(), &mut value) };
        Vec3::from_jph(value)
    }

    /// Engine speed, rpm.
    pub fn engine_rpm(&self) -> f32 {
        let (engine, _) = engine_and_transmission(self.entry);
        // SAFETY: the engine is a member of the controller, which the constraint owns and the
        // world borrowed here keeps alive; it is only read.
        unsafe { JPH_VehicleEngine_GetCurrentRPM(engine) }
    }

    /// Current gear: −1 reverse, 0 neutral, 1 first gear and so on.
    pub fn current_gear(&self) -> i32 {
        let (_, transmission) = engine_and_transmission(self.entry);
        // SAFETY: as in `engine_rpm`, for the transmission.
        unsafe { JPH_VehicleTransmission_GetCurrentGear(transmission) }
    }

    /// The collision tester the wheels use.
    pub fn collision_tester(&self) -> &VehicleCollisionTester {
        &self.entry.collision_tester
    }
}
