//! How the wheels find the ground.

use std::f32::consts::PI;

use oxijolt_sys::*;

use super::WheelGeometry;
use crate::math::{is_finite_non_negative, is_finite_positive, is_unit};
use crate::owned::{JoltObject, Owned};
use crate::{ObjectLayer, Vec3, VehicleError};

/// How the wheels find the ground (Jolt's `VehicleCollisionTester` kinds). Each wheel casts
/// along its suspension from its attachment point at the start of every step.
///
/// The wheels see the bodies whose object layer collides with the tester's `object_layer` in
/// the world's [`CollisionLayers`](crate::CollisionLayers), never their own chassis, never
/// sensors and never soft bodies: Jolt's `VehicleConstraint` solves the body under a wheel as a
/// rigid body, so a wheel passes through a soft body to the ground below it. Jolt's testers apply no per-sub-shape filter, so compound children cannot be
/// excluded by group; give the wheels a dedicated object layer that collides with exactly the
/// layers they should drive on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VehicleCollisionTester {
    /// A ray from the attachment point, `suspension_max_length + radius` long. Cheapest; misses
    /// small steps and edges between the ray and the tire.
    Ray {
        /// The layer the ray queries as.
        object_layer: ObjectLayer,
        /// World-space up for the slope check, a unit vector. It is fixed: replace the tester
        /// to change it.
        up: Vec3,
        /// Steepest ground, radians in `[0, π]`, measured from `up`, that still counts as
        /// ground.
        max_slope_angle: f32,
    },
    /// A sphere of `radius` cast from the attachment point.
    CastSphere {
        /// The layer the sphere queries as.
        object_layer: ObjectLayer,
        /// Radius of the cast sphere, positive and smaller than every wheel's
        /// `suspension_max_length + radius`.
        radius: f32,
        /// World-space up for the slope check, a unit vector, fixed as for `Ray`.
        up: Vec3,
        /// Steepest ground, radians in `[0, π]`, measured from `up`.
        max_slope_angle: f32,
    },
    /// A cylinder of the wheel's radius and width cast along the suspension. Closest to the
    /// tire's shape; no slope check.
    CastCylinder {
        /// The layer the cylinder queries as.
        object_layer: ObjectLayer,
        /// Fraction in `[0, 1]` of `min(width / 2, radius)` used as the cylinder's convex radius.
        convex_radius_fraction: f32,
    },
}

impl VehicleCollisionTester {
    /// A ray tester with up +Y and Jolt's sample slope limit of 80°.
    pub fn ray(object_layer: ObjectLayer) -> Self {
        Self::Ray {
            object_layer,
            up: Vec3::new(0.0, 1.0, 0.0),
            max_slope_angle: 80.0_f32.to_radians(),
        }
    }

    /// A sphere tester of `radius` with up +Y and a slope limit of 80°.
    pub fn cast_sphere(object_layer: ObjectLayer, radius: f32) -> Self {
        Self::CastSphere {
            object_layer,
            radius,
            up: Vec3::new(0.0, 1.0, 0.0),
            max_slope_angle: 80.0_f32.to_radians(),
        }
    }

    /// A cylinder tester with Jolt's default convex radius fraction 0.1.
    pub fn cast_cylinder(object_layer: ObjectLayer) -> Self {
        Self::CastCylinder {
            object_layer,
            convex_radius_fraction: 0.1,
        }
    }

    /// The object layer the tester queries as.
    pub fn object_layer(&self) -> ObjectLayer {
        match *self {
            Self::Ray { object_layer, .. }
            | Self::CastSphere { object_layer, .. }
            | Self::CastCylinder { object_layer, .. } => object_layer,
        }
    }

    /// The world-space up of a ray or sphere tester; `None` for the cylinder.
    pub fn up(&self) -> Option<Vec3> {
        match *self {
            Self::Ray { up, .. } | Self::CastSphere { up, .. } => Some(up),
            Self::CastCylinder { .. } => None,
        }
    }

    /// This tester with its up replaced by `up`; the cylinder has none and stays as it is.
    pub(crate) fn with_up(self, new_up: Vec3) -> Self {
        match self {
            Self::Ray {
                object_layer,
                max_slope_angle,
                ..
            } => Self::Ray {
                object_layer,
                up: new_up,
                max_slope_angle,
            },
            Self::CastSphere {
                object_layer,
                radius,
                max_slope_angle,
                ..
            } => Self::CastSphere {
                object_layer,
                radius,
                up: new_up,
                max_slope_angle,
            },
            cylinder @ Self::CastCylinder { .. } => cylinder,
        }
    }

    /// Checks the tester against the world's layers and the wheels it serves.
    pub(crate) fn validate(
        &self,
        object_layer_count: u32,
        mut wheels: impl Iterator<Item = WheelGeometry>,
    ) -> Result<(), VehicleError> {
        let invalid = |what| Err(VehicleError::InvalidValue(what));
        if self.object_layer().get() >= object_layer_count {
            return invalid("collision tester object layer does not exist in this world");
        }
        let slope_ok = |angle: f32| is_finite_non_negative(angle) && angle <= PI;
        match *self {
            Self::Ray {
                up,
                max_slope_angle,
                ..
            } => {
                // The ray length `max length + radius` (`VehicleCollisionTesterRay::Collide`)
                // is finite: wheel lengths are at most `limits::MAX_SHAPE_EXTENT`.
                if !(is_unit(up) && slope_ok(max_slope_angle)) {
                    return invalid(
                        "ray tester needs a unit up and a max slope angle between 0 and pi",
                    );
                }
            }
            Self::CastSphere {
                radius,
                up,
                max_slope_angle,
                ..
            } => {
                if !(is_unit(up) && slope_ok(max_slope_angle)) {
                    return invalid(
                        "sphere tester needs a unit up and a max slope angle between 0 and pi",
                    );
                }
                if !is_finite_positive(radius) {
                    return invalid("sphere tester radius must be finite and positive");
                }
                // The cast length (`VehicleCollisionTesterCastSphere::Collide`), finite because
                // wheel lengths are at most `limits::MAX_SHAPE_EXTENT`.
                if !wheels.all(|w| w.suspension_max_length + w.radius - radius > 0.0) {
                    return invalid(
                        "sphere tester radius must be below every wheel's max suspension length plus radius",
                    );
                }
            }
            Self::CastCylinder {
                convex_radius_fraction,
                ..
            } => {
                if !(convex_radius_fraction.is_finite()
                    && (0.0..=1.0).contains(&convex_radius_fraction))
                {
                    return invalid("cylinder tester convex radius fraction must be in [0, 1]");
                }
                if !wheels.all(|w| w.width > 0.0 && w.suspension_max_length > 0.0) {
                    return invalid(
                        "cylinder tester needs every wheel's width and max suspension length positive",
                    );
                }
            }
        }
        Ok(())
    }

    /// The joltc tester of this validated tester for the vehicle whose chassis has the Jolt id
    /// `vehicle_body`, holding one reference. It skips the chassis and every soft body: Jolt's
    /// `VehicleConstraint` solves the body under a wheel as a rigid body.
    pub(crate) fn create(&self, vehicle_body: JPH_BodyID) -> Owned<JPH_VehicleCollisionTester> {
        let tester: *mut JPH_VehicleCollisionTester = match *self {
            Self::Ray {
                object_layer,
                up,
                max_slope_angle,
            } => {
                let up = up.to_jph();
                // SAFETY: Jolt is initialised (a world exists); `up` is a live local and every
                // value was validated.
                unsafe {
                    JPH_VehicleCollisionTesterRay_Create2(
                        object_layer.get(),
                        &up,
                        max_slope_angle,
                        vehicle_body,
                    )
                }
                .cast()
            }
            Self::CastSphere {
                object_layer,
                radius,
                up,
                max_slope_angle,
            } => {
                let up = up.to_jph();
                // SAFETY: as for the ray.
                unsafe {
                    JPH_VehicleCollisionTesterCastSphere_Create2(
                        object_layer.get(),
                        radius,
                        &up,
                        max_slope_angle,
                        vehicle_body,
                    )
                }
                .cast()
            }
            Self::CastCylinder {
                object_layer,
                convex_radius_fraction,
            } => {
                // SAFETY: as for the ray.
                unsafe {
                    JPH_VehicleCollisionTesterCastCylinder_Create2(
                        object_layer.get(),
                        convex_radius_fraction,
                        vehicle_body,
                    )
                }
                .cast()
            }
        };
        // SAFETY: the extension returns the new tester holding one reference (it calls `AddRef`),
        // which the guard takes over; the tester owns its body filter. Every tester kind derives
        // from `VehicleCollisionTester` with single inheritance, joltc's cast convention.
        unsafe { Owned::from_raw(tester) }
            .unwrap_or_else(|| unreachable!("joltc `new`s the tester"))
    }
}

/// A collision tester, of which the owner holds one reference. A vehicle keeps its own
/// (`VehicleConstraint::mVehicleCollisionTester`), so the owner may release its reference once
/// the tester is set.
impl JoltObject for JPH_VehicleCollisionTester {
    unsafe fn destroy(ptr: *mut Self) {
        // SAFETY: the owner holds one reference (trait contract), released here once.
        unsafe { JPH_VehicleCollisionTester_Destroy(ptr) };
    }
}
