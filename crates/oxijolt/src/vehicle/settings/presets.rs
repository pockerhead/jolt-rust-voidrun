//! Ready-made settings for common vehicles, built from the plain settings types; the vehicle
//! creators validate them like any other settings.

use super::{
    MotorcycleSettings, VehicleAntiRollBar, VehicleCollisionTester, VehicleDifferentialSettings,
    VehicleEngineSettings, VehicleTransmissionSettings, WheelSettings, WheeledVehicleSettings,
};
use crate::{SpringSettings, Vec3};

impl WheeledVehicleSettings {
    /// A four-wheeled, front-wheel-drive car with Jolt's default engine and automatic
    /// transmission.
    ///
    /// `front_left_wheel` is the front left wheel's attachment point in the chassis' local
    /// space, with up +Y and forward +Z, so left is +X, as in Jolt's samples. The other wheels
    /// mirror it: front right at `(-x, y, z)`, rear left at `(x, y, -z)`, rear right at
    /// `(-x, y, -z)`, in that wheel order. Every wheel has radius `wheel_radius`, width 0.2 and
    /// suspension travel 0.05 to 0.5 m. The front wheels steer up to 30° and are driven through
    /// one differential; the hand brake holds the rear wheels. Anti-roll bars join the front pair
    /// and the rear pair. The wheels find the ground with `collision_tester`.
    ///
    /// The values suit a chassis like the one the tests drive: a 1500 kg box of half extents
    /// (0.9, 0.3, 2.0) with its centre of mass moved 0.3 m down, and wheels at
    /// `(0.9, -0.1, 1.4)` of radius 0.35. Nothing is checked here:
    /// [`PhysicsWorld::create_wheeled_vehicle`](crate::PhysicsWorld::create_wheeled_vehicle) validates the
    /// settings and refuses, for example, a radius that is not positive.
    ///
    /// # Example
    /// ```
    /// use oxijolt::*;
    ///
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// let mut world = PhysicsWorld::new(WorldSettings::default())?;
    /// let hull = Shape::new_box(Vec3::new(0.9, 0.3, 2.0))?;
    /// let chassis_shape = Shape::new_offset_center_of_mass(&hull, Vec3::new(0.0, -0.3, 0.0))?;
    /// let chassis = world.create_body(
    ///     &chassis_shape,
    ///     &BodySettings::new_dynamic().position(RVec3::new(0.0, 1.0, 0.0)).mass(1500.0),
    /// )?;
    /// let tester = VehicleCollisionTester::cast_sphere(ObjectLayer::MOVING, 0.2);
    /// let settings = WheeledVehicleSettings::car(Vec3::new(0.9, -0.1, 1.4), 0.35, tester);
    /// let car = world.create_wheeled_vehicle(chassis, &settings)?;
    /// assert_eq!(world.vehicle(car)?.wheel_count(), 4);
    /// # Ok(())
    /// # }
    /// ```
    pub fn car(
        front_left_wheel: Vec3,
        wheel_radius: f32,
        collision_tester: VehicleCollisionTester,
    ) -> Self {
        let Vec3 { x, y, z } = front_left_wheel;
        let wheel = |x: f32, z: f32| {
            WheelSettings::new(Vec3::new(x, y, z))
                .radius(wheel_radius)
                .width(0.2)
                .suspension_min_length(0.05)
                .suspension_max_length(0.5)
        };
        let front = |x| {
            wheel(x, z)
                .max_steer_angle(30.0_f32.to_radians())
                .max_hand_brake_torque(0.0)
        };
        let rear = |x| wheel(x, -z).max_steer_angle(0.0);
        WheeledVehicleSettings::new(
            vec![front(x), front(-x), rear(x), rear(-x)],
            vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
            collision_tester,
        )
        .anti_roll_bars(vec![
            VehicleAntiRollBar::new(0, 1),
            VehicleAntiRollBar::new(2, 3),
        ])
    }
}

impl MotorcycleSettings {
    /// The motorcycle of Jolt's `MotorcycleTest` sample, with a six-gear 150 N·m engine up to
    /// 10000 rpm driving the rear wheel and a pitch and roll limit of 60°.
    ///
    /// `front_wheel` is the front wheel's attachment point in the chassis' local space, with up
    /// +Y and forward +Z; the rear wheel is attached at `(x, y, -z)`. Both wheels have radius
    /// `wheel_radius`, width 0.05 and suspension travel 0.3 to 0.5 m. The
    /// front suspension and steering axis are raked back 30° (caster) and the front wheel steers
    /// up to 30°; the front brake gives 500 N·m, the rear 250 N·m. The lean controller keeps
    /// Jolt's defaults. The wheels find the ground with `collision_tester`.
    ///
    /// The sample's chassis is a 240 kg box of half extents (0.2, 0.3, 0.4) with its centre of
    /// mass moved 0.3 m down, and wheels at `(0, -0.27, 0.75)` of radius 0.31.
    /// [`PhysicsWorld::create_motorcycle`](crate::PhysicsWorld::create_motorcycle) validates the
    /// settings; it refuses wheels at one point along forward, which here is about
    /// `z = -0.125`, where the end of the front's raked suspension is level with the rear's.
    pub fn bike(
        front_wheel: Vec3,
        wheel_radius: f32,
        collision_tester: VehicleCollisionTester,
    ) -> Self {
        let rake = 30.0_f32.to_radians().tan();
        let length = (1.0 + rake * rake).sqrt();
        let Vec3 { x, y, z } = front_wheel;
        let wheel = |z: f32, frequency: f32, brake: f32| {
            WheelSettings::new(Vec3::new(x, y, z))
                .radius(wheel_radius)
                .width(0.05)
                .suspension_min_length(0.3)
                .suspension_max_length(0.5)
                .suspension_spring(SpringSettings::FrequencyAndDamping {
                    frequency,
                    damping: 0.5,
                })
                .max_brake_torque(brake)
        };
        let front = wheel(z, 1.5, 500.0)
            .max_steer_angle(30.0_f32.to_radians())
            .suspension_direction(Vec3::new(0.0, -1.0 / length, rake / length))
            .steering_axis(Vec3::new(0.0, 1.0 / length, -rake / length));
        let rear = wheel(-z, 2.0, 250.0).max_steer_angle(0.0);
        let rear_drive =
            VehicleDifferentialSettings::new(None, Some(1)).differential_ratio(1.93 * 40.0 / 16.0);
        let vehicle =
            WheeledVehicleSettings::new(vec![front, rear], vec![rear_drive], collision_tester)
                .max_pitch_roll_angle(60.0_f32.to_radians())
                .engine(
                    VehicleEngineSettings::default()
                        .max_torque(150.0)
                        .min_rpm(1000.0)
                        .max_rpm(10000.0),
                )
                .transmission(
                    VehicleTransmissionSettings::default()
                        .gear_ratios(vec![2.27, 1.63, 1.3, 1.09, 0.96, 0.88])
                        .reverse_gear_ratios(vec![-4.0])
                        .shift_down_rpm(2000.0)
                        .shift_up_rpm(8000.0)
                        .clutch_strength(2.0),
                );
        MotorcycleSettings::new(vehicle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ObjectLayer;

    #[test]
    fn car_preset_equals_the_explicit_settings() {
        let tester = VehicleCollisionTester::cast_sphere(ObjectLayer::MOVING, 0.2);
        let wheel = |x: f32, z: f32| {
            WheelSettings::new(Vec3::new(x, -0.1, z))
                .radius(0.35)
                .width(0.2)
                .suspension_min_length(0.05)
                .suspension_max_length(0.5)
        };
        let front = |x| {
            wheel(x, 1.4)
                .max_steer_angle(30.0_f32.to_radians())
                .max_hand_brake_torque(0.0)
        };
        let rear = |x| wheel(x, -1.4).max_steer_angle(0.0);
        let explicit = WheeledVehicleSettings::new(
            vec![front(0.9), front(-0.9), rear(0.9), rear(-0.9)],
            vec![VehicleDifferentialSettings::new(Some(0), Some(1))],
            tester,
        )
        .anti_roll_bars(vec![
            VehicleAntiRollBar::new(0, 1),
            VehicleAntiRollBar::new(2, 3),
        ])
        .engine(VehicleEngineSettings::default())
        .transmission(VehicleTransmissionSettings::default());
        assert_eq!(
            WheeledVehicleSettings::car(Vec3::new(0.9, -0.1, 1.4), 0.35, tester),
            explicit
        );
    }
}
