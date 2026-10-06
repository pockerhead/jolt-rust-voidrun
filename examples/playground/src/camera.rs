//! An orbit camera around a target, the rays it shoots through the screen, and the walking
//! directions it gives the keys.

use glam::Vec3;
use oxijolt::RayCast;

/// How far a picking ray reaches, metres.
pub const RAY_LENGTH: f32 = 200.0;

/// A camera orbiting `target`: `yaw` about +Y (0 puts the eye on the target's +Z side),
/// `pitch` above the horizon, `distance` from the target, `fov_y` the vertical field of view;
/// angles in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraHint {
    /// The point looked at.
    pub target: [f32; 3],
    /// Turn about +Y.
    pub yaw: f32,
    /// Height above the horizon.
    pub pitch: f32,
    /// Distance from the target, metres.
    pub distance: f32,
    /// Vertical field of view.
    pub fov_y: f32,
}

impl CameraHint {
    /// A camera looking at `target` from `distance` at `yaw` and `pitch`, with a 50° field of
    /// view.
    pub fn new(target: [f32; 3], yaw: f32, pitch: f32, distance: f32) -> Self {
        Self {
            target,
            yaw,
            pitch,
            distance,
            fov_y: 50.0_f32.to_radians(),
        }
    }

    /// The eye position.
    pub fn eye(&self) -> Vec3 {
        let (sin_yaw, cos_yaw) = self.yaw.sin_cos();
        let (sin_pitch, cos_pitch) = self.pitch.sin_cos();
        Vec3::from(self.target)
            + self.distance * Vec3::new(sin_yaw * cos_pitch, sin_pitch, cos_yaw * cos_pitch)
    }

    /// The ray from the eye through the point `ndc` of the screen (x right, y up, both in
    /// `[-1, 1]`) on a screen `aspect` times wider than high.
    pub fn ray(&self, ndc: [f32; 2], aspect: f32) -> RayCast {
        let eye = self.eye();
        let forward = (Vec3::from(self.target) - eye).normalize();
        let right = forward.cross(Vec3::Y).normalize();
        let up = right.cross(forward);
        let half_height = (self.fov_y / 2.0).tan();
        let direction =
            (forward + right * (ndc[0] * half_height * aspect) + up * (ndc[1] * half_height))
                .normalize()
                * RAY_LENGTH;
        RayCast::new(
            crate::math::rvec(eye.to_array()),
            crate::math::vec3(direction),
        )
    }
}

/// The world direction on the ground of a walk input (x right, y forward relative to a camera
/// at `yaw`), with its length.
pub fn walk_direction(walk: [f32; 2], yaw: f32) -> Vec3 {
    let (sin_yaw, cos_yaw) = yaw.sin_cos();
    let forward = Vec3::new(-sin_yaw, 0.0, -cos_yaw);
    let right = Vec3::new(cos_yaw, 0.0, -sin_yaw);
    let direction = right * walk[0] + forward * walk[1];
    if direction.length_squared() > 1.0 {
        direction.normalize()
    } else {
        direction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_centre_ray_hits_the_target() {
        let camera = CameraHint::new([1.0, 2.0, 3.0], 0.7, 0.4, 10.0);
        let ray = camera.ray([0.0, 0.0], 16.0 / 9.0);
        let origin = Vec3::from(crate::math::position_f32(ray.origin()));
        let direction = Vec3::from(<[f32; 3]>::from(ray.direction()));
        let at_target = origin + direction * (10.0 / RAY_LENGTH);
        assert!(at_target.distance(Vec3::new(1.0, 2.0, 3.0)) < 1e-4);
    }

    #[test]
    fn walking_forward_moves_away_from_the_eye() {
        for yaw in [0.0, 1.0, -2.5] {
            let camera = CameraHint::new([0.0; 3], yaw, 0.3, 5.0);
            let forward = walk_direction([0.0, 1.0], yaw);
            let away = (Vec3::from(camera.target) - camera.eye()) * Vec3::new(1.0, 0.0, 1.0);
            assert!(forward.dot(away.normalize()) > 0.999);
            let right = walk_direction([1.0, 0.0], yaw);
            assert!(
                right.cross(forward).y > 0.999,
                "right, forward and up are right-handed"
            );
        }
    }
}
