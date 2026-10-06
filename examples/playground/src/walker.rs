//! A walking humanoid on Jolt's `CharacterVirtual`, shared by the character and vehicle
//! scenes: the guide's tick (caller-fed vertical speed, stick to floor, walk stairs) along the
//! character's own up, with the velocity of what it stands on added.

use glam::Vec3;
use oxijolt::{
    CharacterId, CharacterRef, CharacterSettings, ExtendedUpdateSettings, GroundState,
    PhysicsWorld, QueryFilter, RVec3,
};

use crate::camera::walk_direction;
use crate::digest::Digest;
use crate::input::Held;
use crate::math::{glam, glam_quat, position, quat, vec3};
use crate::scene::{Result, DT};
use crate::visual::Visual;

/// Gravity along the character's down, m/s².
const GRAVITY: f32 = 9.81;
/// Walking speed, m/s; sprinting doubles it.
const WALK_SPEED: f32 = 3.0;
/// Upward speed of a jump, m/s.
const JUMP_SPEED: f32 = 5.0;

/// A humanoid character and the vertical speed its caller carries between ticks.
#[derive(Debug)]
pub struct Walker {
    id: CharacterId,
    height: f32,
    radius: f32,
    vel_up: f32,
    jumped: bool,
}

impl Walker {
    /// A humanoid `height` metres tall of `radius` at `position`, standing on the ground.
    pub fn new(
        world: &mut PhysicsWorld,
        position: RVec3,
        height: f32,
        radius: f32,
    ) -> Result<Self> {
        let settings =
            CharacterSettings::humanoid(height, radius)?.max_slope_angle(45.0_f32.to_radians());
        let id = world.create_character(&settings, position, oxijolt::Quat::IDENTITY)?;
        // A new character knows no ground until its contacts are found.
        world.refresh_character_contacts(id, &QueryFilter::new())?;
        Ok(Self {
            id,
            height,
            radius,
            vel_up: 0.0,
            jumped: false,
        })
    }

    /// The character.
    pub fn id(&self) -> CharacterId {
        self.id
    }

    /// Whether the walker has jumped since it was created.
    pub fn has_jumped(&self) -> bool {
        self.jumped
    }

    /// One tick: walk along `held` (relative to its camera yaw), jump when `jump` and on the
    /// ground, fall under gravity along the character's down.
    pub fn tick(&mut self, world: &mut PhysicsWorld, held: &Held, jump: bool) -> Result<()> {
        let character = world.character(self.id)?;
        let up = glam(character.up());
        let on_ground = character.ground_state() == GroundState::OnGround;
        let ground_velocity = glam(character.ground_velocity());
        self.vel_up = if on_ground {
            // One tick of gravity, so that stick to floor has something to follow.
            -GRAVITY * DT
        } else {
            self.vel_up - GRAVITY * DT
        };
        if jump && on_ground {
            self.vel_up = JUMP_SPEED;
            self.jumped = true;
        }
        let speed = if held.sprint {
            2.0 * WALK_SPEED
        } else {
            WALK_SPEED
        };
        let along_ground = |v: Vec3| v - up * v.dot(up);
        let walk = along_ground(walk_direction(held.walk, held.camera_yaw)) * speed;
        let carried = if on_ground {
            along_ground(ground_velocity)
        } else {
            Vec3::ZERO
        };
        let mut character = world.character_mut(self.id)?;
        character.set_linear_velocity(vec3(walk + carried + up * self.vel_up))?;
        let extended = ExtendedUpdateSettings::default()
            .stick_to_floor_step_down(vec3(up * -0.5))
            .walk_stairs_step_up(vec3(up * 0.4));
        world.update_character(
            self.id,
            DT,
            vec3(up * -GRAVITY),
            &extended,
            &QueryFilter::new(),
        )?;
        Ok(())
    }

    /// The capsule's description.
    pub fn visual(&self) -> Visual {
        Visual::Capsule {
            half_height: self.height / 2.0 - self.radius,
            radius: self.radius,
        }
    }

    /// Where the capsule is: the position, plus the rotated shape offset, plus the padding along
    /// up, as Jolt places the shape; and its rotation.
    pub fn capsule_pose(&self, character: &CharacterRef<'_>) -> ([f32; 3], [f32; 4]) {
        capsule_pose(character, self.height)
    }

    /// Folds the character's state and the caller's vertical speed into `digest`.
    pub fn write_state(&self, world: &PhysicsWorld, digest: &mut Digest) -> Result<()> {
        let character = world.character(self.id)?;
        digest.rvec3(character.position());
        digest.quat(character.rotation());
        digest.vec3(character.linear_velocity());
        digest.u32(character.ground_state() as u32);
        digest.u32(
            character
                .ground_body()
                .map_or(u32::MAX, |body| body.to_raw()),
        );
        digest.f32(self.vel_up);
        digest.bool(self.jumped);
        Ok(())
    }
}

/// The pose of a humanoid capsule `height` tall on `character`: Jolt puts the shape at the
/// position plus the rotated shape offset `(0, height / 2, 0)` plus the padding along up.
pub fn capsule_pose(character: &CharacterRef<'_>, height: f32) -> ([f32; 3], [f32; 4]) {
    /// Jolt's default `character_padding`, which the humanoid preset keeps.
    const PADDING: f32 = 0.02;
    let rotation = glam_quat(character.rotation());
    let centre = position(character.position())
        + rotation * Vec3::new(0.0, height / 2.0, 0.0)
        + glam(character.up()) * PADDING;
    (centre.to_array(), <[f32; 4]>::from(quat(rotation)))
}

#[cfg(test)]
mod tests {
    use oxijolt::{InnerBody, ObjectLayer, Shape, WorldSettings};

    use super::*;
    use crate::math::{about_axis, rvec};

    /// Jolt keeps a character's inner body where the character's shape is, so the drawn
    /// capsule must sit exactly on it.
    #[test]
    fn the_drawn_capsule_is_the_character_shape() {
        let (height, radius) = (1.8, 0.3);
        let capsule = Shape::new_capsule(height / 2.0 - radius, radius).unwrap();
        for tilt in [0.0, 30.0_f32.to_radians()] {
            let mut world = PhysicsWorld::new(WorldSettings::default()).unwrap();
            let rotation = about_axis([1.0, 0.0, 0.3], tilt);
            let up = glam_quat(rotation) * Vec3::Y;
            let settings = CharacterSettings::humanoid(height, radius)
                .unwrap()
                .up(vec3(up))
                .inner_body(Some(InnerBody {
                    shape: &capsule,
                    object_layer: ObjectLayer::MOVING,
                }));
            let id = world
                .create_character(&settings, rvec([1.0, 2.0, -3.0]), rotation)
                .unwrap();
            let character = world.character(id).unwrap();
            let (centre, drawn_rotation) = capsule_pose(&character, height);
            let inner = world.body(character.inner_body().unwrap()).unwrap();
            let inner_centre = position(inner.position());
            assert!(
                inner_centre.distance(Vec3::from(centre)) < 1e-5,
                "tilt {tilt}: drawn {centre:?}, inner body {inner_centre:?}"
            );
            let inner_rotation = glam_quat(inner.rotation());
            assert!(
                inner_rotation.angle_between(glam::Quat::from_array(drawn_rotation)) < 1e-4,
                "tilt {tilt}"
            );
        }
    }
}
