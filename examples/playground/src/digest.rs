//! A 64-bit FNV-1a digest of typed values, to compare scene runs bit for bit.

use oxijolt::{PhysicsWorld, Quat, RVec3, Vec3};

use crate::draw::DrawList;
use crate::scene::Result;

/// FNV-1a over the little-endian bytes of the values folded in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Digest(u64);

impl Default for Digest {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Digest {
    /// Folds raw bytes.
    pub fn bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }

    /// Folds a flag.
    pub fn bool(&mut self, value: bool) {
        self.bytes(&[u8::from(value)]);
    }

    /// Folds an integer.
    pub fn u32(&mut self, value: u32) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds an integer.
    pub fn i32(&mut self, value: i32) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds an integer.
    pub fn u64(&mut self, value: u64) {
        self.bytes(&value.to_le_bytes());
    }

    /// Folds the bits of a float.
    pub fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }

    /// Folds the bits of each component.
    pub fn f32s(&mut self, values: &[f32]) {
        for &value in values {
            self.f32(value);
        }
    }

    /// Folds the bits of each component.
    pub fn vec3(&mut self, value: Vec3) {
        self.f32s(&<[f32; 3]>::from(value));
    }

    /// Folds the bits of each component.
    pub fn quat(&mut self, value: Quat) {
        self.f32s(&<[f32; 4]>::from(value));
    }

    /// Folds the bits of each component, in the precision of positions.
    pub fn rvec3(&mut self, value: RVec3) {
        for component in <[oxijolt::Real; 3]>::from(value) {
            self.bytes(&component.to_le_bytes());
        }
    }

    /// The digest of everything folded so far.
    pub fn finish(&self) -> u64 {
        self.0
    }

    /// Folds the state of every body of `world` in ascending id order, as the binding reports
    /// it: the raw id, the motion type, whether the body is active, its position, rotation and
    /// linear and angular velocity, and for a soft body each vertex's position, velocity and
    /// inverse mass.
    pub fn world(&mut self, world: &mut PhysicsWorld) -> Result<()> {
        let ids = world.body_ids();
        self.u64(u64::from(world.body_count()));
        let mut vertices = Vec::new();
        for id in ids {
            let body = world.body(id)?;
            self.u32(id.to_raw());
            self.u32(body.motion_type() as u32);
            self.bool(body.is_active());
            self.rvec3(body.position());
            self.quat(body.rotation());
            self.vec3(body.linear_velocity());
            self.vec3(body.angular_velocity());
            if body.is_soft_body() {
                world.soft_body(id)?.vertices_into(&mut vertices);
                self.u64(vertices.len() as u64);
                for vertex in &vertices {
                    self.rvec3(vertex.position);
                    self.vec3(vertex.velocity);
                    self.f32(vertex.inverse_mass);
                }
            }
        }
        Ok(())
    }

    /// Folds what a draw list shows: every solid in order (description id and version, pose,
    /// colour) and every surface vertex. Lines and text are left out, so the wireframe and the
    /// HUD do not change a run's digest.
    pub fn draw_list(&mut self, list: &DrawList) {
        self.u64(list.solids.len() as u64);
        for solid in &list.solids {
            self.u32(solid.visual.id);
            self.u32(solid.visual.version);
            self.f32s(&solid.position);
            self.f32s(&solid.rotation);
            self.f32s(&solid.colour);
        }
        self.u64(list.surfaces.len() as u64);
        for surface in &list.surfaces {
            for vertex in &surface.vertices {
                self.f32s(vertex);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use oxijolt::{BodyId, BodySettings, EventSettings, Shape, SoftBodySettings};
    use oxijolt::{SoftBodySharedSettings, SoftBodyVertex};

    use super::*;
    use crate::math::rvec;
    use crate::scene::{new_world, SceneConfig};

    /// A world with a floor, a resting cube that no scene tracks, and a soft tetrahedron, none
    /// of them stepped yet: every body is active and at rest.
    fn fixture() -> (PhysicsWorld, BodyId, BodyId) {
        let (mut world, layers) =
            new_world(&SceneConfig::default(), 1, EventSettings::default()).expect("a world");
        world
            .create_body(
                &Shape::new_box(Vec3::new(10.0, 0.5, 10.0)).unwrap(),
                &BodySettings::new_static()
                    .position(rvec([0.0, -0.5, 0.0]))
                    .object_layer(layers.ground),
            )
            .unwrap();
        let cube = world
            .create_body(
                &Shape::new_box(Vec3::new(0.5, 0.5, 0.5)).unwrap(),
                &BodySettings::new_dynamic()
                    .position(rvec([0.0, 0.5, 0.0]))
                    .object_layer(layers.moving),
            )
            .unwrap();
        let corners = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ];
        let vertices = corners
            .iter()
            .map(|&corner| SoftBodyVertex::new(Vec3::from(corner)))
            .collect();
        let faces = vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];
        let shared = SoftBodySharedSettings::builder(vertices, faces)
            .build()
            .unwrap();
        let soft = world
            .create_soft_body(
                &shared,
                &SoftBodySettings::default()
                    .position(rvec([3.0, 1.0, 0.0]))
                    .object_layer(layers.moving),
            )
            .unwrap();
        (world, cube, soft)
    }

    fn digest_of(world: &mut PhysicsWorld) -> u64 {
        let mut digest = Digest::default();
        digest.world(world).unwrap();
        digest.finish()
    }

    fn pose(world: &PhysicsWorld, body: BodyId) -> (RVec3, Quat) {
        let reading = world.body(body).unwrap();
        (reading.position(), reading.rotation())
    }

    #[test]
    fn the_world_digest_repeats_for_the_same_state() {
        let (mut first, _, _) = fixture();
        let (mut second, _, _) = fixture();
        assert_eq!(digest_of(&mut first), digest_of(&mut second));
    }

    #[test]
    fn a_velocity_change_alone_changes_the_world_digest() {
        let (mut world, cube, _) = fixture();
        let (before, at) = (digest_of(&mut world), pose(&world, cube));
        world
            .body_mut(cube)
            .unwrap()
            .set_angular_velocity(Vec3::new(0.0, 1.0e-3, 0.0))
            .unwrap();
        assert_eq!(pose(&world, cube), at);
        assert_ne!(digest_of(&mut world), before);
    }

    #[test]
    fn an_activation_change_alone_changes_the_world_digest() {
        let (mut world, cube, _) = fixture();
        let (before, at) = (digest_of(&mut world), pose(&world, cube));
        assert!(world.body(cube).unwrap().is_active());
        world.body_mut(cube).unwrap().deactivate().unwrap();
        let reading = world.body(cube).unwrap();
        assert!(!reading.is_active());
        assert_eq!((reading.position(), reading.rotation()), at);
        assert_eq!(reading.linear_velocity(), Vec3::ZERO);
        assert_eq!(reading.angular_velocity(), Vec3::ZERO);
        assert_ne!(digest_of(&mut world), before);
    }

    #[test]
    fn a_soft_vertex_velocity_change_alone_changes_the_world_digest() {
        let (mut world, _, soft) = fixture();
        let positions = |world: &PhysicsWorld| -> Vec<RVec3> {
            let vertices = world.soft_body(soft).unwrap().vertices();
            vertices.iter().map(|vertex| vertex.position).collect()
        };
        let (before, at) = (digest_of(&mut world), positions(&world));
        world
            .soft_body_mut(soft)
            .unwrap()
            .set_vertex_velocity(2, Vec3::new(1.0e-3, 0.0, 0.0))
            .unwrap();
        assert_eq!(positions(&world), at);
        assert_ne!(digest_of(&mut world), before);
    }
}
