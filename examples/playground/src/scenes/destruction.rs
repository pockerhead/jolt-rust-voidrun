//! A breakable wall: one static body whose shape is published from a mutable compound of 48
//! bricks. Hard impacts and clicks knock bricks out: the hit children are decoded from the
//! contacts, removed from the compound, the rest is published to the wall, and each brick falls
//! on as a dynamic body.

use std::collections::VecDeque;

use oxijolt::{
    BodyId, BodySettings, CompoundChild, ContactEvent, EventSettings, MotionQuality,
    MutableCompound, PhysicsWorld, Quat, QueryFilter, RayCast, Shape, SubShapeId, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList};
use crate::input::{Edges, Input};
use crate::math::{glam, position, position_f32, rvec, vec3};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visual, VisualKey, Visuals};

/// Bricks across and up, and a brick's half extents, metres.
const WALL: [u32; 2] = [8, 6];
const BRICK: [f32; 3] = [0.25, 0.12, 0.15];
/// Where the wall stands.
const WALL_AT: [f32; 3] = [0.0, 0.0, 0.0];
/// Impulse in N·s from which an impact on the wall knocks bricks out.
const BREAKING_IMPULSE: f32 = 150.0;
/// How far around a brick that is hit hard the bricks come loose with it, metres.
const BLAST_RADIUS: f32 = 0.55;
/// Speed of a brick knocked out, m/s.
const KNOCK_SPEED: f32 = 5.0;
/// The most cannonballs and loose bricks at once; the oldest go first.
const MAX_BALLS: usize = 10;
const MAX_DEBRIS: usize = 200;

const MILESTONES: &[&str] = &["brick knocked out", "wall republished"];
const CONTROLS: &[(&str, &str)] = &[
    ("F", "fire a cannonball at the cursor"),
    ("left click", "knock out the brick under the cursor"),
];

/// The destruction scene.
pub struct Destruction {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    editor: MutableCompound,
    brick_shape: Shape,
    brick_visual: VisualKey,
    /// Brick ids and their positions in the wall, in the editor's child order.
    bricks: Vec<(u32, [f32; 3])>,
    wall: Option<(BodyId, VisualKey)>,
    ball: Shaped,
    ball_visual: VisualKey,
    balls: VecDeque<BodyId>,
    debris: VecDeque<BodyId>,
    publications: u32,
    milestones: Milestones,
}

impl Destruction {
    /// Builds the floor and the wall.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let events = EventSettings::default()
            .contacts(true)
            .collision_estimates(true);
        let (mut world, layers) = new_world(config, 3, events)?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        tracked.spawn(
            &mut world,
            &Shaped::plane(40.0)?,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;

        let brick = Shaped::cuboid(BRICK)?;
        let mut bricks = Vec::new();
        for row in 0..WALL[1] {
            for column in 0..WALL[0] {
                // Every other row is offset by a quarter brick, like a laid wall.
                let shift = if row % 2 == 0 { -0.06 } else { 0.06 };
                let x = (column as f32 - (WALL[0] - 1) as f32 / 2.0) * 2.0 * BRICK[0] + shift;
                let y = BRICK[1] + row as f32 * 2.0 * BRICK[1];
                bricks.push((row * WALL[0] + column, [x, y, 0.0]));
            }
        }
        let children: Vec<CompoundChild<'_>> = bricks
            .iter()
            .map(|&(id, at)| CompoundChild {
                shape: &brick.shape,
                position: at.into(),
                rotation: Quat::IDENTITY,
                user_data: id,
            })
            .collect();
        let editor = MutableCompound::from_children(&children)?;
        let wall_visual = visuals.add(wall_visual(&brick.visual, &bricks));
        let wall_body =
            world.create_body(&editor.to_shape()?, &ground.clone().position(rvec(WALL_AT)))?;
        tracked.adopt(&world, wall_body, wall_visual, colours::STRUCTURE)?;
        let ball = Shaped::sphere(0.25)?;
        let ball_visual = visuals.add(ball.visual.clone());
        let brick_visual = visuals.add(brick.visual.clone());
        Ok(Self {
            world,
            layers,
            visuals,
            tracked,
            editor,
            brick_shape: brick.shape,
            brick_visual,
            bricks,
            wall: Some((wall_body, wall_visual)),
            ball,
            ball_visual,
            balls: VecDeque::new(),
            debris: VecDeque::new(),
            publications: 0,
            milestones: Milestones::new(MILESTONES),
        })
    }

    /// Fires a cannonball along `ray`.
    fn fire(&mut self, ray: RayCast) -> Result<()> {
        if self.balls.len() >= MAX_BALLS {
            if let Some(oldest) = self.balls.pop_front() {
                self.tracked.remove(&mut self.world, oldest)?;
            }
        }
        let direction = glam(ray.direction).normalize();
        let settings = BodySettings::new_dynamic()
            .position(rvec((position(ray.origin) + direction).to_array()))
            .linear_velocity(vec3(direction * 25.0))
            .motion_quality(MotionQuality::LinearCast)
            .object_layer(self.layers.moving)
            .mass(50.0);
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &self.ball.shape,
            self.ball_visual,
            &settings,
            colours::PLAYER,
        )?;
        self.balls.push_back(body);
        Ok(())
    }

    /// The bricks of the wall that `sub_shape` of the wall's current shape leads to.
    fn brick_of(&self, sub_shape: SubShapeId) -> Option<u32> {
        let (wall, _) = self.wall?;
        let child = self.world.compound_sub_shape(wall, sub_shape).ok()??;
        Some(child.user_data)
    }

    /// The bricks hit hard enough by the step's new contacts, decoded against the wall's
    /// current shape, with their neighbours, and the velocity of what hit them.
    fn hit_bricks(&self, contacts: &[ContactEvent]) -> Vec<(u32, Vec3)> {
        let Some((wall, _)) = self.wall else {
            return Vec::new();
        };
        let mut hits = Vec::new();
        for event in contacts {
            let ContactEvent::Added {
                manifold,
                estimate: Some(estimate),
                ..
            } = event
            else {
                continue;
            };
            let impulse: f32 = estimate.contact_impulses.iter().sum();
            if impulse < BREAKING_IMPULSE {
                continue;
            }
            let pair = manifold.pair;
            let (sub_shape, other) = if pair.body1 == wall {
                (pair.sub_shape1, pair.body2)
            } else if pair.body2 == wall {
                (pair.sub_shape2, pair.body1)
            } else {
                continue;
            };
            // The bricks fly away from what hit them; it may already bounce back.
            let from = self.tracked.pose(other).map(|(p, _)| position(p));
            for brick in self
                .brick_of(sub_shape)
                .map(|brick| self.neighbours(brick))
                .unwrap_or_default()
            {
                let centre = self.brick_centre(brick);
                let away =
                    from.map_or(glam::Vec3::ZERO, |from| (centre - from).normalize_or_zero());
                hits.push((brick, vec3(away * KNOCK_SPEED)));
            }
        }
        hits
    }

    /// The world position of `brick`'s centre in the wall.
    fn brick_centre(&self, brick: u32) -> glam::Vec3 {
        let wall_at = self
            .wall
            .and_then(|(wall, _)| self.tracked.pose(wall))
            .map_or(glam::Vec3::from(WALL_AT), |(p, _)| position(p));
        let local = self
            .bricks
            .iter()
            .find(|&&(id, _)| id == brick)
            .map_or([0.0; 3], |&(_, at)| at);
        wall_at + glam::Vec3::from(local)
    }

    /// `brick` and the bricks whose centres are within [`BLAST_RADIUS`] of its centre.
    fn neighbours(&self, brick: u32) -> Vec<u32> {
        let Some(&(_, centre)) = self.bricks.iter().find(|&&(id, _)| id == brick) else {
            return Vec::new();
        };
        self.bricks
            .iter()
            .filter(|(_, at)| {
                glam::Vec3::from(*at).distance(glam::Vec3::from(centre)) <= BLAST_RADIUS
            })
            .map(|&(id, _)| id)
            .collect()
    }

    /// Knocks out `hits`, each a brick id and the velocity of what hit it: removes them from
    /// the compound (each once), publishes the rest to the wall, or removes the wall when no
    /// brick is left, and lets every brick fall on as a dynamic body.
    fn knock_out(&mut self, mut hits: Vec<(u32, Vec3)>) -> Result<()> {
        let Some((wall, wall_key)) = self.wall else {
            return Ok(());
        };
        hits.sort_by_key(|&(brick, _)| brick);
        hits.dedup_by_key(|&mut (brick, _)| brick);
        let mut indices: Vec<(usize, Vec3)> = hits
            .iter()
            .filter_map(|&(brick, velocity)| {
                let index = self.bricks.iter().position(|&(id, _)| id == brick)?;
                Some((index, velocity))
            })
            .collect();
        if indices.is_empty() {
            return Ok(());
        }
        let wall_at = self
            .tracked
            .pose(wall)
            .map_or(glam::Vec3::from(WALL_AT), |(p, _)| position(p));
        // Highest index first, so the indices still to remove stay valid.
        indices.sort_by_key(|&(index, _)| std::cmp::Reverse(index));
        let mut loose = Vec::new();
        for (index, velocity) in indices {
            self.editor.remove_shape(index as u32)?;
            let (_, local) = self.bricks.remove(index);
            loose.push((wall_at + glam::Vec3::from(local), velocity));
        }
        if self.bricks.is_empty() {
            self.tracked.remove(&mut self.world, wall)?;
            self.wall = None;
        } else {
            let shape = self.editor.to_shape()?;
            self.world
                .body_mut(wall)?
                .set_shape(&shape, None, oxijolt::Activation::Activate)?;
            let brick_visual = self.visuals.get(self.brick_visual).clone();
            let replaced = self
                .visuals
                .replace(wall_key, wall_visual(&brick_visual, &self.bricks));
            self.tracked.set_visual(wall, replaced);
            self.wall = Some((wall, replaced));
            self.publications += 1;
            self.milestones.reach("wall republished");
        }
        for (at, velocity) in loose {
            self.spawn_debris(at, velocity)?;
        }
        self.milestones.reach("brick knocked out");
        Ok(())
    }

    fn spawn_debris(&mut self, at: glam::Vec3, velocity: Vec3) -> Result<()> {
        if self.debris.len() >= MAX_DEBRIS {
            if let Some(oldest) = self.debris.pop_front() {
                self.tracked.remove(&mut self.world, oldest)?;
            }
        }
        let settings = BodySettings::new_dynamic()
            .position(rvec(at.to_array()))
            .linear_velocity(velocity)
            .object_layer(self.layers.moving)
            .mass(4.0);
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &self.brick_shape,
            self.brick_visual,
            &settings,
            colours::BODY,
        )?;
        self.debris.push_back(body);
        Ok(())
    }

    /// The brick under the ray of a click.
    fn picked_brick(&self, ray: RayCast) -> Result<Option<u32>> {
        let Some((wall, _)) = self.wall else {
            return Ok(None);
        };
        let hit = self.world.cast_ray(ray, &QueryFilter::new())?;
        Ok(hit
            .filter(|hit| hit.body == wall)
            .and_then(|hit| hit.compound_child)
            .map(|child| child.user_data))
    }

    /// Removes debris that fell below the world, in id order.
    fn clear_fallen(&mut self) -> Result<()> {
        let mut fallen: Vec<BodyId> = self
            .debris
            .iter()
            .copied()
            .filter(|&body| {
                self.tracked
                    .pose(body)
                    .is_some_and(|(p, _)| position_f32(p)[1] < -20.0)
            })
            .collect();
        fallen.sort();
        for body in fallen {
            self.tracked.remove(&mut self.world, body)?;
            self.debris.retain(|&debris| debris != body);
        }
        Ok(())
    }
}

/// The wall's description: one brick at each position.
fn wall_visual(brick: &Visual, bricks: &[(u32, [f32; 3])]) -> Visual {
    Visual::Compound(
        bricks
            .iter()
            .map(|&(_, at)| (brick.clone(), at, [0.0, 0.0, 0.0, 1.0]))
            .collect(),
    )
}

impl Scene for Destruction {
    fn update(&mut self, input: &Input) -> Result<()> {
        if input.edges.fire {
            let ray = input
                .held
                .aim
                .unwrap_or_else(|| self.camera().ray([0.0, 0.0], 16.0 / 9.0));
            self.fire(ray)?;
        }
        if let Some(ray) = input.edges.pick {
            if let Some(brick) = self.picked_brick(ray)? {
                let away = glam(ray.direction).normalize() * KNOCK_SPEED;
                self.knock_out(vec![(brick, vec3(away))])?;
            }
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        // Decode the hits against the shape the step collided with, before any edit.
        let hits = self.hit_bricks(&events.contacts);
        self.knock_out(hits)?;
        self.clear_fallen()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        out.hud.push(format!(
            "wall: {} bricks, published {} times; {} loose bricks",
            self.bricks.len(),
            self.publications,
            self.debris.len()
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) {
        self.tracked.write_state(digest);
        digest.u64(self.bricks.len() as u64);
        for (id, _) in &self.bricks {
            digest.u32(*id);
        }
        digest.u32(self.publications);
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.0, 0.8, 0.0], 0.5, 0.25, 6.5)
    }

    fn script(&self, tick: u32) -> Input {
        let mut input = Input::default();
        let shooter = |x: f32, y: f32| {
            CameraHint::new([x, y, 0.0], 0.35, 0.05, 8.0).ray([0.0, 0.0], 16.0 / 9.0)
        };
        input.held.aim = Some(match tick {
            0..=100 => shooter(-0.6, 0.9),
            _ => shooter(0.7, 0.5),
        });
        input.edges = Edges {
            fire: matches!(tick, 40 | 150),
            pick: (tick == 240).then(|| {
                CameraHint::new([-1.4, 1.1, 0.0], 0.0, 0.0, 5.0).ray([0.0, 0.0], 16.0 / 9.0)
            }),
            ..Edges::default()
        };
        input
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scene() -> Destruction {
        Destruction::new(
            &SceneConfig {
                worker_threads: Some(1),
            },
            1,
        )
        .unwrap()
    }

    #[test]
    fn duplicate_hits_remove_one_brick() {
        let mut scene = scene();
        let push = Vec3::new(0.0, 0.0, -5.0);
        scene
            .knock_out(vec![(5, push), (5, push), (5, push)])
            .unwrap();
        assert_eq!(scene.bricks.len(), 47);
        assert_eq!(scene.debris.len(), 1);
        assert!(scene.bricks.iter().all(|&(id, _)| id != 5));
        // A brick that is gone already changes nothing.
        scene.knock_out(vec![(5, push)]).unwrap();
        assert_eq!((scene.bricks.len(), scene.debris.len()), (47, 1));
    }

    #[test]
    fn destroying_every_brick_removes_the_wall() {
        let mut scene = scene();
        let bodies = scene.world.body_count();
        let every: Vec<(u32, Vec3)> = (0..48).map(|brick| (brick, Vec3::ZERO)).collect();
        scene.knock_out(every).unwrap();
        assert!(scene.wall.is_none());
        assert_eq!(scene.debris.len(), 48);
        assert_eq!(scene.world.body_count(), bodies - 1 + 48);
        for _ in 0..30 {
            scene.update(&Input::default()).unwrap();
        }
    }

    #[test]
    fn reset_after_complete_destruction_rebuilds_the_wall() {
        use crate::scene::SceneKind;
        use crate::session::Session;
        let config = SceneConfig {
            worker_threads: Some(1),
        };
        let mut session = Session::new(SceneKind::Destruction, config).unwrap();
        let bodies = session.scene().world().body_count();
        let mut scene = scene();
        let every: Vec<(u32, Vec3)> = (0..48).map(|brick| (brick, Vec3::ZERO)).collect();
        scene.knock_out(every).unwrap();
        assert!(scene.wall.is_none());
        session.reset().unwrap();
        assert_eq!(session.scene().world().body_count(), bodies);
        let mut list = DrawList::default();
        session.draw(&mut list).unwrap();
        assert!(list.hud[0].starts_with("wall: 48 bricks"));
    }

    #[test]
    fn a_hit_decodes_to_the_brick_at_its_child() {
        let scene = scene();
        let ray = RayCast::new(
            rvec([0.0, 2.0 * BRICK[1] * 2.5, 2.0]),
            Vec3::new(0.0, 0.0, -4.0),
        );
        let picked = scene.picked_brick(ray).unwrap().unwrap();
        let (_, at) = scene.bricks.iter().find(|&&(id, _)| id == picked).unwrap();
        assert!((at[1] - 2.0 * BRICK[1] * 2.5).abs() <= BRICK[1]);
        assert!(at[0].abs() <= 2.0 * BRICK[0]);
    }
}
