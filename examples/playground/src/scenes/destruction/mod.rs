//! A breakable wall: a dynamic body standing on the ground, whose shape is published from a
//! mutable compound of 48 bricks. Hard hits of cannonballs, and clicks, knock bricks out: the hit children are decoded from the contacts and removed from the compound,
//! and each brick falls on as a body. What is left splits into pieces of bricks that touch face
//! to face; the largest stays in the wall's body, every other piece becomes a body of its own,
//! and physics decides whether a piece stands, topples or falls.

use std::collections::VecDeque;

use oxijolt::{
    Activation, BodyId, BodySettings, CompoundChild, ContactEvent, EventSettings, MotionQuality,
    MutableCompound, PhysicsWorld, Quat, QueryFilter, RayCast, SubShapeId, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList};
use crate::input::{Edges, Input};
use crate::math::{glam, glam_quat, position, position_f32, quat, rvec, vec3};
use crate::scene::{new_world, step, Layers, Milestones, Result, Scene, SceneConfig};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visual, VisualKey, Visuals};

/// Bricks across and up, and a brick's half extents, metres.
const WALL: [u32; 2] = [8, 6];
const BRICK: [f32; 3] = [0.25, 0.12, 0.15];
/// The mass of a brick, kg: fired clay of about 2000 kg/m³. A piece weighs as much as its
/// bricks.
const BRICK_MASS: f32 = 72.0;
/// Where the wall stands.
const WALL_AT: [f32; 3] = [0.0, 0.0, 0.0];
/// Impulse in N·s from which a cannonball's hit knocks bricks out.
const BREAKING_IMPULSE: f32 = 150.0;
/// How far around a brick that is hit hard the bricks come loose with it, metres.
const BLAST_RADIUS: f32 = 0.55;
/// Speed of a brick knocked out, m/s.
const KNOCK_SPEED: f32 = 5.0;
/// The most cannonballs and loose bricks at once; the oldest go first.
const MAX_BALLS: usize = 10;
const MAX_DEBRIS: usize = 200;
/// How far apart two brick centres may be off the brick grid and still count as neighbours,
/// metres; the overlap below which two bricks of neighbouring rows do not touch.
const GRID_TOLERANCE: f32 = 0.01;

const MILESTONES: &[&str] = &["brick knocked out", "wall republished", "piece broke off"];
const CONTROLS: &[(&str, &str)] = &[
    ("F", "fire a cannonball at the cursor"),
    ("left click", "knock out the brick under the cursor"),
];

/// A brick: its id, which is its place in the laid wall, and its centre in its piece's body.
type Brick = (u32, [f32; 3]);

/// A piece of the wall: a dynamic body whose shape its own mutable compound publishes.
struct Piece {
    body: BodyId,
    visual: VisualKey,
    editor: MutableCompound,
    /// The piece's bricks, in the editor's child order.
    bricks: Vec<Brick>,
}

/// Where a body is and how it moves.
#[derive(Clone, Copy)]
struct Motion {
    position: glam::Vec3,
    rotation: glam::Quat,
    linear: glam::Vec3,
    angular: glam::Vec3,
}

impl Motion {
    fn of(world: &PhysicsWorld, body: BodyId) -> Result<Self> {
        let body = world.body(body)?;
        Ok(Self {
            position: position(body.position()),
            rotation: glam_quat(body.rotation()),
            linear: glam(body.linear_velocity()),
            angular: glam(body.angular_velocity()),
        })
    }

    /// The world position of `local`, a point of the body.
    fn at(&self, local: [f32; 3]) -> glam::Vec3 {
        self.position + self.rotation * glam::Vec3::from(local)
    }

    /// The velocity of the body's point at `point`, for a body whose centre of mass is at
    /// `centre`.
    fn velocity_at(&self, point: glam::Vec3, centre: glam::Vec3) -> glam::Vec3 {
        self.linear + self.angular.cross(point - centre)
    }
}

/// The destruction scene.
pub struct Destruction {
    world: PhysicsWorld,
    layers: Layers,
    visuals: Visuals,
    tracked: Tracked,
    brick: Shaped,
    brick_visual: VisualKey,
    /// The pieces in the order they broke off; the wall itself first.
    pieces: Vec<Piece>,
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
        let bricks: Vec<Brick> = (0..WALL[0] * WALL[1]).map(|id| (id, laid_at(id))).collect();
        let ball = Shaped::sphere(0.25)?;
        let ball_visual = visuals.add(ball.visual.clone());
        let brick_visual = visuals.add(brick.visual.clone());
        let mut scene = Self {
            world,
            layers,
            visuals,
            tracked,
            brick,
            brick_visual,
            pieces: Vec::new(),
            ball,
            ball_visual,
            balls: VecDeque::new(),
            debris: VecDeque::new(),
            publications: 0,
            milestones: Milestones::new(MILESTONES),
        };
        let standing = Motion {
            position: glam::Vec3::from(WALL_AT),
            rotation: glam::Quat::IDENTITY,
            linear: glam::Vec3::ZERO,
            angular: glam::Vec3::ZERO,
        };
        scene.spawn_piece(bricks, standing, glam::Vec3::from(WALL_AT))?;
        Ok(scene)
    }

    /// Creates a piece of `bricks` placed like a body with `motion` whose centre of mass is at
    /// `centre`, moving as that body's points move.
    fn spawn_piece(
        &mut self,
        bricks: Vec<Brick>,
        motion: Motion,
        centre: glam::Vec3,
    ) -> Result<()> {
        let children: Vec<CompoundChild<'_>> = bricks
            .iter()
            .map(|&(id, at)| CompoundChild {
                shape: &self.brick.shape,
                position: at.into(),
                rotation: Quat::IDENTITY,
                user_data: id,
            })
            .collect();
        let editor = MutableCompound::from_children(&children)?;
        let own_centre = motion.at(centroid(&bricks));
        let settings = BodySettings::new_dynamic()
            .position(rvec(motion.position.to_array()))
            .rotation(quat(motion.rotation))
            .linear_velocity(vec3(motion.velocity_at(own_centre, centre)))
            .angular_velocity(vec3(motion.angular))
            .object_layer(self.layers.moving)
            .mass(BRICK_MASS * bricks.len() as f32);
        let visual = self.visuals.add(piece_visual(&self.brick.visual, &bricks));
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &editor.to_shape()?,
            visual,
            &settings,
            colours::STRUCTURE,
        )?;
        self.pieces.push(Piece {
            body,
            visual,
            editor,
            bricks,
        });
        Ok(())
    }

    /// Fires a cannonball along `ray`.
    fn fire(&mut self, ray: RayCast) -> Result<()> {
        if self.balls.len() >= MAX_BALLS {
            if let Some(oldest) = self.balls.pop_front() {
                self.tracked.remove(&mut self.world, oldest)?;
            }
        }
        let direction = glam(ray.direction()).normalize();
        let settings = BodySettings::new_dynamic()
            .position(rvec((position(ray.origin()) + direction).to_array()))
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

    fn piece_of_body(&self, body: BodyId) -> Option<&Piece> {
        self.pieces.iter().find(|piece| piece.body == body)
    }

    /// The piece that holds `brick`, with the brick's centre in the piece.
    fn piece_of_brick(&self, brick: u32) -> Option<(&Piece, [f32; 3])> {
        self.pieces.iter().find_map(|piece| {
            let &(_, at) = piece.bricks.iter().find(|&&(id, _)| id == brick)?;
            Some((piece, at))
        })
    }

    /// The brick of `body`, a piece, that `sub_shape` of its current shape leads to.
    fn brick_of(&self, body: BodyId, sub_shape: SubShapeId) -> Option<u32> {
        let child = self.world.compound_sub_shape(body, sub_shape).ok()??;
        Some(child.user_data)
    }

    /// The bricks hit hard enough by a cannonball in the step's new contacts, decoded against
    /// each piece's current shape, with their neighbours, and the velocity they fly off with.
    fn hit_bricks(&self, contacts: &[ContactEvent]) -> Vec<(u32, Vec3)> {
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
            let (piece, sub_shape, other) = if self.piece_of_body(pair.body1).is_some() {
                (pair.body1, pair.sub_shape1, pair.body2)
            } else if self.piece_of_body(pair.body2).is_some() {
                (pair.body2, pair.sub_shape2, pair.body1)
            } else {
                continue;
            };
            if !self.balls.contains(&other) {
                continue;
            }
            // The bricks fly away from what hit them; it may already bounce back.
            let from = self.tracked.pose(other).map(|(p, _)| position(p));
            for brick in self
                .brick_of(piece, sub_shape)
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

    /// The world position of `brick`'s centre.
    fn brick_centre(&self, brick: u32) -> glam::Vec3 {
        let Some((piece, at)) = self.piece_of_brick(brick) else {
            return glam::Vec3::from(WALL_AT);
        };
        self.tracked
            .pose(piece.body)
            .map_or(glam::Vec3::from(at), |(p, q)| {
                position(p) + glam_quat(q) * glam::Vec3::from(at)
            })
    }

    /// `brick` and the bricks of its piece whose centres are within [`BLAST_RADIUS`] of its
    /// centre.
    fn neighbours(&self, brick: u32) -> Vec<u32> {
        let Some((piece, centre)) = self.piece_of_brick(brick) else {
            return Vec::new();
        };
        piece
            .bricks
            .iter()
            .filter(|(_, at)| {
                glam::Vec3::from(*at).distance(glam::Vec3::from(centre)) <= BLAST_RADIUS
            })
            .map(|&(id, _)| id)
            .collect()
    }

    /// Knocks out `hits`, each a brick id and the velocity it flies off with: removes them from
    /// their pieces (each once), lets every brick fall on as a body and splits each piece that
    /// lost bricks into the pieces of what is left.
    fn knock_out(&mut self, mut hits: Vec<(u32, Vec3)>) -> Result<()> {
        hits.sort_by_key(|&(brick, _)| brick);
        hits.dedup_by_key(|&mut (brick, _)| brick);
        let mut knocked = false;
        // Pieces split in their order; the pieces they make go after the others.
        for index in 0..self.pieces.len() {
            let body = self.pieces[index].body;
            let mine: Vec<(u32, Vec3)> = hits
                .iter()
                .copied()
                .filter(|&(brick, _)| self.pieces[index].bricks.iter().any(|&(id, _)| id == brick))
                .collect();
            if mine.is_empty() {
                continue;
            }
            let motion = Motion::of(&self.world, body)?;
            let centre = motion.at(centroid(&self.pieces[index].bricks));
            for (brick, velocity) in mine {
                let at = self.remove_brick(index, brick)?;
                self.spawn_debris(motion.at(at), motion.rotation, glam(velocity))?;
            }
            knocked = true;
            self.split(index, motion, centre)?;
        }
        self.pieces.retain(|piece| !piece.bricks.is_empty());
        if knocked {
            self.milestones.reach("brick knocked out");
        }
        Ok(())
    }

    /// Removes `brick` from piece `index`'s editor and list and returns its centre in the piece.
    fn remove_brick(&mut self, index: usize, brick: u32) -> Result<[f32; 3]> {
        let piece = &mut self.pieces[index];
        let child = piece
            .bricks
            .iter()
            .position(|&(id, _)| id == brick)
            .ok_or("the brick is not in the piece")?;
        piece.editor.remove_shape(child as u32)?;
        Ok(piece.bricks.remove(child).1)
    }

    /// Splits piece `index`, whose body moved with `motion` around `centre` before it lost
    /// bricks, into the groups of its bricks that touch face to face. The largest group (the
    /// first of the largest, in the order of their lowest brick ids) stays in the piece's body,
    /// which the rest of its compound is published to; other groups of two or more become new
    /// pieces and single bricks fall on as loose bricks. A piece of no group of two loses its
    /// body.
    fn split(&mut self, index: usize, motion: Motion, centre: glam::Vec3) -> Result<()> {
        let groups = touching_groups(&self.pieces[index].bricks);
        let kept = groups
            .iter()
            .enumerate()
            .filter(|(_, group)| group.len() >= 2)
            .max_by_key(|&(order, group)| (group.len(), std::cmp::Reverse(order)))
            .map(|(order, _)| order);
        for (order, group) in groups.iter().enumerate() {
            if Some(order) == kept {
                continue;
            }
            let bricks: Vec<Brick> = group
                .iter()
                .map(|&brick| self.remove_brick(index, brick).map(|at| (brick, at)))
                .collect::<Result<_>>()?;
            if let [(_, at)] = bricks.as_slice() {
                let point = motion.at(*at);
                self.spawn_debris(point, motion.rotation, motion.velocity_at(point, centre))?;
            } else {
                self.spawn_piece(bricks, motion, centre)?;
                self.milestones.reach("piece broke off");
            }
        }
        let body = self.pieces[index].body;
        if kept.is_none() {
            self.tracked.remove(&mut self.world, body)?;
            return Ok(());
        }
        let piece = &self.pieces[index];
        let shape = piece.editor.to_shape()?;
        let mass = BRICK_MASS * piece.bricks.len() as f32;
        let visual = piece_visual(&self.brick.visual, &piece.bricks);
        self.world
            .body_mut(body)?
            .set_shape(&shape, Some(mass), Activation::Activate)?;
        let replaced = self.visuals.replace(self.pieces[index].visual, visual);
        self.tracked.set_visual(body, replaced);
        self.pieces[index].visual = replaced;
        self.publications += 1;
        self.milestones.reach("wall republished");
        Ok(())
    }

    fn spawn_debris(
        &mut self,
        at: glam::Vec3,
        rotation: glam::Quat,
        velocity: glam::Vec3,
    ) -> Result<()> {
        if self.debris.len() >= MAX_DEBRIS {
            if let Some(oldest) = self.debris.pop_front() {
                self.tracked.remove(&mut self.world, oldest)?;
            }
        }
        let settings = BodySettings::new_dynamic()
            .position(rvec(at.to_array()))
            .rotation(quat(rotation))
            .linear_velocity(vec3(velocity))
            .object_layer(self.layers.moving)
            .mass(BRICK_MASS);
        let body = self.tracked.spawn_keyed(
            &mut self.world,
            &self.brick.shape,
            self.brick_visual,
            &settings,
            colours::BODY,
        )?;
        self.debris.push_back(body);
        Ok(())
    }

    /// The brick under the ray of a click.
    fn picked_brick(&self, ray: RayCast) -> Result<Option<u32>> {
        let hit = self.world.cast_ray(&ray, &QueryFilter::new())?;
        Ok(hit
            .filter(|hit| self.piece_of_body(hit.body).is_some())
            .and_then(|hit| hit.compound_child)
            .map(|child| child.user_data))
    }

    /// Removes loose bricks and pieces that fell below the world, in id order.
    fn clear_fallen(&mut self) -> Result<()> {
        let below = |body: BodyId| {
            self.tracked
                .pose(body)
                .is_some_and(|(p, _)| position_f32(p)[1] < -20.0)
        };
        let mut fallen: Vec<BodyId> = self.debris.iter().copied().filter(|&b| below(b)).collect();
        fallen.extend(
            self.pieces
                .iter()
                .map(|piece| piece.body)
                .filter(|&b| below(b)),
        );
        fallen.sort();
        for body in fallen {
            self.tracked.remove(&mut self.world, body)?;
            self.debris.retain(|&debris| debris != body);
            self.pieces.retain(|piece| piece.body != body);
        }
        Ok(())
    }

    /// The bricks the pieces hold.
    fn brick_count(&self) -> usize {
        self.pieces.iter().map(|piece| piece.bricks.len()).sum()
    }
}

/// Where brick `id` (row by row from the bottom left) sits in the laid wall.
fn laid_at(id: u32) -> [f32; 3] {
    let (row, column) = (id / WALL[0], id % WALL[0]);
    // Every other row is offset by a quarter brick, like a laid wall.
    let shift = if row % 2 == 0 { -0.06 } else { 0.06 };
    let x = (column as f32 - (WALL[0] - 1) as f32 / 2.0) * 2.0 * BRICK[0] + shift;
    let y = BRICK[1] + row as f32 * 2.0 * BRICK[1];
    [x, y, 0.0]
}

/// A click from the front at brick `id` of the laid wall.
fn click_at(id: u32) -> RayCast {
    let [x, y, z] = laid_at(id);
    let [wx, wy, wz] = WALL_AT;
    RayCast::new(
        rvec([wx + x, wy + y, wz + z + 3.0]),
        Vec3::new(0.0, 0.0, -6.0),
    )
}

/// The bricks the script clicks after the cannonballs, which cut the top right corner loose,
/// and the tick of the first; one click every six ticks.
const CORNER_CUT: [u32; 5] = [29, 30, 31, 37, 45];
const CORNER_CUT_FROM: u32 = 250;

/// The mean of the brick centres, the centre of mass of equal bricks.
fn centroid(bricks: &[Brick]) -> [f32; 3] {
    let sum = bricks
        .iter()
        .fold(glam::Vec3::ZERO, |sum, &(_, at)| sum + glam::Vec3::from(at));
    (sum / bricks.len().max(1) as f32).to_array()
}

/// Whether two bricks of one piece touch face to face: side by side in a row, or in
/// neighbouring rows with overlapping ends.
fn touch(a: [f32; 3], b: [f32; 3]) -> bool {
    let (dx, dy) = ((a[0] - b[0]).abs(), (a[1] - b[1]).abs());
    let side_by_side = dy < GRID_TOLERANCE && (dx - 2.0 * BRICK[0]).abs() < GRID_TOLERANCE;
    let stacked =
        (dy - 2.0 * BRICK[1]).abs() < GRID_TOLERANCE && dx < 2.0 * BRICK[0] - GRID_TOLERANCE;
    side_by_side || stacked
}

/// The groups of `bricks` that touch face to face, each by ascending id, ordered by their
/// lowest id.
fn touching_groups(bricks: &[Brick]) -> Vec<Vec<u32>> {
    let mut sorted = bricks.to_vec();
    sorted.sort_by_key(|&(id, _)| id);
    let mut group_of = vec![usize::MAX; sorted.len()];
    let mut groups = Vec::new();
    for start in 0..sorted.len() {
        if group_of[start] != usize::MAX {
            continue;
        }
        let group = groups.len();
        group_of[start] = group;
        let mut members = vec![start];
        let mut next = 0;
        while next < members.len() {
            let at = sorted[members[next]].1;
            next += 1;
            for other in 0..sorted.len() {
                if group_of[other] == usize::MAX && touch(at, sorted[other].1) {
                    group_of[other] = group;
                    members.push(other);
                }
            }
        }
        members.sort_unstable();
        groups.push(members.into_iter().map(|member| sorted[member].0).collect());
    }
    groups
}

/// A piece's description: one brick at each position.
fn piece_visual(brick: &Visual, bricks: &[Brick]) -> Visual {
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
                let away = glam(ray.direction()).normalize() * KNOCK_SPEED;
                self.knock_out(vec![(brick, vec3(away))])?;
            }
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        // Decode the hits against the shapes the step collided with, before any edit.
        let hits = self.hit_bricks(&events.contacts);
        self.knock_out(hits)?;
        self.clear_fallen()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        out.hud.push(format!(
            "wall: {} bricks, published {} times; {} pieces, {} loose bricks",
            self.brick_count(),
            self.publications,
            self.pieces.len(),
            self.debris.len()
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        digest.u64(self.pieces.len() as u64);
        for piece in &self.pieces {
            digest.u32(piece.body.to_raw());
            digest.u64(piece.bricks.len() as u64);
            for (id, _) in &piece.bricks {
                digest.u32(*id);
            }
        }
        digest.u32(self.publications);
        Ok(())
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn world_mut(&mut self) -> &mut PhysicsWorld {
        &mut self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.0, 0.8, 0.0], 0.5, 0.25, 6.5)
    }

    /// Close on the wall, from the side the cannonballs come from.
    fn record_camera(&self, _tick: u32) -> CameraHint {
        CameraHint::new([0.0, 0.7, 0.0], 0.4, 0.15, 4.2)
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
        let cut = tick
            .checked_sub(CORNER_CUT_FROM)
            .filter(|after| after % 6 == 0)
            .and_then(|after| CORNER_CUT.get((after / 6) as usize));
        input.edges = Edges {
            fire: matches!(tick, 40 | 150),
            pick: match tick {
                240 => Some(
                    CameraHint::new([-1.4, 1.1, 0.0], 0.0, 0.0, 5.0).ray([0.0, 0.0], 16.0 / 9.0),
                ),
                _ => cut.map(|&brick| click_at(brick)),
            },
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
                ..SceneConfig::default()
            },
            1,
        )
        .unwrap()
    }

    /// Pushes knocked-out bricks out of the way, behind the wall.
    const OUT: Vec3 = Vec3::new(0.0, 0.0, -KNOCK_SPEED);

    fn run(scene: &mut Destruction, ticks: u32) {
        for _ in 0..ticks {
            scene.update(&Input::default()).unwrap();
        }
    }

    /// The angle in radians by which `body` is turned from where it started.
    fn tilt(scene: &Destruction, body: BodyId) -> f32 {
        glam_quat(scene.world.body(body).unwrap().rotation()).angle_between(glam::Quat::IDENTITY)
    }

    #[test]
    fn the_wall_stands_on_the_ground() {
        let mut scene = scene();
        let wall = scene.pieces[0].body;
        run(&mut scene, 120);
        assert!(tilt(&scene, wall) < 0.01, "{}", tilt(&scene, wall));
        let y = position(scene.world.body(wall).unwrap().position()).y;
        assert!(y.abs() < 0.03, "{y}");
    }

    #[test]
    fn duplicate_hits_remove_one_brick() {
        let mut scene = scene();
        scene.knock_out(vec![(5, OUT), (5, OUT), (5, OUT)]).unwrap();
        assert_eq!(scene.brick_count(), 47);
        assert_eq!((scene.pieces.len(), scene.debris.len()), (1, 1));
        assert!(scene.piece_of_brick(5).is_none());
        // A brick that is gone already changes nothing.
        scene.knock_out(vec![(5, OUT)]).unwrap();
        assert_eq!((scene.brick_count(), scene.debris.len()), (47, 1));
    }

    #[test]
    fn destroying_every_brick_removes_the_wall() {
        let mut scene = scene();
        let bodies = scene.world.body_count();
        let every: Vec<(u32, Vec3)> = (0..48).map(|brick| (brick, Vec3::ZERO)).collect();
        scene.knock_out(every).unwrap();
        assert!(scene.pieces.is_empty());
        assert_eq!(scene.debris.len(), 48);
        assert_eq!(scene.world.body_count(), bodies - 1 + 48);
        run(&mut scene, 30);
    }

    /// Without its bottom row but the rightmost brick, the wall stands on one brick at its
    /// right end, its centre of mass far beside that support, and falls over to the left until
    /// its open end lies on the ground.
    #[test]
    fn a_wall_on_one_brick_falls_over() {
        let mut scene = scene();
        let wall = scene.pieces[0].body;
        let bottom: Vec<(u32, Vec3)> = (0..WALL[0] - 1).map(|brick| (brick, OUT)).collect();
        scene.knock_out(bottom).unwrap();
        assert_eq!(scene.pieces.len(), 1, "the wall stays one piece");
        assert_eq!(scene.pieces[0].body, wall);
        // The bottom left corner of brick 8, now the left end of the lowest row.
        let (_, [x, y, _]) = scene.piece_of_brick(8).unwrap();
        let open_end = [x - BRICK[0], y - BRICK[1], 0.0];
        run(&mut scene, 150);
        let motion = Motion::of(&scene.world, wall).unwrap();
        assert!(motion.at(open_end).y < 0.03, "{}", motion.at(open_end));
        assert!(tilt(&scene, wall) > 0.05, "{}", tilt(&scene, wall));
    }

    /// Bricks cut out around the top right corner leave it a piece of its own, which falls as
    /// one body onto the row below the cut while the wall stands.
    #[test]
    fn a_cut_loose_corner_falls_as_one_body() {
        let mut scene = scene();
        let wall = scene.pieces[0].body;
        let cut: Vec<(u32, Vec3)> = CORNER_CUT.map(|brick| (brick, OUT)).to_vec();
        scene.knock_out(cut).unwrap();
        assert_eq!(scene.pieces.len(), 2);
        let corner = scene.pieces[1].body;
        let ids: Vec<u32> = scene.pieces[1].bricks.iter().map(|&(id, _)| id).collect();
        assert_eq!(ids, [38, 39, 46, 47]);
        assert_eq!(scene.pieces[0].bricks.len(), 48 - 5 - 4);
        let start = position(scene.world.body(corner).unwrap().position());
        run(&mut scene, 60);
        let now = position(scene.world.body(corner).unwrap().position());
        assert!(start.y - now.y > 0.2, "{start} {now}");
        assert_eq!(scene.pieces[1].bricks.len(), 4);
        assert!(tilt(&scene, wall) < 0.05, "{}", tilt(&scene, wall));
    }

    #[test]
    fn groups_follow_face_contact_in_the_laid_wall() {
        let scene = scene();
        let bricks = &scene.pieces[0].bricks;
        assert_eq!(touching_groups(bricks), vec![(0..48).collect::<Vec<u32>>()]);
        // Brick 8 (row 1, column 0) is shifted right, so it touches bricks 0 and 1 below it
        // but not brick 2.
        let at = |id: u32| bricks.iter().find(|&&(b, _)| b == id).unwrap().1;
        assert!(touch(at(8), at(0)) && touch(at(8), at(1)) && !touch(at(8), at(2)));
        assert!(touch(at(0), at(1)) && !touch(at(0), at(2)));
        let split: Vec<Brick> = bricks
            .iter()
            .copied()
            .filter(|&(id, _)| !CORNER_CUT.contains(&id))
            .collect();
        let groups = touching_groups(&split);
        assert_eq!(groups.len(), 2);
        assert_eq!(groups[1], [38, 39, 46, 47]);
    }

    /// The digest of `ticks` scripted ticks of `session`.
    fn scripted(session: &mut crate::session::Session, ticks: u32) -> u64 {
        let mut digest = Digest::default();
        let mut list = DrawList::default();
        for _ in 0..ticks {
            session.tick_scripted().unwrap();
            session.write_state(&mut digest).unwrap();
            session.draw(&mut list).unwrap();
            digest.draw_list(&list);
        }
        digest.finish()
    }

    #[test]
    fn reset_after_complete_destruction_rebuilds_the_wall() {
        use crate::scene::SceneKind;
        use crate::session::Session;
        let config = SceneConfig {
            worker_threads: Some(1),
            ..SceneConfig::default()
        };
        let fresh_bodies = scene().world.body_count();
        let mut fresh = Session::new(SceneKind::Destruction, config.clone()).unwrap();
        let first = scripted(&mut fresh, 60);
        let mut session = Session::new(SceneKind::Destruction, config).unwrap();
        // Click every brick from the front, top row first, so that what is left stands on a
        // full bottom row until the last brick goes.
        let mut bricks = scene().pieces.remove(0).bricks;
        bricks.sort_by_key(|&(id, _)| std::cmp::Reverse(id / WALL[0]));
        let hud = |session: &mut Session| {
            let mut list = DrawList::default();
            session.draw(&mut list).unwrap();
            list.hud[0].clone()
        };
        for &(brick, _) in &bricks {
            let input = Input {
                edges: Edges {
                    pick: Some(click_at(brick)),
                    ..Edges::default()
                },
                ..Input::default()
            };
            session.tick(input).unwrap();
        }
        assert!(
            hud(&mut session).starts_with("wall: 0 bricks"),
            "{}",
            hud(&mut session)
        );
        assert!(session.scene().world().body_count() > fresh_bodies);

        session.reset().unwrap();
        assert_eq!(session.scene().world().body_count(), fresh_bodies);
        assert!(hud(&mut session).starts_with("wall: 48 bricks, published 0 times"));
        assert_eq!(scripted(&mut session, 60), first);
    }

    #[test]
    fn a_hit_decodes_to_the_brick_at_its_child() {
        let scene = scene();
        let ray = RayCast::new(
            rvec([0.0, 2.0 * BRICK[1] * 2.5, 2.0]),
            Vec3::new(0.0, 0.0, -4.0),
        );
        let picked = scene.picked_brick(ray).unwrap().unwrap();
        let (_, at) = scene.piece_of_brick(picked).unwrap();
        assert!((at[1] - 2.0 * BRICK[1] * 2.5).abs() <= BRICK[1]);
        assert!(at[0].abs() <= 2.0 * BRICK[0]);
    }
}
