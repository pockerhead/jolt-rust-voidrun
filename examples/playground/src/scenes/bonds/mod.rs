//! A wall of separate bricks held together by breakable bonds: fixed constraints between
//! neighbouring bricks, and anchors of the bottom bricks to a pier that only clicks break. The
//! wall spans from the pier to a kinematic prop; when the prop is lowered away, the wall hangs
//! from the pier and the bonds near it carry its weight. After every step each bond's impulse
//! readouts are turned into a force and a torque, and a bond loaded beyond its limit is
//! disabled, so the wall cracks at the pier and its free end falls.

use oxijolt::{
    BodyId, BodySettings, ConstraintId, FixedConstraint, FixedConstraintSettings, PhysicsWorld,
    Quat, QueryFilter, RayCast, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, Colour, DrawList};
use crate::input::Input;
use crate::math::{glam, glam_quat, position, rvec};
use crate::scene::{new_world, step, Milestones, Result, Scene, SceneConfig, DT};
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visuals};

/// Bricks across and up, and a brick's half extents, metres.
const WALL: [u32; 2] = [8, 6];
const BRICK: [f32; 3] = [0.25, 0.12, 0.15];
/// The mass of a brick, kg: fired clay of about 2000 kg/m³.
const BRICK_MASS: f32 = 72.0;
/// The height of the pier and the prop the wall stands on, metres.
const PIER_HEIGHT: f32 = 1.5;
/// The bottom-row columns the pier holds up from the left and the prop from the right.
const PIER_COLUMNS: u32 = 2;
const PROP_COLUMNS: u32 = 2;
/// The force in N and the torque in N·m beyond which a bond breaks: its readouts over the step,
/// divided by the step's time. Measured with unbreakable bonds, the propped wall loads a bond
/// with at most 8.0 kN and 0.42 kN·m, and the bonds at the pier reach 98 kN and 7.5 kN·m once
/// the prop is lowered.
const MAX_BOND_FORCE: f32 = 20_000.0;
const MAX_BOND_TORQUE: f32 = 1_500.0;
/// The most bonds that break in one step, the most loaded first.
const MAX_BREAKS_PER_STEP: usize = 2;
/// The tick from which the prop is lowered, over how many ticks, and by how far in metres.
const RELEASE_FROM: u32 = 90;
const RELEASE_TICKS: u32 = 45;
const RELEASE_DROP: f32 = PIER_HEIGHT;
/// How many ticks a broken bond is drawn in the highlight colour.
const FLASH_TICKS: u32 = 20;

const MILESTONES: &[&str] = &["wall stands", "bond broke", "wall broke in two"];
const CONTROLS: &[(&str, &str)] = &[(
    "left click",
    "break every bond of the brick under the cursor",
)];

/// One side of a bond.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum End {
    /// A brick by its place in the laid wall, row by row from the bottom left.
    Brick(u32),
    /// The static pier.
    Pier,
}

/// A fixed constraint between two neighbouring bricks, or a bottom brick and the pier.
struct Bond {
    id: ConstraintId<FixedConstraint>,
    /// Body 1 and body 2 of the constraint.
    ends: [End; 2],
    /// The weld point on the wall's front face and the direction of the joint there, in the
    /// frame of end 1, for drawing.
    at: glam::Vec3,
    along: glam::Vec3,
    /// Ticks left to draw the bond as broken.
    flash: u32,
}

/// The bonds scene.
pub struct Bonds {
    world: PhysicsWorld,
    visuals: Visuals,
    tracked: Tracked,
    pier: BodyId,
    prop: BodyId,
    /// The bricks' bodies, by their place in the laid wall.
    bricks: Vec<BodyId>,
    /// The bonds in creation order, which is constraint id order.
    bonds: Vec<Bond>,
    max_force: f32,
    max_torque: f32,
    tick: u32,
    /// Bonds the load broke, and bonds clicks broke.
    load_breaks: u32,
    click_breaks: u32,
    /// The highest load of an intact bond between bricks in the last step, as a share of its
    /// limit.
    highest_load: f32,
    milestones: Milestones,
}

impl Bonds {
    /// Builds the ground, the pier, the prop and the bonded wall.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        Self::with_limits(config, generation, MAX_BOND_FORCE, MAX_BOND_TORQUE)
    }

    /// The scene with bonds that break beyond `max_force` in N or `max_torque` in N·m.
    fn with_limits(
        config: &SceneConfig,
        generation: u64,
        max_force: f32,
        max_torque: f32,
    ) -> Result<Self> {
        let (mut world, layers) = new_world(config, 3, oxijolt::EventSettings::default())?;
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
        let support = Shaped::cuboid([
            PIER_COLUMNS as f32 * BRICK[0],
            PIER_HEIGHT / 2.0,
            BRICK[2] * 1.5,
        ])?;
        let pier = BodySettings::new_static()
            .position(rvec(support_at(0, PIER_COLUMNS)))
            .object_layer(layers.ground);
        let pier = tracked.spawn(
            &mut world,
            &support,
            &pier,
            &mut visuals,
            colours::STRUCTURE,
        )?;
        let prop = BodySettings::new_kinematic()
            .position(rvec(support_at(WALL[0] - PROP_COLUMNS, WALL[0])))
            .object_layer(layers.moving);
        let prop = tracked.spawn(
            &mut world,
            &support,
            &prop,
            &mut visuals,
            colours::KINEMATIC,
        )?;
        let brick = Shaped::cuboid(BRICK)?;
        let brick_visual = visuals.add(brick.visual.clone());
        let mut bricks = Vec::new();
        for id in 0..WALL[0] * WALL[1] {
            let settings = BodySettings::new_dynamic()
                .position(rvec(laid_at(id)))
                .object_layer(layers.moving)
                .mass(BRICK_MASS);
            bricks.push(tracked.spawn_keyed(
                &mut world,
                &brick.shape,
                brick_visual,
                &settings,
                colours::BODY,
            )?);
        }
        let mut scene = Self {
            world,
            visuals,
            tracked,
            pier,
            prop,
            bricks,
            bonds: Vec::new(),
            max_force,
            max_torque,
            tick: 0,
            load_breaks: 0,
            click_breaks: 0,
            highest_load: 0.0,
            milestones: Milestones::new(MILESTONES),
        };
        for (ends, at, along) in bond_layout() {
            scene.bond(ends, at, along)?;
        }
        Ok(scene)
    }

    fn body(&self, end: End) -> BodyId {
        match end {
            End::Brick(brick) => self.bricks[brick as usize],
            End::Pier => self.pier,
        }
    }

    /// Welds `ends` at `at`, a world point on the face they share, whose joint runs `along`.
    fn bond(&mut self, ends: [End; 2], at: glam::Vec3, along: glam::Vec3) -> Result<()> {
        let settings = FixedConstraintSettings::new(
            rvec(at.to_array()),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        let id = self
            .world
            .create_constraint(self.body(ends[0]), self.body(ends[1]), &settings)?;
        let origin = position(self.world.body(self.body(ends[0]))?.position());
        let front = glam::Vec3::new(0.0, 0.0, BRICK[2] + 0.005);
        self.bonds.push(Bond {
            id,
            ends,
            at: at - origin + front,
            along,
            flash: 0,
        });
        Ok(())
    }

    /// The force in N and the torque in N·m that bond `index` applied over the last step.
    fn force_and_torque(&self, index: usize) -> Result<(f32, f32)> {
        let constraint = self.world.constraint(self.bonds[index].id)?;
        let force = glam(constraint.total_lambda_position()).length() / DT;
        let torque = glam(constraint.total_lambda_rotation()).length() / DT;
        Ok((force, torque))
    }

    /// The load of bond `index` in the last step, as a share of its limit: the larger of its
    /// force over [`MAX_BOND_FORCE`] and its torque over [`MAX_BOND_TORQUE`].
    fn load(&self, index: usize) -> Result<f32> {
        let (force, torque) = self.force_and_torque(index)?;
        Ok((force / self.max_force).max(torque / self.max_torque))
    }

    fn is_intact(&self, index: usize) -> Result<bool> {
        Ok(self.world.constraint(self.bonds[index].id)?.is_enabled())
    }

    /// Disables bond `index` and starts its flash.
    fn break_bond(&mut self, index: usize) -> Result<()> {
        self.world
            .constraint_mut(self.bonds[index].id)?
            .set_enabled(false);
        self.bonds[index].flash = FLASH_TICKS;
        Ok(())
    }

    /// Breaks the most loaded of the intact bonds between bricks whose load in the last step was
    /// beyond their limit, at most [`MAX_BREAKS_PER_STEP`]; equal loads break in constraint id
    /// order. The bricks anchored to the pier stay anchored.
    fn break_overloaded(&mut self) -> Result<()> {
        let mut overloaded = Vec::new();
        self.highest_load = 0.0;
        for index in 0..self.bonds.len() {
            if self.bonds[index].ends.contains(&End::Pier) || !self.is_intact(index)? {
                continue;
            }
            let load = self.load(index)?;
            self.highest_load = self.highest_load.max(load);
            if load > 1.0 {
                overloaded.push((index, load));
            }
        }
        overloaded.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
        for &(index, _) in overloaded.iter().take(MAX_BREAKS_PER_STEP) {
            self.break_bond(index)?;
            self.load_breaks += 1;
            self.milestones.reach("bond broke");
        }
        Ok(())
    }

    /// Lowers the prop by [`RELEASE_DROP`] over [`RELEASE_TICKS`] ticks from [`RELEASE_FROM`],
    /// then stops it; at the start every brick is woken, since a kinematic body that moves away
    /// does not wake the bodies it stops touching.
    fn move_prop(&mut self) -> Result<()> {
        let Some(lowered) = self.tick.checked_sub(RELEASE_FROM) else {
            return Ok(());
        };
        if lowered > RELEASE_TICKS {
            return Ok(());
        }
        if lowered == 0 {
            for &brick in &self.bricks {
                self.world.body_mut(brick)?.activate();
            }
        }
        let share = (lowered + 1).min(RELEASE_TICKS) as f32 / RELEASE_TICKS as f32;
        let mut at = support_at(WALL[0] - PROP_COLUMNS, WALL[0]);
        at[1] -= RELEASE_DROP * share;
        self.world
            .body_mut(self.prop)?
            .move_kinematic(rvec(at), Quat::IDENTITY, DT)?;
        Ok(())
    }

    /// The brick under the ray of a click.
    fn picked_brick(&self, ray: RayCast) -> Result<Option<u32>> {
        let hit = self.world.cast_ray(&ray, &QueryFilter::new())?;
        Ok(hit.and_then(|hit| {
            let brick = self.bricks.iter().position(|&body| body == hit.body)?;
            Some(brick as u32)
        }))
    }

    /// Breaks every intact bond of `brick`.
    fn break_brick(&mut self, brick: u32) -> Result<()> {
        for index in 0..self.bonds.len() {
            if self.bonds[index].ends.contains(&End::Brick(brick)) && self.is_intact(index)? {
                self.break_bond(index)?;
                self.click_breaks += 1;
            }
        }
        Ok(())
    }

    /// Whether some bricks are held to the pier by no chain of intact bonds.
    fn is_broken_in_two(&self) -> Result<bool> {
        let pier = self.bricks.len();
        let mut groups = Groups::new(pier + 1);
        for index in 0..self.bonds.len() {
            if self.is_intact(index)? {
                let [a, b] = self.bonds[index].ends.map(|end| match end {
                    End::Brick(brick) => brick as usize,
                    End::Pier => pier,
                });
                groups.join(a, b);
            }
        }
        let held = groups.root(pier);
        Ok((0..pier).any(|brick| groups.root(brick) != held))
    }

    /// The bonds intact and broken.
    fn counts(&self) -> Result<(usize, usize)> {
        let mut intact = 0;
        for index in 0..self.bonds.len() {
            intact += usize::from(self.is_intact(index)?);
        }
        Ok((intact, self.bonds.len() - intact))
    }

    /// The colour of an intact bond with `load`, a share of its limit.
    fn load_colour(load: f32) -> Colour {
        if load < 0.5 {
            colours::BODY_THIRD
        } else {
            colours::PLAYER
        }
    }
}

/// Where brick `id` (row by row from the bottom left) sits in the laid wall on the pier.
fn laid_at(id: u32) -> [f32; 3] {
    let (row, column) = (id / WALL[0], id % WALL[0]);
    // Every other row is offset by a quarter brick, like a laid wall.
    let shift = if row % 2 == 0 { -0.06 } else { 0.06 };
    let x = (column as f32 - (WALL[0] - 1) as f32 / 2.0) * 2.0 * BRICK[0] + shift;
    let y = PIER_HEIGHT + BRICK[1] + row as f32 * 2.0 * BRICK[1];
    [x, y, 0.0]
}

/// The centre of a support box under bottom-row columns `from..to`.
fn support_at(from: u32, to: u32) -> [f32; 3] {
    let left = laid_at(from)[0] - BRICK[0];
    let right = laid_at(to - 1)[0] + BRICK[0];
    [(left + right) / 2.0, PIER_HEIGHT / 2.0, 0.0]
}

/// Every bond of the wall, in the order they are made: for each brick by id, its bond to the
/// pier, to its right neighbour, and to the bricks above that overlap it, left first. Each is
/// its ends, its weld point at the centre of the face they share and the direction of that
/// face's joint. The pairs come from the brick grid, so a gap between bricks would not drop a
/// bond.
fn bond_layout() -> Vec<([End; 2], glam::Vec3, glam::Vec3)> {
    let mut bonds = Vec::new();
    for id in 0..WALL[0] * WALL[1] {
        let (row, column) = (id / WALL[0], id % WALL[0]);
        let centre = glam::Vec3::from(laid_at(id));
        if row == 0 && column < PIER_COLUMNS {
            let at = centre - glam::Vec3::Y * BRICK[1];
            bonds.push(([End::Pier, End::Brick(id)], at, glam::Vec3::X));
        }
        if column + 1 < WALL[0] {
            let at = centre + glam::Vec3::X * BRICK[0];
            bonds.push(([End::Brick(id), End::Brick(id + 1)], at, glam::Vec3::Y));
        }
        if row + 1 < WALL[1] {
            for above in (row + 1) * WALL[0]..(row + 2) * WALL[0] {
                let other = glam::Vec3::from(laid_at(above));
                let overlap = 2.0 * BRICK[0] - (other.x - centre.x).abs();
                if overlap > 0.0 {
                    let x = (centre.x + other.x) / 2.0;
                    let at = glam::Vec3::new(x, centre.y + BRICK[1], 0.0);
                    bonds.push(([End::Brick(id), End::Brick(above)], at, glam::Vec3::X));
                }
            }
        }
    }
    bonds
}

/// Union-find over `n` nodes.
struct Groups(Vec<usize>);

impl Groups {
    fn new(n: usize) -> Self {
        Self((0..n).collect())
    }

    fn root(&self, mut node: usize) -> usize {
        while self.0[node] != node {
            node = self.0[node];
        }
        node
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.root(a), self.root(b));
        self.0[a.max(b)] = a.min(b);
    }
}

impl Scene for Bonds {
    fn update(&mut self, input: &Input) -> Result<()> {
        if let Some(ray) = input.edges.pick {
            if let Some(brick) = self.picked_brick(ray)? {
                self.break_brick(brick)?;
            }
        }
        self.move_prop()?;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        for bond in &mut self.bonds {
            bond.flash = bond.flash.saturating_sub(1);
        }
        self.break_overloaded()?;
        if self.tick + 1 == RELEASE_FROM && self.load_breaks == 0 {
            self.milestones.reach("wall stands");
        }
        if self.is_broken_in_two()? {
            self.milestones.reach("wall broke in two");
        }
        self.tick += 1;
        Ok(())
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        for (index, bond) in self.bonds.iter().enumerate() {
            let colour = if self.is_intact(index)? {
                Self::load_colour(self.load(index)?)
            } else if bond.flash > 0 {
                colours::HIGHLIGHT
            } else {
                continue;
            };
            let Some((p, q)) = self.tracked.pose(self.body(bond.ends[0])) else {
                continue;
            };
            let (p, q) = (position(p), glam_quat(q));
            let half = bond.along * 0.06;
            out.line(
                (p + q * (bond.at - half)).to_array(),
                (p + q * (bond.at + half)).to_array(),
                colour,
            );
        }
        let (intact, broken) = self.counts()?;
        out.hud.push(format!(
            "bonds: {intact} intact, {broken} broken; highest load {:.0} % of the limit",
            self.highest_load * 100.0
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        digest.u32(self.tick);
        digest.u64(self.bonds.len() as u64);
        for bond in &self.bonds {
            let constraint = self.world.constraint(bond.id)?;
            digest.u32(bond.id.to_raw());
            digest.bool(constraint.is_enabled());
            digest.vec3(constraint.total_lambda_position());
            digest.vec3(constraint.total_lambda_rotation());
            digest.u32(bond.flash);
        }
        digest.u32(self.load_breaks);
        digest.u32(self.click_breaks);
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
        CameraHint::new([0.0, 1.4, 0.0], 0.35, 0.2, 6.5)
    }

    /// From the front, wide enough to see the free end fall to the ground.
    fn record_camera(&self, _tick: u32) -> CameraHint {
        CameraHint::new([0.0, 1.3, 0.0], 0.12, 0.12, 5.8)
    }

    fn record_ticks(&self) -> u32 {
        300
    }

    fn script(&self, _tick: u32) -> Input {
        Input::default()
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}

#[cfg(test)]
mod tests;
