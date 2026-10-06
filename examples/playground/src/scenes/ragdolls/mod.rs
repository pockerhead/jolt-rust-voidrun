//! Humanoid ragdolls dropped on terrain and stairs, each watched by a settle detector, and a
//! puppet on a pedestal that a skeleton mapper drives from a detailed animation skeleton:
//! kinematically, or with its joint motors while it falls.

mod humanoid;
mod rig;

use oxijolt::{
    Activation, BodySettings, EventSettings, MappedSkeleton, MotionType, PhysicsWorld, RagdollId,
    RagdollSettings, SettleDetector, SkeletonMapper, SkeletonPose, TranslationLocks, Vec3,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList, Solid};
use crate::input::{Edges, Input};
use crate::math::{position_f32, rvec};
use crate::scene::{new_world, step, Milestones, Result, Scene, SceneConfig, DT};
use crate::terrain;
use crate::tracked::Tracked;
use crate::visual::{Shaped, VisualKey, Visuals};
use rig::Rig;

/// The most ragdolls dropped at once; the oldest goes first.
const MAX_DROPPED: usize = 8;
/// Where the puppet's pelvis is when it stands on its pedestal.
const PUPPET: [f32; 3] = [0.0, 1.0 + humanoid::FEET + 0.01, -3.0];
/// How far beside the puppet its animation skeleton is drawn.
const SKELETON_OFFSET: [f32; 3] = [1.6, 0.0, 0.0];

const MILESTONES: &[&str] = &[
    "mapped pose drives the puppet",
    "ragdoll settled",
    "motor-driven fall",
];
const CONTROLS: &[(&str, &str)] = &[
    ("Space", "drop another ragdoll"),
    ("M", "puppet: motors and falling, or back to kinematic"),
];

/// A dropped ragdoll and its settle detector.
struct Dropped {
    id: RagdollId,
    detector: SettleDetector,
    ticks: u32,
    settled: Option<u32>,
}

/// The ragdolls scene.
pub struct Ragdolls {
    world: PhysicsWorld,
    visuals: Visuals,
    tracked: Tracked,
    settings: RagdollSettings,
    part_visuals: Vec<VisualKey>,
    dropped: Vec<Dropped>,
    drops: u32,
    puppet: RagdollId,
    puppet_dynamic: bool,
    mapper: SkeletonMapper,
    rig: Rig,
    /// The animation skeleton of the last tick, mapped from the puppet's pose.
    shown: Option<SkeletonPose>,
    tick: u32,
    kinematic_ticks: u32,
    motor_ticks: u32,
    milestones: Milestones,
}

impl Ragdolls {
    /// Builds the terrain, the stairs, the pedestal, four falling ragdolls and the puppet.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let hills = terrain::rolling(0.3);
        let terrain = terrain::height_field(33, 1.0, |x, z| {
            hills(x, z) * ((x.abs() + z.abs() - 6.0) / 6.0).clamp(0.0, 1.0)
        })?;
        tracked.spawn(&mut world, &terrain, &ground, &mut visuals, colours::GROUND)?;
        for step in 0..4 {
            let top = 0.3 * (step + 1) as f32;
            let at = ground
                .clone()
                .position(rvec([3.0 + 0.6 * step as f32, top / 2.0, 2.0]));
            let shaped = Shaped::cuboid([0.3, top / 2.0, 1.5])?;
            tracked.spawn(&mut world, &shaped, &at, &mut visuals, colours::STRUCTURE)?;
        }
        let pedestal = ground.clone().position(rvec([PUPPET[0], 0.5, PUPPET[2]]));
        tracked.spawn(
            &mut world,
            &Shaped::cuboid([0.6, 0.5, 0.6])?,
            &pedestal,
            &mut visuals,
            colours::STRUCTURE,
        )?;

        let (settings, part_shapes) = humanoid::settings(layers.moving)?;
        let part_visuals = part_shapes
            .into_iter()
            .map(|visual| visuals.add(visual))
            .collect();
        let puppet = world.create_ragdoll(
            &settings,
            Some(&humanoid::bind_pose(PUPPET)),
            Activation::Activate,
        )?;
        world
            .ragdoll_mut(puppet)?
            .set_motion_type(MotionType::Kinematic, Activation::Activate)?;
        let rig = Rig::humanoid();
        let ragdoll_skeleton = humanoid::skeleton()?;
        let rig_skeleton = rig.skeleton()?;
        let mapper = SkeletonMapper::new(
            MappedSkeleton {
                skeleton: &ragdoll_skeleton,
                neutral_pose: &humanoid::bind_pose([0.0; 3]),
            },
            MappedSkeleton {
                skeleton: &rig_skeleton,
                neutral_pose: &rig.neutral_pose(),
            },
            TranslationLocks::None,
        )?;
        let mut scene = Self {
            world,
            visuals,
            tracked,
            settings,
            part_visuals,
            dropped: Vec::new(),
            drops: 0,
            puppet,
            puppet_dynamic: false,
            mapper,
            rig,
            shown: None,
            tick: 0,
            kinematic_ticks: 0,
            motor_ticks: 0,
            milestones: Milestones::new(MILESTONES),
        };
        // The puppet starts in the wave's first pose, from which every kinematic drive is a
        // small step.
        let start = scene.puppet_target(0)?;
        scene.world.ragdoll_mut(puppet)?.set_pose(&start)?;
        for _ in 0..4 {
            scene.drop_ragdoll()?;
        }
        Ok(scene)
    }

    /// Drops a ragdoll from 1.5 to 3 m with a push, cycling through four places and turns.
    fn drop_ragdoll(&mut self) -> Result<()> {
        if self.dropped.len() >= MAX_DROPPED {
            let oldest = self.dropped.remove(0);
            self.world.remove_ragdoll(oldest.id)?;
        }
        let n = self.drops;
        self.drops += 1;
        let (x, z) = [(-3.0, 2.0), (3.6, 2.0), (-1.0, 4.0), (1.5, -0.5)][n as usize % 4];
        let height = 1.5 + 0.5 * (n % 4) as f32;
        let mut pose = humanoid::bind_pose([x, humanoid::FEET + height, z]);
        // Lying on its side or its back, turned a little each time.
        let tilt = crate::math::about_axis([0.3, 0.2 * n as f32, 1.0], 1.3 + 0.4 * n as f32);
        let tilt = crate::math::glam_quat(tilt);
        for joint in &mut pose.joints {
            joint.translation = crate::math::vec3(tilt * crate::math::glam(joint.translation));
            joint.rotation = crate::math::quat(tilt * crate::math::glam_quat(joint.rotation));
        }
        let id = self
            .world
            .create_ragdoll(&self.settings, Some(&pose), Activation::Activate)?;
        let push = Vec3::new(0.6 - 0.4 * (n % 3) as f32, 0.0, 0.5);
        self.world
            .ragdoll_mut(id)?
            .set_linear_and_angular_velocity(push, Vec3::new(0.0, 1.0, 0.0))?;
        self.dropped.push(Dropped {
            id,
            detector: SettleDetector::default(),
            ticks: 0,
            settled: None,
        });
        Ok(())
    }

    /// The wave at tick `tick` on the pedestal, mapped onto the ragdoll.
    fn puppet_target(&self, tick: u32) -> Result<SkeletonPose> {
        let local = self.rig.animated_local(tick as f32 * DT);
        let animation = self.rig.to_model(&local, rvec(PUPPET));
        Ok(self.mapper.map_reverse(&animation)?)
    }

    /// Drives the puppet toward the wave of tick `self.tick`.
    fn drive_puppet(&mut self) -> Result<()> {
        let target = self.puppet_target(self.tick)?;
        let mut puppet = self.world.ragdoll_mut(self.puppet)?;
        if self.puppet_dynamic {
            puppet.drive_to_pose_using_motors(&target)?;
            self.motor_ticks += 1;
        } else {
            puppet.drive_to_pose_using_kinematics(&target, DT)?;
            self.kinematic_ticks += 1;
        }
        Ok(())
    }

    /// Switches the puppet between kinematic and dynamic with motors.
    fn toggle_puppet(&mut self) -> Result<()> {
        let target = self.puppet_target(self.tick)?;
        let mut puppet = self.world.ragdoll_mut(self.puppet)?;
        if self.puppet_dynamic {
            // Back on the pedestal first: one kinematic drive from wherever it fell could ask
            // for more than the velocity limits.
            puppet.set_pose(&target)?;
            puppet.set_motion_type(MotionType::Kinematic, Activation::Activate)?;
        } else {
            puppet.set_motion_type(MotionType::Dynamic, Activation::Activate)?;
            // A nudge forward, so it topples off the pedestal toward the camera.
            puppet.set_linear_and_angular_velocity(Vec3::new(0.0, 0.0, 0.8), Vec3::ZERO)?;
        }
        self.puppet_dynamic = !self.puppet_dynamic;
        Ok(())
    }

    fn check_milestones(&mut self) -> Result<()> {
        if self.kinematic_ticks >= 30 {
            self.milestones.reach("mapped pose drives the puppet");
        }
        for dropped in &mut self.dropped {
            dropped.ticks += 1;
            let ragdoll = self.world.ragdoll(dropped.id)?;
            if dropped.settled.is_none() && dropped.detector.update(&ragdoll) {
                dropped.settled = Some(dropped.ticks);
                self.milestones.reach("ragdoll settled");
            }
        }
        if self.puppet_dynamic && self.motor_ticks >= 30 {
            let pelvis = self.world.ragdoll(self.puppet)?.pose().root_offset;
            if position_f32(pelvis)[1] < PUPPET[1] - 0.4 {
                self.milestones.reach("motor-driven fall");
            }
        }
        Ok(())
    }
}

impl Scene for Ragdolls {
    fn update(&mut self, input: &Input) -> Result<()> {
        self.tick += 1;
        if input.edges.jump {
            self.drop_ragdoll()?;
        }
        if input.edges.motor {
            self.toggle_puppet()?;
        }
        self.drive_puppet()?;
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        let pose = self.world.ragdoll(self.puppet)?.pose();
        let local = self.rig.animated_local(self.tick as f32 * DT);
        self.shown = Some(self.mapper.map(&pose, &local)?);
        self.check_milestones()
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        self.tracked.draw(out);
        let ragdolls = self
            .dropped
            .iter()
            .map(|dropped| {
                (
                    dropped.id,
                    dropped.settled.map_or(colours::BODY, |_| colours::ASLEEP),
                )
            })
            .chain([(self.puppet, colours::PLAYER)]);
        for (id, colour) in ragdolls {
            let ragdoll = self.world.ragdoll(id)?;
            for (index, &body) in ragdoll.body_ids().iter().enumerate() {
                let reading = self.world.body(body)?;
                out.solids.push(Solid {
                    visual: self.part_visuals[index],
                    position: position_f32(reading.position()),
                    rotation: reading.rotation().into(),
                    colour,
                });
            }
        }
        if let Some(shown) = &self.shown {
            let offset = glam::Vec3::from(SKELETON_OFFSET);
            let at = |index: usize| {
                (glam::Vec3::from(position_f32(shown.root_offset))
                    + crate::math::glam(shown.joints[index].translation)
                    + offset)
                    .to_array()
            };
            for (index, parent) in self.rig.parents().enumerate() {
                if let Some(parent) = parent {
                    out.line(at(parent), at(index), colours::QUERY);
                }
            }
        }
        let settled: Vec<String> = self
            .dropped
            .iter()
            .map(|dropped| {
                dropped.settled.map_or("falling".to_owned(), |ticks| {
                    format!("settled after {ticks} ticks")
                })
            })
            .collect();
        out.hud.push(format!("ragdolls: {}", settled.join(", ")));
        let puppet = if self.puppet_dynamic {
            "dynamic, motors on"
        } else {
            "kinematic"
        };
        out.hud.push(format!(
            "puppet: {puppet}; yellow lines: the animation skeleton mapped from it"
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) {
        self.tracked.write_state(digest);
        for id in self.world.ragdoll_ids() {
            digest.u32(id.to_raw());
            if let Ok(ragdoll) = self.world.ragdoll(id) {
                let pose = ragdoll.pose();
                digest.rvec3(pose.root_offset);
                for joint in &pose.joints {
                    digest.vec3(joint.translation);
                    digest.quat(joint.rotation);
                }
            }
        }
        for dropped in &self.dropped {
            digest.u32(dropped.settled.unwrap_or(u32::MAX));
        }
        digest.bool(self.puppet_dynamic);
        digest.u32(self.tick);
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        CameraHint::new([0.5, 0.8, 0.0], 0.45, 0.4, 9.0)
    }

    fn record_ticks(&self) -> u32 {
        420
    }

    fn script(&self, tick: u32) -> Input {
        Input {
            edges: Edges {
                motor: tick == 180,
                jump: tick == 260,
                ..Edges::default()
            },
            ..Input::default()
        }
    }

    fn milestones(&self) -> &Milestones {
        &self.milestones
    }

    fn controls(&self) -> &'static [(&'static str, &'static str)] {
        CONTROLS
    }
}
