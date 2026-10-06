//! A running scene with its tick counter, pending key presses and the wireframe toggle: the one
//! path through which the window, the headless runner and the recorder drive a scene.

use std::sync::atomic::{AtomicU64, Ordering};

use crate::digest::Digest;
use crate::draw::DrawList;
use crate::input::{Held, Input, InputQueue};
use crate::scene::{Result, Scene, SceneConfig, SceneKind};

/// Generations of scene builds in this process; each build's visual keys carry its own.
static NEXT_GENERATION: AtomicU64 = AtomicU64::new(1);

/// How far around the camera target the wireframe reaches, metres.
pub const WIREFRAME_RADIUS: f32 = 12.0;
/// The most wireframe lines drawn per frame.
pub const WIREFRAME_LINES: usize = 60_000;

/// One scene being run.
pub struct Session {
    kind: SceneKind,
    config: SceneConfig,
    scene: Box<dyn Scene>,
    tick: u32,
    wireframe: bool,
    wireframe_drawn: bool,
    queue: InputQueue,
    #[cfg(feature = "debug-renderer")]
    lines: oxijolt::DebugLines,
}

impl Session {
    /// A fresh build of `kind` at tick 0.
    pub fn new(kind: SceneKind, config: SceneConfig) -> Result<Self> {
        let scene = kind.build(&config, NEXT_GENERATION.fetch_add(1, Ordering::Relaxed))?;
        Ok(Self {
            kind,
            config,
            scene,
            tick: 0,
            wireframe: false,
            wireframe_drawn: false,
            queue: InputQueue::default(),
            #[cfg(feature = "debug-renderer")]
            lines: oxijolt::DebugLines::new(),
        })
    }

    /// Rebuilds the scene from scratch: tick 0, nothing pending, the wireframe kept as it was.
    pub fn reset(&mut self) -> Result<()> {
        let wireframe = self.wireframe;
        *self = Self::new(self.kind, self.config)?;
        self.wireframe = wireframe;
        Ok(())
    }

    /// The scene being run.
    pub fn kind(&self) -> SceneKind {
        self.kind
    }

    /// The scene.
    pub fn scene(&self) -> &dyn Scene {
        self.scene.as_ref()
    }

    /// Ticks run since the build.
    pub fn tick_count(&self) -> u32 {
        self.tick
    }

    /// Whether the wireframe is on.
    pub fn wireframe(&self) -> bool {
        self.wireframe
    }

    /// Presses waiting for the next tick.
    pub fn queue(&mut self) -> &mut InputQueue {
        &mut self.queue
    }

    /// Runs one tick with `input`.
    pub fn tick(&mut self, input: Input) -> Result<()> {
        if input.edges.wireframe {
            self.wireframe = !self.wireframe;
        }
        self.scene.update(&input)?;
        self.tick += 1;
        Ok(())
    }

    /// Runs one tick with `held` and the presses waiting in the queue.
    pub fn tick_queued(&mut self, held: Held) -> Result<()> {
        let input = self.queue.next_tick(held);
        self.tick(input)
    }

    /// Runs one tick of the scene's script.
    pub fn tick_scripted(&mut self) -> Result<()> {
        let input = self.scene.script(self.tick);
        self.tick(input)
    }

    /// Folds the run's state into `digest`: every body of the world as the binding reports it,
    /// then the scene's own state.
    pub fn write_state(&mut self, digest: &mut Digest) -> Result<()> {
        digest.world(self.scene.world_mut())?;
        self.scene.write_state(digest)
    }

    /// The milestones of the scene's clip not reached yet, including the wireframe for a scene
    /// that shows it when the debug renderer is built in.
    pub fn missing_milestones(&self) -> Vec<&'static str> {
        let mut missing = self.scene.milestones().missing();
        let wireframe_shown = !cfg!(feature = "debug-renderer") || self.wireframe_drawn;
        if self.scene.shows_wireframe() && !wireframe_shown {
            missing.push("wireframe drawn");
        }
        missing
    }

    /// Fills `out` with the scene's draw list, the wireframe when it is on, and the HUD.
    pub fn draw(&mut self, out: &mut DrawList) -> Result<()> {
        out.clear();
        self.scene.draw(out)?;
        if self.wireframe {
            self.draw_wireframe(out)?;
        }
        Ok(())
    }

    #[cfg(feature = "debug-renderer")]
    fn draw_wireframe(&mut self, out: &mut DrawList) -> Result<()> {
        let centre = crate::math::rvec(self.scene.camera().target);
        let settings =
            oxijolt::DebugLineSettings::new(centre, WIREFRAME_RADIUS).max_lines(WIREFRAME_LINES);
        self.scene
            .world()
            .debug_lines(&settings, &oxijolt::QueryFilter::new(), &mut self.lines)?;
        for line in self.lines.lines() {
            out.line(
                crate::math::position_f32(line.from),
                crate::math::position_f32(line.to),
                crate::draw::colours::WIREFRAME,
            );
        }
        let truncated = if self.lines.is_truncated() {
            ", truncated"
        } else {
            ""
        };
        out.hud.push(format!(
            "wireframe: {} lines{truncated}",
            self.lines.lines().len()
        ));
        self.wireframe_drawn |= !self.lines.lines().is_empty();
        Ok(())
    }

    #[cfg(not(feature = "debug-renderer"))]
    fn draw_wireframe(&mut self, out: &mut DrawList) -> Result<()> {
        out.hud
            .push("wireframe needs the debug-renderer feature".to_owned());
        Ok(())
    }
}
