//! Scene queries under the cursor, body controls, saving and restoring, and a floating origin,
//! in a small town on terrain: every tick the cursor ray, a sphere cast along it, a sphere
//! overlap and a point test at the hit; a click pushes a crate, T makes it kinematic or dynamic,
//! K and L save and restore the world, B moves the origin to the cursor.

use oxijolt::{
    Activation, BodyId, BodySettings, CollideShape, EventSettings, MotionType, PhysicsWorld, Quat,
    QueryFilter, RayCast, ShapeCast, StateError, WorldState,
};

use crate::camera::CameraHint;
use crate::digest::Digest;
use crate::draw::{colours, DrawList, Solid};
use crate::input::{Edges, Input};
use crate::math::{about_axis, glam, position, position_f32, rvec, vec3};
use crate::scene::{new_world, step, Milestones, Result, Scene, SceneConfig};
use crate::terrain;
use crate::tracked::Tracked;
use crate::visual::{Shaped, Visual, VisualKey, Visuals};

/// Radius of the cast and overlap spheres, metres.
const PROBE_RADIUS: f32 = 0.3;
/// The tick at which the script shows the collider wireframe.
const WIREFRAME_TICK: u32 = 250;

const MILESTONES: &[&str] = &[
    "ray hit",
    "shape cast stopped",
    "overlap found",
    "restored",
    "rebased",
];
const CONTROLS: &[(&str, &str)] = &[
    (
        "mouse",
        "ray, sphere cast, overlap and point test under the cursor",
    ),
    ("left click", "push the body under the cursor"),
    ("T", "the last clicked crate: kinematic or dynamic"),
    ("K, L", "save the world, restore it"),
    ("B", "move the origin to the point under the cursor"),
];

/// What the queries under the cursor found this tick.
#[derive(Debug, Default)]
struct Found {
    /// The body the ray hit, the hit point and the surface normal.
    hit: Option<(BodyId, [f32; 3], [f32; 3])>,
    /// Where the sphere cast along the ray stopped.
    cast_stop: Option<[f32; 3]>,
    /// The bodies the sphere at the hit overlaps, with the depth.
    overlaps: Vec<(BodyId, f32)>,
    /// The bodies whose shape contains the hit point pushed 5 cm in.
    containing: Vec<BodyId>,
}

/// The queries scene.
pub struct Queries {
    world: PhysicsWorld,
    visuals: Visuals,
    tracked: Tracked,
    terrain: BodyId,
    probe: Shaped,
    probe_visual: VisualKey,
    /// A post drawn at the world's origin, which a rebase moves.
    origin_post: VisualKey,
    crates: Vec<BodyId>,
    picked: Option<BodyId>,
    saved: Option<WorldState>,
    origin: [f32; 3],
    found: Found,
    message: String,
    milestones: Milestones,
}

impl Queries {
    /// Builds the terrain, four buildings and the crates.
    pub fn new(config: &SceneConfig, generation: u64) -> Result<Self> {
        let (mut world, layers) = new_world(config, 2, EventSettings::default())?;
        let mut visuals = Visuals::new(generation);
        let mut tracked = Tracked::default();
        let ground = BodySettings::new_static().object_layer(layers.ground);
        let hills = terrain::rolling(0.25);
        let hills_shape = terrain::height_field(33, 1.0, |x, z| hills(x, z) - 0.3)?;
        let terrain = tracked.spawn(
            &mut world,
            &hills_shape,
            &ground,
            &mut visuals,
            colours::GROUND,
        )?;

        // Buildings: compounds of a block and a roof turned a quarter about its ridge.
        let roof = Shaped::cuboid([1.0, 0.45, 0.45])?;
        let turned = about_axis([1.0, 0.0, 0.0], std::f32::consts::FRAC_PI_4);
        for (index, [x, z, height]) in [
            [-5.0, -3.0, 1.2],
            [-1.0, -4.0, 1.8],
            [3.5, -3.0, 1.0],
            [6.0, 1.5, 1.5],
        ]
        .into_iter()
        .enumerate()
        {
            let block = Shaped::cuboid([1.0, height, 1.0])?;
            let house = Shaped::compound(&[
                (&block, [0.0, height, 0.0], Quat::IDENTITY, 1),
                (&roof, [0.0, 2.0 * height, 0.0], turned, 2),
            ])?;
            let at = ground
                .clone()
                .position(rvec([x, -0.2, z]))
                .user_data(index as u64);
            tracked.spawn(&mut world, &house, &at, &mut visuals, colours::STRUCTURE)?;
        }

        let crate_shape = Shaped::cuboid([0.3; 3])?;
        let crate_visual = visuals.add(crate_shape.visual.clone());
        let mut crates = Vec::new();
        for index in 0..8 {
            let x = -4.0 + 1.1 * index as f32;
            let z = 1.0 + 0.6 * (index % 3) as f32;
            let settings = BodySettings::new_dynamic()
                .position(rvec([x, hills(x, z) + 0.6, z]))
                .object_layer(layers.moving)
                .allow_dynamic_or_kinematic(true)
                .mass(10.0);
            let body = tracked.spawn_keyed(
                &mut world,
                &crate_shape.shape,
                crate_visual,
                &settings,
                colours::BODY,
            )?;
            crates.push(body);
        }
        let probe = Shaped::sphere(PROBE_RADIUS)?;
        let probe_visual = visuals.add(probe.visual.clone());
        let origin_post = visuals.add(Visual::Box {
            half_extent: [0.06, 1.5, 0.06],
        });
        Ok(Self {
            world,
            visuals,
            tracked,
            terrain,
            probe,
            probe_visual,
            origin_post,
            crates,
            picked: None,
            saved: None,
            origin: [0.0; 3],
            found: Found::default(),
            message: String::new(),
            milestones: Milestones::new(MILESTONES),
        })
    }

    /// Runs the four queries along `aim`.
    fn query(&mut self, aim: RayCast) -> Result<()> {
        let filter = QueryFilter::new();
        let mut found = Found::default();
        if let Some(hit) = self.world.cast_ray(aim, &filter)? {
            let point = aim.point_at(hit.fraction);
            found.hit = Some((hit.body, position_f32(point), <[f32; 3]>::from(hit.normal)));
            self.milestones.reach("ray hit");

            let sphere = CollideShape::new(&self.probe.shape, point, Quat::IDENTITY);
            let mut overlaps: Vec<(BodyId, f32)> = self
                .world
                .collide_shape(&sphere, &filter)?
                .into_iter()
                .map(|overlap| (overlap.body, overlap.penetration_depth))
                .collect();
            // Jolt reports overlaps in no fixed order.
            overlaps.sort_by_key(|&(body, _)| body);
            if !overlaps.is_empty() {
                self.milestones.reach("overlap found");
            }
            found.overlaps = overlaps;

            let inside = position(point) - glam(hit.normal) * 0.05;
            found.containing = self
                .world
                .collide_point(rvec(inside.to_array()), &filter)?
                .into_iter()
                .map(|hit| hit.body)
                .collect();
        }
        let cast = ShapeCast::new(&self.probe.shape, aim.origin, Quat::IDENTITY, aim.direction);
        if let Some(stop) = self.world.cast_shape(&cast, &filter)? {
            let travelled = glam(aim.direction) * stop.fraction;
            found.cast_stop = Some((position(aim.origin) + travelled).to_array());
            self.milestones.reach("shape cast stopped");
        }
        self.found = found;
        Ok(())
    }

    /// Pushes the body under a click and remembers a crate as the one T toggles.
    fn push(&mut self, ray: RayCast) -> Result<()> {
        let Some(hit) = self.world.cast_ray(ray, &QueryFilter::new())? else {
            return Ok(());
        };
        if !self.crates.contains(&hit.body) {
            return Ok(());
        }
        self.picked = Some(hit.body);
        let mut body = self.world.body_mut(hit.body)?;
        if body.motion_type() == MotionType::Dynamic {
            let push = glam(ray.direction).normalize() * 60.0;
            body.add_impulse_at_point(vec3(push), ray.point_at(hit.fraction))?;
        }
        Ok(())
    }

    /// Toggles the last clicked crate between dynamic and kinematic.
    fn toggle_picked(&mut self) -> Result<()> {
        let Some(crate_body) = self.picked else {
            return Ok(());
        };
        let mut body = self.world.body_mut(crate_body)?;
        let (motion, colour) = if body.motion_type() == MotionType::Dynamic {
            (MotionType::Kinematic, colours::KINEMATIC)
        } else {
            (MotionType::Dynamic, colours::BODY)
        };
        body.set_motion_type(motion, Activation::Activate)?;
        self.tracked.set_colour(crate_body, colour);
        Ok(())
    }

    /// Moves the origin to the point the cursor ray `aim` hits now, or under the camera target
    /// without a cursor or a hit; returns the translation.
    fn rebase(&mut self, aim: Option<RayCast>) -> Result<[f32; 3]> {
        let hit = match aim {
            Some(ray) => self
                .world
                .cast_ray(ray, &QueryFilter::new())?
                .map(|hit| position_f32(ray.point_at(hit.fraction))),
            None => None,
        };
        let focus = hit.unwrap_or(self.camera().target);
        let shift = [-focus[0], 0.0, -focus[2]];
        let bodies = self.tracked.ids_in_order();
        self.world.rebase(&bodies, Quat::IDENTITY, rvec(shift))?;
        self.tracked.sync_all(&self.world);
        for (origin, delta) in self.origin.iter_mut().zip(shift) {
            *origin += delta;
        }
        self.message = format!("origin moved by ({:.1}, 0, {:.1})", shift[0], shift[2]);
        self.milestones.reach("rebased");
        Ok(shift)
    }

    fn restore(&mut self) -> Result<()> {
        let Some(saved) = &self.saved else {
            self.message = "nothing saved yet (K)".to_owned();
            return Ok(());
        };
        match self.world.restore_state(saved) {
            Ok(()) => {
                self.tracked.sync_all(&self.world);
                self.message = "restored the saved world".to_owned();
                self.milestones.reach("restored");
            }
            Err(StateError::WorldChanged) => {
                self.message =
                    "restore refused: the world changed since the save (WorldChanged)".to_owned();
            }
            Err(error) => return Err(error.into()),
        }
        Ok(())
    }
}

impl Scene for Queries {
    fn update(&mut self, input: &Input) -> Result<()> {
        if let Some(ray) = input.edges.pick {
            self.push(ray)?;
        }
        if input.edges.toggle {
            self.toggle_picked()?;
        }
        if input.edges.save {
            self.saved = Some(self.world.save_state());
            self.message = "saved the world".to_owned();
        }
        if input.edges.restore {
            self.restore()?;
        }
        let mut aim = input.held.aim;
        if input.edges.rebase {
            let shift = glam::Vec3::from(self.rebase(aim)?);
            // The cursor ray was taken before the move; it moves along, like the camera.
            aim = aim.map(|ray| {
                RayCast::new(
                    rvec((position(ray.origin) + shift).to_array()),
                    ray.direction,
                )
            });
        }
        step(&mut self.world)?;
        let events = self.world.take_events();
        self.tracked.sync(&self.world, &events);
        if let Some(aim) = aim {
            self.query(aim)?;
        }
        Ok(())
    }

    fn draw(&self, out: &mut DrawList) -> Result<()> {
        let hit_body = self
            .found
            .hit
            .map(|(body, _, _)| body)
            .filter(|&body| body != self.terrain);
        self.tracked.draw_with(out, |body, colour| {
            if Some(body) == hit_body {
                colours::HIGHLIGHT
            } else {
                colour
            }
        });
        // The world's origin: a post with its x axis in red and its z axis in blue.
        out.solids.push(Solid {
            visual: self.origin_post,
            position: [0.0, 1.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
            colour: colours::HIGHLIGHT,
        });
        out.line([0.0, 0.0, 0.0], [1.5, 0.0, 0.0], colours::HIGHLIGHT);
        out.line([0.0, 0.0, 0.0], [0.0, 0.0, 1.5], colours::BODY_ALT);
        if let Some((_, point, normal)) = self.found.hit {
            out.cross(point, 0.3, colours::QUERY);
            let tip = (glam::Vec3::from(point) + glam::Vec3::from(normal)).to_array();
            out.line(point, tip, colours::QUERY);
        }
        if let Some(stop) = self.found.cast_stop {
            out.solids.push(Solid {
                visual: self.probe_visual,
                position: stop,
                rotation: [0.0, 0.0, 0.0, 1.0],
                colour: colours::QUERY,
            });
        }
        let depths: Vec<String> = self
            .found
            .overlaps
            .iter()
            .map(|(body, depth)| format!("{} by {depth:.2} m", body.to_raw()))
            .collect();
        out.hud
            .push(format!("sphere at the hit overlaps: {}", depths.join(", ")));
        let containing: Vec<String> = self
            .found
            .containing
            .iter()
            .map(|body| body.to_raw().to_string())
            .collect();
        out.hud.push(format!(
            "bodies containing the hit point: {}",
            containing.join(", ")
        ));
        out.hud.push(format!(
            "origin offset ({:.1}, {:.1}, {:.1}); {}",
            self.origin[0], self.origin[1], self.origin[2], self.message
        ));
        Ok(())
    }

    fn write_state(&self, digest: &mut Digest) -> Result<()> {
        self.tracked.write_state(digest);
        digest.f32s(&self.origin);
        if let Some((body, point, normal)) = self.found.hit {
            digest.u32(body.to_raw());
            digest.f32s(&point);
            digest.f32s(&normal);
        }
        if let Some(stop) = self.found.cast_stop {
            digest.f32s(&stop);
        }
        for (body, depth) in &self.found.overlaps {
            digest.u32(body.to_raw());
            digest.f32(*depth);
        }
        for body in &self.found.containing {
            digest.u32(body.to_raw());
        }
        digest.bool(self.saved.is_some());
        Ok(())
    }

    fn visuals(&self) -> &Visuals {
        &self.visuals
    }

    fn world(&self) -> &PhysicsWorld {
        &self.world
    }

    fn camera(&self) -> CameraHint {
        let [x, y, z] = self.origin;
        CameraHint::new([x, y + 1.0, z], 0.3, 0.5, 13.0)
    }

    /// Closer than the camera the cursor's ray comes from, following the origin like it; the
    /// last second shows the collider wireframe from further out.
    fn record_camera(&self, tick: u32) -> CameraHint {
        let [x, y, z] = self.origin;
        if tick < WIREFRAME_TICK {
            CameraHint::new([x, y + 0.6, z + 1.0], 0.3, 0.45, 8.5)
        } else {
            self.camera()
        }
    }

    fn script(&self, tick: u32) -> Input {
        // The cursor sweeps across the town and back, from the default camera.
        let sweep = (tick as f32 / 300.0 * std::f32::consts::TAU).sin();
        let aim = self.camera().ray([0.6 * sweep, -0.25], 16.0 / 9.0);
        let crate_ray = |world: &PhysicsWorld, crate_body: BodyId| {
            world.body(crate_body).ok().map(|body| {
                let eye = self.camera().eye();
                let target = position(body.position());
                RayCast::new(rvec(eye.to_array()), vec3((target - eye) * 1.5))
            })
        };
        let mut input = Input::default();
        input.held.aim = Some(aim);
        input.edges = Edges {
            wireframe: tick == WIREFRAME_TICK,
            pick: (tick == 40)
                .then(|| crate_ray(&self.world, self.crates[3]))
                .flatten(),
            save: tick == 60,
            restore: matches!(tick, 90 | 200),
            toggle: tick == 110,
            // With the cursor far to the left, so the origin moves visibly.
            rebase: tick == 230,
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

    fn shows_wireframe(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::Held;

    fn scene() -> Queries {
        Queries::new(
            &SceneConfig {
                worker_threads: Some(1),
            },
            1,
        )
        .unwrap()
    }

    /// Where `ray` hits a static body of `scene` now.
    fn static_hit(scene: &Queries, ray: RayCast) -> [f32; 3] {
        let hit = scene
            .world
            .cast_ray(ray, &QueryFilter::new())
            .unwrap()
            .expect("the ray hits");
        let body = scene.world.body(hit.body).unwrap();
        assert_eq!(body.motion_type(), MotionType::Static);
        position_f32(ray.point_at(hit.fraction))
    }

    fn input(aim: RayCast, rebase: bool) -> Input {
        Input {
            held: Held {
                aim: Some(aim),
                ..Held::default()
            },
            edges: Edges {
                rebase,
                ..Edges::default()
            },
        }
    }

    #[test]
    fn b_on_the_first_tick_moves_the_origin_to_the_point_under_the_cursor() {
        let mut scene = scene();
        let ray = scene.camera().ray([0.4, -0.45], 16.0 / 9.0);
        let point = static_hit(&scene, ray);
        assert!(
            point[0].abs() > 1.0,
            "away from the camera target: {point:?}"
        );
        scene.update(&input(ray, true)).unwrap();
        assert_eq!(scene.origin, [-point[0], 0.0, -point[2]]);
        // The tick's queries ran along the moved ray: the hit is the same point, moved.
        let (_, moved, _) = scene.found.hit.unwrap();
        let expected = [0.0, point[1], 0.0];
        for (a, b) in moved.iter().zip(expected) {
            assert!((a - b).abs() < 1e-3, "{moved:?} against {expected:?}");
        }
    }

    #[test]
    fn b_rebases_to_the_cursor_of_its_own_tick() {
        let mut scene = scene();
        let before = scene.camera().ray([-0.4, -0.45], 16.0 / 9.0);
        let now = scene.camera().ray([0.4, -0.45], 16.0 / 9.0);
        scene.update(&input(before, false)).unwrap();
        let (old, new) = (static_hit(&scene, before), static_hit(&scene, now));
        assert!((old[0] - new[0]).abs() > 1.0, "{old:?} and {new:?}");
        scene.update(&input(now, true)).unwrap();
        assert_eq!(scene.origin, [-new[0], 0.0, -new[2]]);
    }
}
