//! What a frame draws: posed shapes, lines, deforming surfaces and text, all from data the
//! binding reports.

use crate::visual::VisualKey;

/// A colour, red, green and blue in `[0, 1]`.
pub type Colour = [f32; 3];

/// Colours the scenes share.
pub mod colours {
    use super::Colour;

    /// Static ground.
    pub const GROUND: Colour = [0.45, 0.5, 0.42];
    /// Static structures: walls, stairs, ramps.
    pub const STRUCTURE: Colour = [0.62, 0.6, 0.56];
    /// Dynamic bodies at rest colour.
    pub const BODY: Colour = [0.85, 0.55, 0.25];
    /// A second body colour.
    pub const BODY_ALT: Colour = [0.3, 0.55, 0.85];
    /// A third body colour.
    pub const BODY_THIRD: Colour = [0.55, 0.75, 0.35];
    /// Kinematic bodies.
    pub const KINEMATIC: Colour = [0.75, 0.35, 0.75];
    /// The controlled character or vehicle.
    pub const PLAYER: Colour = [0.95, 0.85, 0.3];
    /// A highlight: a hit, an impact, a sensor.
    pub const HIGHLIGHT: Colour = [1.0, 0.25, 0.2];
    /// Sleeping bodies.
    pub const ASLEEP: Colour = [0.45, 0.45, 0.6];
    /// Water.
    pub const WATER: Colour = [0.2, 0.45, 0.8];
    /// Debug wireframe lines.
    pub const WIREFRAME: Colour = [0.2, 1.0, 0.4];
    /// Lines of query results.
    pub const QUERY: Colour = [1.0, 1.0, 0.3];
    /// Cloth and soft bodies.
    pub const SOFT: Colour = [0.85, 0.35, 0.45];
}

/// A shape description drawn at a pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solid {
    /// The scene's description of the shape.
    pub visual: VisualKey,
    /// Position of the shape's origin, metres.
    pub position: [f32; 3],
    /// Rotation, `[x, y, z, w]`.
    pub rotation: [f32; 4],
    /// Base colour before shading.
    pub colour: Colour,
}

/// A line segment in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Line {
    /// Start point.
    pub from: [f32; 3],
    /// End point.
    pub to: [f32; 3],
    /// Colour.
    pub colour: Colour,
}

/// A triangle surface whose vertices change every frame: soft bodies, water.
#[derive(Clone, Debug, PartialEq)]
pub struct Surface {
    /// Vertex positions in world space.
    pub vertices: Vec<[f32; 3]>,
    /// Triangles as indices into `vertices`, counter-clockwise seen from the front.
    pub triangles: Vec<[u32; 3]>,
    /// Base colour before shading.
    pub colour: Colour,
    /// Drawn after everything else, half transparent.
    pub translucent: bool,
}

/// Everything one frame draws.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DrawList {
    /// Posed shapes.
    pub solids: Vec<Solid>,
    /// Lines.
    pub lines: Vec<Line>,
    /// Deforming surfaces.
    pub surfaces: Vec<Surface>,
    /// Text lines for the heads-up display.
    pub hud: Vec<String>,
}

impl DrawList {
    /// Empties every list, keeping the allocations.
    pub fn clear(&mut self) {
        self.solids.clear();
        self.lines.clear();
        self.surfaces.clear();
        self.hud.clear();
    }

    /// Adds a line from `from` to `to`.
    pub fn line(&mut self, from: [f32; 3], to: [f32; 3], colour: Colour) {
        self.lines.push(Line { from, to, colour });
    }

    /// Adds a small cross of three lines at `at`, `size` metres across.
    pub fn cross(&mut self, at: [f32; 3], size: f32, colour: Colour) {
        let h = size / 2.0;
        for axis in 0..3 {
            let (mut from, mut to) = (at, at);
            from[axis] -= h;
            to[axis] += h;
            self.line(from, to, colour);
        }
    }

    /// Whether every position in the list is finite.
    pub fn is_finite(&self) -> bool {
        let finite = |p: &[f32]| p.iter().all(|value| value.is_finite());
        self.solids
            .iter()
            .all(|solid| finite(&solid.position) && finite(&solid.rotation))
            && self
                .lines
                .iter()
                .all(|line| finite(&line.from) && finite(&line.to))
            && self
                .surfaces
                .iter()
                .all(|surface| surface.vertices.iter().all(|v| finite(v)))
    }
}
