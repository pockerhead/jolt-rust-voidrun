//! Drawing a frame's draw list with macroquad: posed meshes transformed and shaded on the CPU
//! and sent in draw calls within macroquad's capacity, then lines, then translucent surfaces.

use macroquad::prelude::{
    clear_background, draw_line_3d, draw_mesh, set_camera, set_default_camera, Camera3D, Color,
    Mesh, RenderTarget, Vec2, Vec3, Vec4, Vertex,
};

use playground::camera::CameraHint;
use playground::draw::{Colour, DrawList};
use playground::mesh::{MeshStore, ShadedVertex, TriangleBatch, TRIANGLES_PER_DRAW_CALL};
use playground::visual::Visuals;

/// What macroquad is configured to take per draw call, vertices and indices alike.
pub const DRAW_CALL_CAPACITY: usize = 60_000;

/// The sky behind every scene.
pub const BACKGROUND: Color = Color::new(0.6, 0.8, 1.0, 1.0);

/// Alpha of translucent surfaces.
const TRANSLUCENT_ALPHA: u8 = 140;

/// Where a frame goes and how it is seen.
pub struct View {
    /// The camera.
    pub camera: CameraHint,
    /// A render target instead of the screen.
    pub target: Option<RenderTarget>,
    /// The aspect ratio; the screen's or the target's when `None`.
    pub aspect: Option<f32>,
    /// Whether to clear to [`BACKGROUND`] first.
    pub clear: bool,
}

/// Turns draw lists into macroquad draw calls, keeping the meshes of the scene's descriptions
/// and the buffers between frames.
#[derive(Default)]
pub struct Renderer {
    meshes: MeshStore,
    opaque: TriangleBatch,
    translucent: TriangleBatch,
    calls: Vec<Mesh>,
    indices: Vec<u16>,
}

impl Renderer {
    /// Draws `list`'s solids, surfaces and lines as seen by `view`.
    pub fn draw_world(&mut self, list: &DrawList, visuals: &Visuals, view: &View) {
        let camera = &view.camera;
        set_camera(&Camera3D {
            position: to_mq(camera.eye().to_array()),
            target: to_mq(camera.target),
            up: Vec3::Y,
            fovy: camera.fov_y,
            aspect: view.aspect,
            render_target: view.target.clone(),
            z_near: 0.1,
            z_far: 1000.0,
            ..Camera3D::default()
        });
        if view.clear {
            clear_background(BACKGROUND);
        }

        self.opaque.clear();
        self.translucent.clear();
        for solid in &list.solids {
            let mesh = self.meshes.get(solid.visual, visuals);
            self.opaque.push_mesh(
                mesh,
                glam::Vec3::from(solid.position),
                glam::Quat::from_array(solid.rotation),
                solid.colour,
                255,
            );
        }
        for surface in &list.surfaces {
            let (batch, alpha) = if surface.translucent {
                (&mut self.translucent, TRANSLUCENT_ALPHA)
            } else {
                (&mut self.opaque, 255)
            };
            batch.push_surface(&surface.vertices, &surface.triangles, surface.colour, alpha);
        }

        let Self {
            opaque,
            translucent,
            calls,
            indices,
            ..
        } = self;
        submit(opaque, calls, indices);
        for line in &list.lines {
            draw_line_3d(to_mq(line.from), to_mq(line.to), colour(line.colour, 255));
        }
        submit(translucent, calls, indices);
        set_default_camera();
    }
}

/// Sends `batch` as one `draw_mesh` per draw call, reusing the meshes in `calls`.
fn submit(batch: &TriangleBatch, calls: &mut Vec<Mesh>, indices: &mut Vec<u16>) {
    if indices.is_empty() {
        indices.extend((0..3 * TRIANGLES_PER_DRAW_CALL).map(|i| i as u16));
    }
    for (index, vertices) in batch.calls().enumerate() {
        debug_assert!(vertices.len() < DRAW_CALL_CAPACITY);
        if calls.len() <= index {
            calls.push(Mesh {
                vertices: Vec::new(),
                indices: Vec::new(),
                texture: None,
            });
        }
        let mesh = &mut calls[index];
        mesh.vertices.clear();
        mesh.vertices.extend(vertices.iter().map(vertex));
        mesh.indices.clear();
        mesh.indices.extend_from_slice(&indices[..vertices.len()]);
        draw_mesh(mesh);
    }
}

fn vertex(shaded: &ShadedVertex) -> Vertex {
    Vertex {
        position: to_mq(shaded.position),
        uv: Vec2::ZERO,
        color: shaded.colour,
        normal: Vec4::ZERO,
    }
}

/// A point in macroquad's glam.
pub fn to_mq([x, y, z]: [f32; 3]) -> Vec3 {
    Vec3::new(x, y, z)
}

/// A playground colour with `alpha` as a macroquad colour.
pub fn colour([r, g, b]: Colour, alpha: u8) -> Color {
    Color::new(r, g, b, f32::from(alpha) / 255.0)
}
