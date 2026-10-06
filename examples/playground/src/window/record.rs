//! Recording a scene's clip: its script at the fixed step, every second tick rendered from the
//! scene's fixed record camera into a render target, read back, reduced and written as a GIF,
//! and a PNG still from the middle of the clip.

use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

use macroquad::prelude::{
    clear_background, draw_text, draw_texture_ex, next_frame, render_target_ex, screen_width,
    Color, DrawTextureParams, Image, RenderTarget, RenderTargetParams, Vec2,
};

use playground::capture::{
    downsample_2x, flip_rows, thumbnail, GifWriter, Lut, MAX_GIF_BYTES, MAX_TOTAL_BYTES,
};
use playground::draw::DrawList;
use playground::scene::{Result, SceneConfig, SceneKind};
use playground::session::Session;

use super::render::{Renderer, View, BACKGROUND};

/// Size of the render target; the GIF is half of it.
const TARGET: [u32; 2] = [720, 406];
/// Width of the PNG still.
const STILL_WIDTH: usize = 240;

/// Records `scenes` into `out` and checks the media limits; returns the process exit code.
pub async fn record_all(scenes: Vec<SceneKind>, out: PathBuf, frames: Option<u32>) -> i32 {
    match record_scenes(&scenes, &out, frames).await {
        Ok(()) => 0,
        Err(error) => {
            eprintln!("{error}");
            1
        }
    }
}

async fn record_scenes(scenes: &[SceneKind], out: &Path, frames: Option<u32>) -> Result<()> {
    fs::create_dir_all(out)?;
    let lut = Lut::new();
    let mut renderer = Renderer::default();
    let target = render_target_ex(
        TARGET[0],
        TARGET[1],
        RenderTargetParams {
            sample_count: 1,
            depth: true,
        },
    );
    let mut total = 0;
    for &kind in scenes {
        let bytes = record(kind, frames, out, &lut, &mut renderer, &target).await?;
        println!("{}: {} bytes", kind.name(), bytes);
        if bytes > MAX_GIF_BYTES {
            return Err(format!(
                "{}.gif has {bytes} bytes, more than {MAX_GIF_BYTES}",
                kind.name()
            )
            .into());
        }
        total += bytes;
    }
    if scenes.len() == SceneKind::ALL.len() && total > MAX_TOTAL_BYTES {
        return Err(format!("the GIFs have {total} bytes, more than {MAX_TOTAL_BYTES}").into());
    }
    Ok(())
}

/// Records one scene; returns the size of its GIF.
async fn record(
    kind: SceneKind,
    frames: Option<u32>,
    out: &Path,
    lut: &Lut,
    renderer: &mut Renderer,
    target: &RenderTarget,
) -> Result<u64> {
    let mut session = Session::new(kind, SceneConfig::default())?;
    let ticks = frames.unwrap_or_else(|| session.scene().record_ticks());
    let [width, height] = TARGET.map(|size| size as usize);
    let gif_path = out.join(format!("{}.gif", kind.name()));
    let mut gif = GifWriter::new(
        BufWriter::new(File::create(&gif_path)?),
        (width / 2) as u16,
        (height / 2) as u16,
    )?;
    let mut list = DrawList::default();
    let mut still = None;
    for tick in 0..ticks {
        session.tick_scripted()?;
        if tick % 2 == 0 {
            continue;
        }
        session.draw(&mut list)?;
        let view = View {
            camera: session.scene().record_camera(tick),
            target: Some(target.clone()),
            aspect: Some(TARGET[0] as f32 / TARGET[1] as f32),
            clear: true,
        };
        renderer.draw_world(&list, session.scene().visuals(), &view);
        let image = target.texture.get_texture_data();
        let top_down = flip_rows(&image.bytes, width, height);
        if still.is_none() && tick >= ticks / 2 {
            still = Some(top_down.clone());
        }
        gif.push(lut.quantize(&downsample_2x(&top_down, width, height)))?;
        show_progress(kind, tick, ticks, target);
        next_frame().await;
    }
    gif.finish()?;
    let missing = session.missing_milestones();
    if !missing.is_empty() {
        return Err(format!("{} did not show: {}", kind.name(), missing.join(", ")).into());
    }
    if let Some(still) = still {
        let (small, small_height) = thumbnail(&still, width, height, STILL_WIDTH);
        // `export_png` expects OpenGL's row order, bottom row first.
        let image = Image {
            bytes: flip_rows(&small, STILL_WIDTH, small_height),
            width: STILL_WIDTH as u16,
            height: small_height as u16,
        };
        let png_path = out.join(format!("{}.png", kind.name()));
        image.export_png(&png_path.to_string_lossy());
    }
    Ok(fs::metadata(&gif_path)?.len())
}

/// Shows the target scaled to the window with the scene and the progress.
fn show_progress(kind: SceneKind, tick: u32, ticks: u32, target: &RenderTarget) {
    clear_background(BACKGROUND);
    let width = screen_width().min(TARGET[0] as f32);
    let height = width * TARGET[1] as f32 / TARGET[0] as f32;
    draw_texture_ex(
        &target.texture,
        0.0,
        30.0,
        Color::new(1.0, 1.0, 1.0, 1.0),
        DrawTextureParams {
            dest_size: Some(Vec2::new(width, height)),
            flip_y: true,
            ..Default::default()
        },
    );
    let text = format!("recording {}: tick {} of {ticks}", kind.name(), tick + 1);
    draw_text(&text, 10.0, 22.0, 22.0, Color::new(1.0, 1.0, 1.0, 1.0));
}
