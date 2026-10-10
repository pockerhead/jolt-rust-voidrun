//! The window: keys and mouse into the session's input, a fixed-step loop, the scene drawn from
//! its draw list, and the menu and help text.

mod record;
mod render;

use macroquad::conf::Conf;
use macroquad::prelude::{
    draw_rectangle, draw_text, get_frame_time, is_key_down, is_key_pressed, is_mouse_button_down,
    is_mouse_button_pressed, mouse_delta_position, mouse_position, mouse_wheel, next_frame,
    screen_height, screen_width, Color, KeyCode, MouseButton,
};
use macroquad::Window;

use playground::camera::CameraHint;
use playground::cli::Mode;
use playground::draw::DrawList;
use playground::input::{Edges, Held};
use playground::scene::{SceneConfig, SceneKind, DT};
use playground::session::Session;

use render::{Renderer, View, DRAW_CALL_CAPACITY};

/// The most ticks one frame runs; a slower frame drops the rest, so the simulation slows down
/// instead of falling ever further behind.
const MAX_TICKS_PER_FRAME: u32 = 4;

/// The keys every scene shares, for the help panel.
const GLOBAL_KEYS: &[(&str, &str)] = &[
    ("1-9, 0, -, =, [", "choose a scene"),
    ("R", "reset the scene"),
    ("P", "pause"),
    ("N", "one tick while paused"),
    ("G", "collider wireframe"),
    ("H", "hide or show this help"),
    ("right drag, wheel", "orbit and zoom the camera"),
    ("Esc", "quit"),
];

/// Opens the window for `mode` and never returns: the process exits when the window closes.
pub fn run(mode: Mode) -> ! {
    let conf = Conf {
        miniquad_conf: macroquad::miniquad::conf::Conf {
            window_title: "oxijolt playground".to_owned(),
            window_width: 1280,
            window_height: 720,
            window_resizable: true,
            sample_count: 4,
            ..Default::default()
        },
        draw_call_vertex_capacity: DRAW_CALL_CAPACITY,
        draw_call_index_capacity: DRAW_CALL_CAPACITY,
        ..Conf::default()
    };
    Window::from_config(conf, async move {
        let code = match mode {
            Mode::Interactive { scene, config } => interactive(scene, config).await,
            Mode::Record {
                scenes,
                out,
                frames,
                config,
            } => record::record_all(scenes, out, frames, config).await,
            Mode::Headless { .. } | Mode::Help => 2,
        };
        std::process::exit(code);
    });
    std::process::exit(0);
}

/// The interactive loop's state.
struct App {
    session: Session,
    /// What every scene is built with, from the command line.
    config: SceneConfig,
    camera: CameraHint,
    paused: bool,
    help: bool,
    accumulator: f32,
    error: Option<String>,
    renderer: Renderer,
    list: DrawList,
}

impl App {
    fn new(kind: SceneKind, config: SceneConfig) -> Result<Self, String> {
        let session = Session::new(kind, config.clone()).map_err(|e| e.to_string())?;
        let camera = session.scene().camera();
        Ok(Self {
            session,
            config,
            camera,
            paused: false,
            help: true,
            accumulator: 0.0,
            error: None,
            renderer: Renderer::default(),
            list: DrawList::default(),
        })
    }

    /// Switches to `kind`, or rebuilds the current scene for `None`.
    fn rebuild(&mut self, kind: Option<SceneKind>) {
        let result = match kind {
            Some(kind) => Session::new(kind, self.config.clone()).map(|session| {
                self.session = session;
            }),
            None => self.session.reset(),
        };
        if let Err(error) = result {
            self.fail(error.to_string());
            return;
        }
        self.camera = self.session.scene().camera();
        self.accumulator = 0.0;
        self.error = None;
        self.paused = false;
    }

    fn fail(&mut self, message: String) {
        self.error = Some(message);
        self.paused = true;
    }

    /// Reads the keys that act on the window and the session rather than on the scene.
    fn global_keys(&mut self) -> bool {
        if is_key_pressed(KeyCode::Escape) {
            return false;
        }
        let digits = [
            KeyCode::Key1,
            KeyCode::Key2,
            KeyCode::Key3,
            KeyCode::Key4,
            KeyCode::Key5,
            KeyCode::Key6,
            KeyCode::Key7,
            KeyCode::Key8,
            KeyCode::Key9,
            KeyCode::Key0,
            KeyCode::Minus,
            KeyCode::Equal,
            KeyCode::LeftBracket,
        ];
        for (key, digit) in digits.into_iter().zip("1234567890-=[".chars()) {
            if is_key_pressed(key) {
                if let Some(kind) = SceneKind::ALL.into_iter().find(|kind| kind.key() == digit) {
                    self.rebuild(Some(kind));
                }
            }
        }
        if is_key_pressed(KeyCode::R) {
            self.rebuild(None);
        }
        if is_key_pressed(KeyCode::P) {
            self.paused = !self.paused;
            self.accumulator = 0.0;
        }
        if is_key_pressed(KeyCode::H) {
            self.help = !self.help;
        }
        true
    }

    /// Orbits and zooms the camera and follows the scene's target.
    fn update_camera(&mut self) {
        if is_mouse_button_down(MouseButton::Right) {
            let delta = mouse_delta_position();
            self.camera.yaw += delta.x * 2.5;
            self.camera.pitch = (self.camera.pitch - delta.y * 2.5).clamp(-0.2, 1.5);
        }
        let (_, wheel) = mouse_wheel();
        if wheel != 0.0 {
            let factor = if wheel > 0.0 { 0.9 } else { 1.0 / 0.9 };
            self.camera.distance = (self.camera.distance * factor).clamp(2.0, 120.0);
        }
        self.camera.target = self.session.scene().camera().target;
    }

    /// The ray under the mouse cursor.
    fn cursor_ray(&self) -> oxijolt::RayCast {
        let (x, y) = mouse_position();
        let (width, height) = (screen_width(), screen_height());
        let ndc = [2.0 * x / width - 1.0, 1.0 - 2.0 * y / height];
        self.camera.ray(ndc, width / height)
    }

    fn held(&self) -> Held {
        let axis = |positive: KeyCode, negative: KeyCode| {
            f32::from(u8::from(is_key_down(positive))) - f32::from(u8::from(is_key_down(negative)))
        };
        let walk = [axis(KeyCode::D, KeyCode::A), axis(KeyCode::W, KeyCode::S)];
        Held {
            walk,
            camera_yaw: self.camera.yaw,
            sprint: is_key_down(KeyCode::LeftShift) || is_key_down(KeyCode::RightShift),
            throttle: walk[1],
            steer: walk[0],
            hand_brake: is_key_down(KeyCode::Space),
            up_down: axis(KeyCode::Up, KeyCode::Down),
            aim: Some(self.cursor_ray()),
        }
    }

    fn edges(&self) -> Edges {
        Edges {
            jump: is_key_pressed(KeyCode::Space),
            switch: is_key_pressed(KeyCode::Tab),
            action: is_key_pressed(KeyCode::E),
            fire: is_key_pressed(KeyCode::F),
            toggle: is_key_pressed(KeyCode::C) || is_key_pressed(KeyCode::T),
            motor: is_key_pressed(KeyCode::M),
            pick: is_mouse_button_pressed(MouseButton::Left).then(|| self.cursor_ray()),
            save: is_key_pressed(KeyCode::K),
            restore: is_key_pressed(KeyCode::L),
            rebase: is_key_pressed(KeyCode::B),
            wireframe: is_key_pressed(KeyCode::G),
        }
    }

    /// Runs the ticks this frame owes, at most [`MAX_TICKS_PER_FRAME`].
    fn advance(&mut self) {
        let held = self.held();
        let edges = self.edges();
        self.session.queue().push_edges(edges);
        let mut ticks = 0;
        if self.paused {
            if is_key_pressed(KeyCode::N) && self.error.is_none() {
                ticks = 1;
            }
        } else {
            self.accumulator += get_frame_time();
            while self.accumulator >= DT && ticks < MAX_TICKS_PER_FRAME {
                self.accumulator -= DT;
                ticks += 1;
            }
            if ticks == MAX_TICKS_PER_FRAME {
                self.accumulator = 0.0;
            }
        }
        for _ in 0..ticks {
            if let Err(error) = self.session.tick_queued(held) {
                self.fail(error.to_string());
                break;
            }
        }
    }

    fn draw(&mut self) {
        if let Err(error) = self.session.draw(&mut self.list) {
            self.fail(error.to_string());
        }
        let view = View {
            camera: self.camera,
            target: None,
            aspect: None,
            clear: true,
            line_pixels: None,
        };
        self.renderer
            .draw_world(&self.list, self.session.scene().visuals(), &view);
        self.draw_text();
    }

    fn draw_text(&self) {
        let scene = self.session.scene();
        let kind = self.session.kind();
        let mut lines: Vec<(String, Color)> = Vec::new();
        let white = Color::new(1.0, 1.0, 1.0, 1.0);
        let state = if self.paused { ", paused" } else { "" };
        let wireframe = if self.session.wireframe() {
            ", wireframe"
        } else {
            ""
        };
        lines.push((
            format!(
                "{} {}  tick {}{state}{wireframe}",
                kind.key(),
                kind.title(),
                self.session.tick_count()
            ),
            white,
        ));
        lines.extend(self.list.hud.iter().map(|line| (line.clone(), white)));
        if let Some(error) = &self.error {
            lines.push((format!("error: {error}"), Color::new(1.0, 0.3, 0.3, 1.0)));
        }
        if self.help {
            lines.push((String::new(), white));
            for other in SceneKind::ALL {
                let colour = if other == kind {
                    Color::new(1.0, 0.85, 0.3, 1.0)
                } else {
                    Color::new(0.8, 0.8, 0.8, 1.0)
                };
                lines.push((format!("{}  {}", other.key(), other.title()), colour));
            }
            lines.push((String::new(), white));
            for (keys, action) in GLOBAL_KEYS.iter().chain(scene.controls()) {
                lines.push((format!("{keys}: {action}"), white));
            }
        } else {
            lines.push(("H: help".to_owned(), white));
        }
        let height = 18.0 * lines.len() as f32 + 12.0;
        draw_rectangle(8.0, 8.0, 470.0, height, Color::new(0.0, 0.0, 0.0, 0.45));
        for (index, (line, colour)) in lines.iter().enumerate() {
            draw_text(line, 16.0, 26.0 + 18.0 * index as f32, 20.0, *colour);
        }
    }
}

/// The interactive loop, until Esc or a failed build of the first scene.
async fn interactive(kind: SceneKind, config: SceneConfig) -> i32 {
    let mut app = match App::new(kind, config) {
        Ok(app) => app,
        Err(error) => {
            eprintln!("{error}");
            return 1;
        }
    };
    loop {
        if !app.global_keys() {
            return 0;
        }
        app.update_camera();
        app.advance();
        app.draw();
        next_frame().await;
    }
}
