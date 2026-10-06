//! The oxijolt playground: small focused scenes that show what the binding does.
//!
//! Each scene owns a `PhysicsWorld`, built fresh when it is chosen or reset, and reports what to
//! draw from the binding's own readouts: body poses, wheel poses, soft body vertices, debug
//! lines. The library holds everything but the window, so the scenes run and are tested
//! headless; the binary adds the window and the recorder behind the `window` feature.

pub mod camera;
pub mod cli;
pub mod digest;
pub mod draw;
pub mod headless;
pub mod input;
pub mod math;
pub mod mesh;
pub mod scene;
pub mod scenes;
pub mod session;
pub mod terrain;
pub mod tracked;
pub mod visual;
