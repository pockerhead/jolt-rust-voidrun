//! Heightfield terrain for the scenes.

use oxijolt::{HeightFieldSettings, Vec3};

use crate::scene::Result;
use crate::visual::Shaped;

/// A square heightfield of `n` x `n` samples `spacing` metres apart, centred on its body's
/// origin, with the surface at height `height(x, z)` over each sample's world `x` and `z`.
pub fn height_field(n: u32, spacing: f32, height: impl Fn(f32, f32) -> f32) -> Result<Shaped> {
    let half = (n - 1) as f32 * spacing / 2.0;
    let mut samples = Vec::with_capacity((n * n) as usize);
    for z in 0..n {
        for x in 0..n {
            samples.push(height(x as f32 * spacing - half, z as f32 * spacing - half));
        }
    }
    let settings = HeightFieldSettings::default()
        .offset(Vec3::new(-half, 0.0, -half))
        .scale(Vec3::new(spacing, 1.0, spacing));
    Shaped::height_field(n, &samples, &settings)
}

/// Rolling hills `amplitude` metres high around 0, about 25 m between crests.
pub fn rolling(amplitude: f32) -> impl Fn(f32, f32) -> f32 {
    move |x, z| amplitude * ((x / 9.0).sin() * (z / 11.0).cos() + 0.4 * (x / 4.3 + z / 5.1).sin())
}
