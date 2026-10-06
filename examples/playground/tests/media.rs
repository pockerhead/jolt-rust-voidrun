//! The committed media: a GIF and a PNG still for every scene, within the size limits, each GIF
//! a readable looping animation.

use std::fs;

use playground::capture::{MAX_GIF_BYTES, MAX_TOTAL_BYTES};
use playground::cli::default_media_dir;
use playground::scene::SceneKind;

#[test]
fn every_scene_has_its_media_within_the_limits() {
    let dir = default_media_dir();
    let mut total = 0;
    for kind in SceneKind::ALL {
        let gif = dir.join(format!("{}.gif", kind.name()));
        let png = dir.join(format!("{}.png", kind.name()));
        let bytes = fs::read(&gif).unwrap_or_else(|error| panic!("{}: {error}", gif.display()));
        assert!(png.is_file(), "{} is missing", png.display());
        let size = bytes.len() as u64;
        assert!(size <= MAX_GIF_BYTES, "{} has {size} bytes", gif.display());
        total += size;

        let mut decoder = gif::DecodeOptions::new().read_info(&bytes[..]).unwrap();
        assert_eq!(decoder.repeat(), gif::Repeat::Infinite, "{}", gif.display());
        let mut frames = 0;
        while decoder.read_next_frame().unwrap().is_some() {
            frames += 1;
        }
        assert!(frames > 10, "{} has {frames} frames", gif.display());
    }
    assert!(total <= MAX_TOTAL_BYTES, "the GIFs have {total} bytes");
}
