//! The committed media: a GIF for every scene at the recorder's size and within the size limits,
//! each a readable looping animation, all of them shown in the playground guide.

use std::fs;

use playground::capture::{GIF_SIZE, MAX_GIF_BYTES, MAX_TOTAL_BYTES};
use playground::cli::default_media_dir;
use playground::scene::SceneKind;

#[test]
fn every_scene_has_its_media_within_the_limits() {
    let dir = default_media_dir();
    let mut total = 0;
    for kind in SceneKind::ALL {
        let gif = dir.join(format!("{}.gif", kind.name()));
        let bytes = fs::read(&gif).unwrap_or_else(|error| panic!("{}: {error}", gif.display()));
        let size = bytes.len() as u64;
        assert!(size <= MAX_GIF_BYTES, "{} has {size} bytes", gif.display());
        total += size;

        let mut decoder = gif::DecodeOptions::new().read_info(&bytes[..]).unwrap();
        assert_eq!(
            [decoder.width(), decoder.height()],
            GIF_SIZE,
            "{}",
            gif.display()
        );
        assert_eq!(decoder.repeat(), gif::Repeat::Infinite, "{}", gif.display());
        let mut frames = 0;
        while decoder.read_next_frame().unwrap().is_some() {
            frames += 1;
        }
        assert!(frames > 10, "{} has {frames} frames", gif.display());
    }
    assert!(total <= MAX_TOTAL_BYTES, "the GIFs have {total} bytes");
}

#[test]
fn the_playground_guide_shows_every_scene() {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../");
    let guide = fs::read_to_string(format!("{root}docs/playground.md")).unwrap();
    for kind in SceneKind::ALL {
        let path = format!("](media/{}.gif)", kind.name());
        assert!(
            guide.contains(&path),
            "docs/playground.md does not show {path}"
        );
    }
    let readme = fs::read_to_string(format!("{root}README.md")).unwrap();
    assert!(readme.contains("](docs/playground.md)"));
}
