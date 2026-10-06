//! The command line.

use std::path::PathBuf;

use crate::scene::SceneKind;

/// What the program was asked to do.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    /// Open the window on a scene.
    Interactive { scene: SceneKind },
    /// Run scenes' scripts without a window and print their summaries.
    Headless {
        scenes: Vec<SceneKind>,
        frames: u32,
        threads: Option<u32>,
    },
    /// Record scenes' clips into GIFs and PNG stills.
    Record {
        scenes: Vec<SceneKind>,
        out: PathBuf,
        frames: Option<u32>,
    },
    /// Print the usage.
    Help,
}

/// Ticks a headless run takes unless told otherwise: at least every scene's clip, so that a
/// default run checks every milestone.
pub const DEFAULT_FRAMES: u32 = 780;

/// The usage text.
pub const USAGE: &str = "\
usage:
  playground [--scene <name>]                      open the window
  playground --headless --scene <name|all> [--frames N] [--threads N]
                                                   run scenes without a window
  playground --record <name|all> [--out DIR] [--frames N]
                                                   record GIFs and PNG stills
  playground --help

scenes: ";

/// Where recordings go unless told otherwise: `docs/media` of the repository.
pub fn default_media_dir() -> PathBuf {
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../docs/media"))
}

/// The scene names, for messages.
pub fn scene_names() -> String {
    SceneKind::ALL
        .iter()
        .map(|kind| kind.name())
        .collect::<Vec<_>>()
        .join(", ")
}

fn scenes(name: &str) -> Result<Vec<SceneKind>, String> {
    if name == "all" {
        return Ok(SceneKind::ALL.to_vec());
    }
    SceneKind::from_name(name)
        .map(|kind| vec![kind])
        .ok_or_else(|| format!("unknown scene {name:?}; scenes: {}", scene_names()))
}

fn number(flag: &str, value: &str) -> Result<u32, String> {
    match value.parse::<u32>() {
        Ok(n) if n > 0 => Ok(n),
        _ => Err(format!("{flag} needs a positive number, not {value:?}")),
    }
}

/// Parses the arguments after the program name.
pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Mode, String> {
    let mut args = args.into_iter();
    let mut scene = None;
    let mut headless = false;
    let mut record = None;
    let mut frames = None;
    let mut threads = None;
    let mut out = None;
    while let Some(arg) = args.next() {
        let mut value = |flag: &str| args.next().ok_or_else(|| format!("{flag} needs a value"));
        match arg.as_str() {
            "--help" | "-h" => return Ok(Mode::Help),
            "--headless" => headless = true,
            "--scene" => scene = Some(value("--scene")?),
            "--record" => record = Some(value("--record")?),
            "--frames" => frames = Some(number("--frames", &value("--frames")?)?),
            "--threads" => threads = Some(number("--threads", &value("--threads")?)?),
            "--out" => out = Some(PathBuf::from(value("--out")?)),
            other => return Err(format!("unknown argument {other:?}")),
        }
    }
    match (headless, record) {
        (true, Some(_)) => Err("--headless and --record exclude each other".to_owned()),
        (true, None) => {
            if out.is_some() {
                return Err("--out belongs to --record".to_owned());
            }
            let name = scene.ok_or("--headless needs --scene <name|all>")?;
            Ok(Mode::Headless {
                scenes: scenes(&name)?,
                frames: frames.unwrap_or(DEFAULT_FRAMES),
                threads,
            })
        }
        (false, Some(name)) => {
            if scene.is_some() || threads.is_some() {
                return Err("--record takes the scene itself and no --threads".to_owned());
            }
            Ok(Mode::Record {
                scenes: scenes(&name)?,
                out: out.unwrap_or_else(default_media_dir),
                frames,
            })
        }
        (false, None) => {
            if frames.is_some() || threads.is_some() || out.is_some() {
                return Err("--frames, --threads and --out need --headless or --record".to_owned());
            }
            let scene = match scene {
                Some(name) if name != "all" => scenes(&name)?[0],
                Some(_) => return Err("the window shows one scene at a time".to_owned()),
                None => SceneKind::ALL[0],
            };
            Ok(Mode::Interactive { scene })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_str(line: &str) -> Result<Mode, String> {
        parse(line.split_whitespace().map(str::to_owned))
    }

    #[test]
    fn every_form_parses() {
        assert_eq!(
            parse_str(""),
            Ok(Mode::Interactive {
                scene: SceneKind::ALL[0]
            })
        );
        let pile = SceneKind::from_name("pile").unwrap();
        assert_eq!(
            parse_str("--scene pile"),
            Ok(Mode::Interactive { scene: pile })
        );
        assert_eq!(
            parse_str("--headless --scene all --frames 300 --threads 4"),
            Ok(Mode::Headless {
                scenes: SceneKind::ALL.to_vec(),
                frames: 300,
                threads: Some(4)
            })
        );
        assert_eq!(
            parse_str("--headless --scene pile"),
            Ok(Mode::Headless {
                scenes: vec![pile],
                frames: DEFAULT_FRAMES,
                threads: None
            })
        );
        assert_eq!(
            parse_str("--record pile --out media --frames 10"),
            Ok(Mode::Record {
                scenes: vec![pile],
                out: PathBuf::from("media"),
                frames: Some(10)
            })
        );
        assert_eq!(
            parse_str("--record all"),
            Ok(Mode::Record {
                scenes: SceneKind::ALL.to_vec(),
                out: default_media_dir(),
                frames: None
            })
        );
        assert_eq!(parse_str("--help"), Ok(Mode::Help));
    }

    #[test]
    fn bad_arguments_are_refused() {
        for line in [
            "--scene nowhere",
            "--scene all",
            "--headless",
            "--headless --scene pile --frames 0",
            "--headless --scene pile --frames ten",
            "--headless --scene pile --threads",
            "--headless --scene pile --out x",
            "--headless --record pile",
            "--record pile --threads 2",
            "--record pile --scene pile",
            "--frames 3",
            "--fast",
        ] {
            assert!(parse_str(line).is_err(), "{line}");
        }
    }
}
