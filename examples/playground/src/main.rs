//! The playground binary: the window, the recorder and the headless runner.

use std::process::ExitCode;

use playground::cli::{self, Mode};
use playground::headless;
use playground::scene::SceneConfig;

fn main() -> ExitCode {
    let mode = match cli::parse(std::env::args().skip(1)) {
        Ok(mode) => mode,
        Err(message) => {
            eprintln!("{message}\n\n{}{}", cli::USAGE, cli::scene_names());
            return ExitCode::from(2);
        }
    };
    match mode {
        Mode::Help => {
            println!("{}{}", cli::USAGE, cli::scene_names());
            ExitCode::SUCCESS
        }
        Mode::Headless {
            scenes,
            frames,
            threads,
        } => {
            let config = SceneConfig {
                worker_threads: threads,
            };
            for kind in scenes {
                match headless::run(kind, frames, config) {
                    Ok(summary) => println!("{summary}"),
                    Err(error) => {
                        eprintln!("{error}");
                        return ExitCode::FAILURE;
                    }
                }
            }
            ExitCode::SUCCESS
        }
        Mode::Interactive { .. } | Mode::Record { .. } => {
            eprintln!("built without the window feature; use --headless");
            ExitCode::from(2)
        }
    }
}
