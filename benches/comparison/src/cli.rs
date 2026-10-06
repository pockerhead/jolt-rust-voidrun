//! The command line: `comparison <command> [--option value]...`.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::ExitCode;

use crate::engine::Profile;
use crate::engines::Variant;
use crate::measure;
use crate::run::{self, RunArgs};
use crate::scene::Scene;

/// What `help` prints.
pub const USAGE: &str = "\
usage: comparison <command> [--option value]...

commands:
  time      one timed run: --variant --scene --profile --threads [--steps 600]
            [--iterations N] [--run-id ID] [--out DIR]; prints its runs.tsv row
  validate  one validation run with the same options; prints its quality.tsv row
  size-jolt Jolt's buffer probe: --scene [--steps 600]; prints its jolt_sizes.tsv row
  machine   prints the machine line
  help      prints this text

variants: jolt, jolt-4, rapier-par, rapier-serial, rapier-simd8, avian-par, avian-serial
profiles: matched, defaults
";

/// Runs the command in `args` (without the program name) and returns the process status.
pub fn main(args: &[String]) -> ExitCode {
    match run_command(args) {
        Ok(output) => {
            if !output.is_empty() {
                println!("{output}");
            }
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("error: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run_command(args: &[String]) -> Result<String, String> {
    let Some((command, rest)) = args.split_first() else {
        return Ok(USAGE.trim_end().to_owned());
    };
    let options = Options::parse(rest)?;
    match command.as_str() {
        "help" | "--help" | "-h" => Ok(USAGE.trim_end().to_owned()),
        "machine" => Ok(measure::machine_line()),
        "time" => {
            require_build_isa()?;
            run::time(&options.run_args()?)
        }
        "validate" => run::validate(&options.run_args()?),
        "size-jolt" => size_jolt(&options),
        other => Err(format!("unknown command {other:?}\n\n{USAGE}")),
    }
}

#[cfg(feature = "jolt")]
fn size_jolt(options: &Options) -> Result<String, String> {
    run::size_jolt(options.scene()?, options.number("steps", 600)?)
}

#[cfg(not(feature = "jolt"))]
fn size_jolt(_: &Options) -> Result<String, String> {
    Err("this binary was built without jolt".to_owned())
}

/// Refuses timed runs on a CPU without the instructions the builds target (AVX2 and FMA).
pub fn require_build_isa() -> Result<(), String> {
    if measure::cpu_has_build_isa() {
        Ok(())
    } else {
        Err("this CPU lacks AVX2 or FMA, which the timed builds target".to_owned())
    }
}

/// `--name value` pairs.
#[derive(Debug, Default)]
pub struct Options(BTreeMap<String, String>);

impl Options {
    pub fn parse(args: &[String]) -> Result<Self, String> {
        let mut options = BTreeMap::new();
        let mut args = args.iter();
        while let Some(arg) = args.next() {
            let name = arg
                .strip_prefix("--")
                .ok_or_else(|| format!("expected --option, got {arg:?}"))?;
            let value = args
                .next()
                .ok_or_else(|| format!("--{name} needs a value"))?;
            if options.insert(name.to_owned(), value.clone()).is_some() {
                return Err(format!("--{name} given twice"));
            }
        }
        Ok(Self(options))
    }

    pub fn get(&self, name: &str) -> Option<&str> {
        self.0.get(name).map(String::as_str)
    }

    pub fn required(&self, name: &str) -> Result<&str, String> {
        self.get(name)
            .ok_or_else(|| format!("--{name} is required"))
    }

    pub fn number<T: std::str::FromStr>(&self, name: &str, default: T) -> Result<T, String> {
        self.get(name).map_or(Ok(default), |value| {
            value
                .parse()
                .map_err(|_| format!("--{name} {value:?} is not a number"))
        })
    }

    pub fn scene(&self) -> Result<Scene, String> {
        let name = self.required("scene")?;
        Scene::from_name(name).ok_or_else(|| format!("unknown scene {name:?}"))
    }

    fn run_args(&self) -> Result<RunArgs, String> {
        let variant = self.required("variant")?;
        let profile = self.get("profile").unwrap_or("matched");
        let scene = self.scene()?;
        let threads = self.number("threads", 1)?;
        if threads == 0 {
            return Err("--threads must be at least 1".to_owned());
        }
        let variant =
            Variant::from_name(variant).ok_or_else(|| format!("unknown variant {variant:?}"))?;
        let profile =
            Profile::from_name(profile).ok_or_else(|| format!("unknown profile {profile:?}"))?;
        let iterations = self
            .get("iterations")
            .map(|_| self.number("iterations", 0))
            .transpose()?;
        let run_id = self.get("run-id").map_or_else(
            || {
                format!(
                    "{}-{}-{}-t{threads}",
                    variant.name,
                    scene.name(),
                    profile.name()
                )
            },
            str::to_owned,
        );
        Ok(RunArgs {
            run_id,
            variant,
            scene,
            profile,
            threads,
            steps: self.number("steps", 600)?,
            iterations,
            out: PathBuf::from(self.get("out").unwrap_or("comparison-out")),
        })
    }
}
