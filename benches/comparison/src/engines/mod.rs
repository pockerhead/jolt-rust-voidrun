//! The engine adapters and the variants a run names: an engine, how it was built, and its
//! solver iterations.

use std::fmt;

use crate::engine::Engine;

#[cfg(feature = "jolt")]
pub mod jolt;
#[cfg(feature = "rapier")]
pub mod rapier;

/// The engines the comparison knows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EngineKind {
    Jolt,
    Rapier,
    Avian,
}

impl fmt::Display for EngineKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::Jolt => "jolt",
            Self::Rapier => "rapier",
            Self::Avian => "avian",
        })
    }
}

/// What a variant requires of the binary that runs it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Build {
    /// Any build with the engine.
    Any,
    /// Built with the `parallel` feature, without `simd8`.
    Parallel,
    /// Built without the `parallel` feature.
    Serial,
    /// Built with `parallel` and `simd8`.
    Simd8,
}

/// A named configuration of one engine, as it appears in the results.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Variant {
    pub name: &'static str,
    pub engine: EngineKind,
    /// Solver iterations; `None` is the engine's default for the profile.
    pub iterations: Option<u32>,
    pub build: Build,
}

impl Variant {
    /// Every variant the runner knows.
    pub const ALL: [Variant; 7] = [
        Variant::new("jolt", EngineKind::Jolt, None, Build::Any),
        Variant::new("jolt-4", EngineKind::Jolt, Some(4), Build::Any),
        Variant::new("rapier-par", EngineKind::Rapier, None, Build::Parallel),
        Variant::new("rapier-serial", EngineKind::Rapier, None, Build::Serial),
        Variant::new("rapier-simd8", EngineKind::Rapier, None, Build::Simd8),
        Variant::new("avian-par", EngineKind::Avian, None, Build::Parallel),
        Variant::new("avian-serial", EngineKind::Avian, None, Build::Serial),
    ];

    const fn new(
        name: &'static str,
        engine: EngineKind,
        iterations: Option<u32>,
        build: Build,
    ) -> Self {
        Self {
            name,
            engine,
            iterations,
            build,
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|v| v.name == name)
    }

    /// Whether this binary was built with the engine and the features the variant names.
    pub fn check_build(&self) -> Result<(), String> {
        if !compiled_engines().contains(&self.engine) {
            return Err(format!(
                "{}: this binary was built without {}",
                self.name, self.engine
            ));
        }
        let (parallel, simd8) = build_features();
        let ok = match self.build {
            Build::Any => true,
            Build::Parallel => parallel && !simd8,
            Build::Serial => !parallel,
            Build::Simd8 => parallel && simd8,
        };
        if ok {
            Ok(())
        } else {
            Err(format!(
                "{}: this binary has parallel={parallel} simd8={simd8}",
                self.name
            ))
        }
    }
}

/// Whether this binary was built with the `parallel` and `simd8` features.
pub fn build_features() -> (bool, bool) {
    (cfg!(feature = "parallel"), cfg!(feature = "simd8"))
}

/// The engines compiled into this binary.
pub fn compiled_engines() -> Vec<EngineKind> {
    let mut engines = Vec::new();
    if cfg!(feature = "jolt") {
        engines.push(EngineKind::Jolt);
    }
    if cfg!(feature = "rapier") {
        engines.push(EngineKind::Rapier);
    }
    engines
}

/// Work that runs with one engine type, chosen at run time by [`dispatch`].
pub trait WithEngine {
    type Output;
    fn run<E: Engine>(self) -> Self::Output;
}

/// Runs `work` with the adapter of `engine`; an error when this binary does not have it.
pub fn dispatch<W: WithEngine>(engine: EngineKind, work: W) -> Result<W::Output, String> {
    match engine {
        #[cfg(feature = "jolt")]
        EngineKind::Jolt => Ok(work.run::<jolt::Jolt>()),
        #[cfg(feature = "rapier")]
        EngineKind::Rapier => Ok(work.run::<rapier::Rapier>()),
        other => {
            drop(work);
            Err(format!("this binary was built without {other}"))
        }
    }
}
