//! A timed comparison of oxijolt with Rapier and Avian on scenes ported from Rapier's stress
//! tests. `docs/comparison.md` describes the method and the results; `comparison help` lists
//! the commands.

pub mod cli;
pub mod engine;
pub mod engines;
pub mod matrix;
pub mod measure;
pub mod os;
pub mod quality;
pub mod run;
pub mod scene;
pub mod scenes;
pub mod summary;
