//! A study of which character laws CharacterVirtual's built-in mechanisms carry, and at what
//! cost: scenes, configurations, the fixed caller, the law predicates and the pinned results.
//! `docs/character-study.md` describes the method and the results.

// Each test file compiles this module on its own and uses a different subset.
#![allow(dead_code)]

pub mod config;
pub mod controller;
pub mod expected;
pub mod frame;
pub mod geometry;
pub mod laws;
pub mod matrix;
pub mod passes;
pub mod scenes;
