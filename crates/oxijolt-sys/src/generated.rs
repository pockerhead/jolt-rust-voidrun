#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// joltc defines `JPH_M_PI` as a literal; bindgen keeps it as written.
#![allow(clippy::approx_constant)]

// The build script picks the committed bindings of the target, or generates them with the
// `bindgen` feature.
include!(env!("JOLTC_BINDINGS"));
