#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
// joltc defines `JPH_M_PI` as a literal; bindgen keeps it as written.
#![allow(clippy::approx_constant)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
