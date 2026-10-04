//! The drawing functions are bound exactly when the `debug-renderer` feature is on.

/// The generated bindings of this build.
const BINDINGS: &str = include_str!(env!("JOLTC_BINDINGS"));

/// One function of each family that joltc defines only under `JPH_DEBUG_RENDERER`.
const DEBUG_RENDERER_FUNCTIONS: [&str; 4] = [
    "pub fn JPH_Shape_Draw(",
    "pub fn JPH_PhysicsSystem_DrawBodies(",
    "pub fn JPH_BodyDrawFilter_Create(",
    "pub fn JPH_DebugRenderer_Create(",
];

#[test]
fn debug_renderer_functions_follow_the_feature() {
    for function in DEBUG_RENDERER_FUNCTIONS {
        assert_eq!(
            BINDINGS.contains(function),
            cfg!(feature = "debug-renderer"),
            "{function}"
        );
    }
    assert!(BINDINGS.contains("pub fn JPH_DrawSettings_InitDefault("));
}
