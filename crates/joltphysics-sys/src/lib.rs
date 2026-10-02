//! Unsafe bindings to [Jolt Physics] 5.6.0 through the [joltc] C wrapper.
//!
//! Everything here is `bindgen` output over joltc's `include/joltc.h` and keeps
//! its `JPH_*` names. The safe API lives in the `joltphysics` crate.
//!
//! # Features
//! - `asserts`: compile Jolt with its debug assertions.
//! - `double-precision`: world positions ([`Real`], `JPH_RVec3`, `JPH_RMat4`) use `f64`.
//! - `cross-platform-deterministic`: build Jolt with its cross-platform deterministic
//!   floating point settings (slower; same results across compilers and platforms).
//!
//! # Native build and `JOLTC_LIB_DIR`
//! By default the build script builds joltc and Jolt from the `vendor/` submodules with
//! CMake, always in Release. Set `JOLTC_LIB_DIR` to an install prefix produced by an
//! earlier build (`OUT_DIR/joltc`) to skip CMake: it holds `lib/` with the joltc and Jolt
//! static libraries, `include/joltc.h` and `joltphysics-sys-manifest.txt`. A prefix is specific
//! to the target, the C runtime, the crate features and the pinned joltc and Jolt
//! commits; the build script validates all of these against the manifest and refuses a
//! mismatch.
//!
//! # Using the raw API
//! joltc's own contract, as observed in its source:
//! - Shape creators such as `JPH_BoxShape_Create` return a shape that holds **one**
//!   reference, and `JPH_Shape_Destroy` releases one. Bodies keep their own reference, so
//!   the creator releases its reference once it no longer needs the shape.
//! - `JPH_PhysicsSystem_Destroy` deletes the broad-phase layer interface and both layer
//!   filters passed in `JPH_PhysicsSystemSettings`. Do not destroy them yourself.
//! - `JPH_Init` is guarded by a plain `bool`, and creating or destroying a physics system
//!   writes an unsynchronised global map. Call `JPH_Init` once, and create and destroy
//!   systems from one thread at a time.
//! - `JPH_PhysicsSystem_Update` uses one global temp allocator. Use
//!   `JPH_PhysicsSystem_Update2` with a per-world `JPH_TempAllocator` instead.
//! - `JPH_JobSystemThreadPool_Create` maps `numThreads <= 0` to "as many as there are
//!   hardware threads", so pass a positive worker count when the count matters.
//! - These joltc functions are not bound: `JPH_RagdollSettings_DisableParentChildCollisions`,
//!   `JPH_Ragdoll_SetPose2`, `JPH_Ragdoll_GetPose2`, `JPH_SkeletonMapper_Initialize`,
//!   `JPH_SkeletonMapper_LockAllTranslations`, `JPH_SkeletonMapper_LockTranslations`,
//!   `JPH_SkeletonMapper_Map` and `JPH_SkeletonMapper_MapReverse`. They reinterpret a
//!   4-aligned `JPH_Mat4` array as Jolt's 16-aligned `Mat44`, which is undefined
//!   behaviour for most arrays a caller can pass.
//!
//! [Jolt Physics]: https://github.com/jrouwe/JoltPhysics
//! [joltc]: https://github.com/amerkoleci/joltc

mod generated;
mod layout;

pub use generated::*;

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
#[cfg(feature = "double-precision")]
pub type Real = f64;

/// Scalar type of world positions: `f64` with the `double-precision` feature, `f32` otherwise.
#[cfg(not(feature = "double-precision"))]
pub type Real = f32;
