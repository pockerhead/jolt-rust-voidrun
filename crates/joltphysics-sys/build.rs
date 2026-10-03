//! Builds joltc and Jolt (or validates a prebuilt copy), links them and
//! generates the raw bindings over `joltc.h` and the fork's `joltc_ext.h`.
//!
//! Two ways to get the native libraries:
//! - by default, CMake builds `native/` from the `vendor/` submodules into
//!   `OUT_DIR/joltc`, an install prefix with `lib/`, `include/joltc.h`,
//!   `include/joltc_ext.h` and a manifest describing everything that affects
//!   the ABI;
//! - with `JOLTC_LIB_DIR` set, an already built prefix of that shape is used
//!   after its manifest, archives and header are validated, and CMake is not
//!   run at all.

use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{bail, Context};

/// joltc commit of the `vendor/joltc` submodule. Must match the gitlink; CI checks this.
const JOLTC_COMMIT: &str = "886e088675bae3a086f8318c7803f8ee962c2f2c";
/// Jolt Physics commit of the `vendor/JoltPhysics` submodule. Must match the gitlink; CI checks this.
const JOLT_COMMIT: &str = "e77f175595e64cb44218cc9d9d56fc365ad0e36a";
/// Jolt Physics release of [`JOLT_COMMIT`].
const JOLT_VERSION: &str = "5.6.0";
/// Revision of the fork's joltc additions in `native/joltc_ext/`. Bump it whenever
/// anything there changes, so prebuilt prefixes built before the change are refused.
const JOLTC_EXT_REVISION: &str = "3";
/// Name of the manifest file in an install prefix.
const MANIFEST_FILE: &str = "joltphysics-sys-manifest.txt";

/// joltc functions left out of the bindings.
///
/// They `reinterpret_cast` a `JPH_Mat4*` (4-aligned) to a `JPH::Mat44*`
/// (16-aligned), which is undefined behaviour for most caller-provided arrays.
/// The ragdoll work has to fix the wrapper (copy through aligned storage)
/// before they can come back.
const EXCLUDED_FUNCTIONS: &[&str] = &[
    "JPH_RagdollSettings_DisableParentChildCollisions",
    "JPH_Ragdoll_SetPose2",
    "JPH_Ragdoll_GetPose2",
    "JPH_SkeletonMapper_Initialize",
    "JPH_SkeletonMapper_LockAllTranslations",
    "JPH_SkeletonMapper_LockTranslations",
    "JPH_SkeletonMapper_Map",
    "JPH_SkeletonMapper_MapReverse",
];

/// joltc functions that joltc.cpp defines only under `JPH_DEBUG_RENDERER`; left out of the
/// bindings without the `debug-renderer` feature, so no Rust code can reference a missing symbol.
const DEBUG_RENDERER_FUNCTIONS: &[&str] = &[
    "JPH_Shape_Draw",
    "JPH_PhysicsSystem_Draw.*",
    "JPH_BodyDrawFilter_.*",
    "JPH_DebugRenderer_.*",
];

/// Everything about the target and the crate features that shapes the native build.
struct NativeConfig {
    double_precision: bool,
    cross_platform_deterministic: bool,
    asserts: bool,
    /// Jolt's debug renderer and joltc's drawing functions are compiled in.
    debug_renderer: bool,
    target: String,
    target_os: String,
    target_env: String,
    target_arch: String,
    /// MSVC runtime library in CMake's spelling, or `"none"` off MSVC.
    crt: &'static str,
}

impl NativeConfig {
    /// Reads the target from Cargo's environment; `cfg!` would describe the build host.
    fn from_env() -> Self {
        let feature = |name: &str| env::var_os(format!("CARGO_FEATURE_{name}")).is_some();
        let var = |name: &str| env::var(name).unwrap_or_default();

        let target_env = var("CARGO_CFG_TARGET_ENV");
        let crt_static = var("CARGO_CFG_TARGET_FEATURE")
            .split(',')
            .any(|f| f == "crt-static");
        let crt = match (target_env.as_str(), crt_static) {
            ("msvc", true) => "MultiThreaded",
            ("msvc", false) => "MultiThreadedDLL",
            _ => "none",
        };

        NativeConfig {
            double_precision: feature("DOUBLE_PRECISION"),
            cross_platform_deterministic: feature("CROSS_PLATFORM_DETERMINISTIC"),
            asserts: feature("ASSERTS"),
            debug_renderer: feature("DEBUG_RENDERER"),
            target: var("TARGET"),
            target_os: var("CARGO_CFG_TARGET_OS"),
            target_env,
            target_arch: var("CARGO_CFG_TARGET_ARCH"),
            crt,
        }
    }

    /// Name of the joltc library target, which joltc derives from the precision.
    fn joltc_lib(&self) -> &'static str {
        if self.double_precision {
            "joltc_double"
        } else {
            "joltc"
        }
    }

    /// File name of a static library as the target's toolchain spells it.
    fn archive_name(&self, lib: &str) -> String {
        if self.target_env == "msvc" {
            format!("{lib}.lib")
        } else {
            format!("lib{lib}.a")
        }
    }
}

/// `ON` or `OFF`, as CMake options expect.
fn on_off(value: bool) -> &'static str {
    if value {
        "ON"
    } else {
        "OFF"
    }
}

/// Every input that changes the ABI or the archive contents, in manifest order.
///
/// Both the CMake path (writing) and the prebuilt path (validating) use this
/// one list, so they cannot drift apart.
fn manifest_entries(cfg: &NativeConfig) -> Vec<(&'static str, String)> {
    vec![
        ("format", "1".to_owned()),
        ("joltc_commit", JOLTC_COMMIT.to_owned()),
        ("joltc_ext", JOLTC_EXT_REVISION.to_owned()),
        ("jolt_commit", JOLT_COMMIT.to_owned()),
        ("jolt_version", JOLT_VERSION.to_owned()),
        ("target", cfg.target.clone()),
        ("crt", cfg.crt.to_owned()),
        ("double_precision", on_off(cfg.double_precision).to_owned()),
        (
            "cross_platform_deterministic",
            on_off(cfg.cross_platform_deterministic).to_owned(),
        ),
        ("asserts", on_off(cfg.asserts).to_owned()),
        ("object_layer_bits", "32".to_owned()),
        ("debug_renderer", on_off(cfg.debug_renderer).to_owned()),
        ("floating_point_exceptions", "OFF".to_owned()),
        ("profiler", "OFF".to_owned()),
        ("joltc_lib", cfg.joltc_lib().to_owned()),
    ]
}

/// The manifest file contents: one `key=value` per line.
fn manifest_text(cfg: &NativeConfig) -> String {
    manifest_entries(cfg)
        .iter()
        .map(|(key, value)| format!("{key}={value}\n"))
        .collect()
}

/// Directory of this crate.
fn manifest_dir() -> anyhow::Result<PathBuf> {
    env::var_os("CARGO_MANIFEST_DIR")
        .map(PathBuf::from)
        .context("CARGO_MANIFEST_DIR is not set")
}

/// Path with forward slashes, which CMake accepts on every platform.
fn cmake_path(path: &Path) -> String {
    path.display().to_string().replace('\\', "/")
}

fn main() -> anyhow::Result<()> {
    println!("cargo:rerun-if-env-changed=JOLTC_LIB_DIR");
    println!("cargo:rerun-if-changed=build.rs");

    let cfg = NativeConfig::from_env();
    let prefix = match env::var_os("JOLTC_LIB_DIR") {
        Some(dir) => use_prebuilt(PathBuf::from(dir), &cfg)?,
        None => build_with_cmake(&cfg)?,
    };

    link(&prefix, &cfg);
    generate_bindings(&prefix.join("include").join("joltc_ext.h"), &cfg)
}

/// Builds the native prefix from the submodules with CMake and returns its path.
fn build_with_cmake(cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let crate_dir = manifest_dir()?;
    let joltc_dir = crate_dir.join("vendor").join("joltc");
    let jolt_dir = crate_dir.join("vendor").join("JoltPhysics");

    // Printing any rerun rule replaces Cargo's scan of the whole package, so
    // Rust-only edits do not rerun CMake. Jolt's Assets are left out on purpose.
    for input in [
        "native",
        "vendor/joltc/CMakeLists.txt",
        "vendor/joltc/include",
        "vendor/joltc/src",
        "vendor/JoltPhysics/Build",
        "vendor/JoltPhysics/Jolt",
    ] {
        println!("cargo:rerun-if-changed={input}");
    }

    if !joltc_dir.join("CMakeLists.txt").exists() || !jolt_dir.join("Build").exists() {
        bail!(
            "the joltc and Jolt Physics sources are missing under {}: run `git submodule update --init`, \
             or set JOLTC_LIB_DIR to a prebuilt prefix",
            crate_dir.join("vendor").display()
        );
    }

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").context("OUT_DIR is not set")?);
    let mut config = cmake::Config::new(crate_dir.join("native"));

    // We always have to build in Release.
    //
    // On Windows, Rust always links against the non-debug CRT. Using the Debug
    // profile (which the cmake crate will sometimes pick by default) causes
    // Jolt/joltc to be linked against the debug CRT, causing linker issues.
    //
    // As a nice side effect, this ensures that we build with a known
    // configuration instead of accidentally enabling or disabling extra
    // features just based on opt-level.
    config.profile("Release");
    config.out_dir(out_dir.join("joltc"));

    config.define("JOLTC_SOURCE_DIR", cmake_path(&joltc_dir));
    config.define("JOLT_PHYSICS_ROOT", cmake_path(&jolt_dir));
    // Jolt comes from the submodule; never download anything at build time.
    config.define("FETCHCONTENT_FULLY_DISCONNECTED", "ON");

    // Having IPO/LTO turned on breaks lld on Windows, and MSVC `/GL` objects
    // cannot be linked by other toolchains.
    config.define("INTERPROCEDURAL_OPTIMIZATION", "OFF");
    // Warnings when building Jolt or joltc don't matter to users of joltphysics-sys.
    config.define("ENABLE_ALL_WARNINGS", "OFF");
    // build.rs installs only what it links (see native/CMakeLists.txt).
    config.define("ENABLE_INSTALL", "OFF");
    config.define("GENERATE_DEBUG_SYMBOLS", "OFF");

    // Headless: no GPU compute backends. DX12 would also add system libraries
    // to the link that this script does not emit.
    for backend in [
        "JPH_USE_DX12",
        "JPH_USE_VK",
        "JPH_USE_MTL",
        "JPH_USE_CPU_COMPUTE",
    ] {
        config.define(backend, "OFF");
    }

    // Otherwise every Jolt worker thread enables floating point traps, and
    // Rust callbacks would later run under them.
    config.define("FLOATING_POINT_EXCEPTIONS_ENABLED", "OFF");
    // No profiler API is bound; the instrumentation would only cost time.
    config.define("PROFILER_IN_DEBUG_AND_RELEASE", "OFF");
    // Jolt's debug renderer only with the `debug-renderer` feature. Both switches are needed:
    // DEBUG_RENDERER_IN_DEBUG_AND_RELEASE defaults to ON and would enable it for this Release build.
    config.define("DEBUG_RENDERER_IN_DISTRIBUTION", on_off(cfg.debug_renderer));
    config.define(
        "DEBUG_RENDERER_IN_DEBUG_AND_RELEASE",
        on_off(cfg.debug_renderer),
    );

    // These feature flags affect the compilation of both Jolt and joltc.
    config.define("DOUBLE_PRECISION", on_off(cfg.double_precision));
    config.define("USE_ASSERTS", on_off(cfg.asserts));
    // Pins Jolt's floating-point flags to its cross-platform-deterministic
    // settings: MSVC `/fp:precise` instead of `/fp:fast`, Clang
    // `-ffp-contract=off`, FMADD off on x86. See the feature in Cargo.toml.
    config.define(
        "CROSS_PLATFORM_DETERMINISTIC",
        on_off(cfg.cross_platform_deterministic),
    );

    if cfg.target_env == "msvc" {
        // Jolt and joltc strip `/EHsc` from their own scopes while C++
        // exceptions are off; the layout checks in the wrapper scope need it.
        config.cxxflag("/EHsc");

        // The native libraries use the same CRT as the Rust binary: the DLL
        // CRT by default, the static CRT with `crt-static`. Without this, Jolt
        // picks the static CRT while joltc keeps the DLL one.
        config.define("USE_STATIC_MSVC_RUNTIME_LIBRARY", "OFF");
        config.define("CMAKE_MSVC_RUNTIME_LIBRARY", cfg.crt);
    }

    // Native Android cross-compile setup. Without this, building
    // joltphysics-sys for Android via the standard
    // `cargo ndk --target <android-triple> check` flow fails because
    // (a) cmake-rs defaults to MSBuild on Windows hosts which can't
    // target Android; (b) the NDK's `android.toolchain.cmake` requires
    // `ANDROID_ABI` to be set as a CMake variable (env var is
    // ignored); (c) `ANDROID_NDK_HOME` is the canonical env name set
    // by `nttld/setup-ndk` GitHub Action + cargo-ndk's local
    // workflow.
    //
    // We translate cargo's target arch to NDK's ABI string and route
    // CMake through the NDK toolchain file. `ANDROID_PLATFORM=android-21`
    // is the same baseline most cargo-ndk users target.
    if cfg.target_os == "android" {
        let android_ndk_home = env::var("ANDROID_NDK_HOME")
            .or_else(|_| env::var("ANDROID_NDK_ROOT"))
            .or_else(|_| env::var("ANDROID_NDK"))
            .context(
                "Android cross-compile requires ANDROID_NDK_HOME (or ANDROID_NDK_ROOT / \
                 ANDROID_NDK) to point at the NDK install. Install via \
                 `nttld/setup-ndk@v1` in CI or the Android SDK manager locally.",
            )?;
        let toolchain_file = format!("{android_ndk_home}/build/cmake/android.toolchain.cmake");
        config.define("CMAKE_TOOLCHAIN_FILE", &toolchain_file);
        let android_abi = match cfg.target_arch.as_str() {
            "aarch64" => "arm64-v8a",
            "arm" => "armeabi-v7a",
            "x86" => "x86",
            "x86_64" => "x86_64",
            arch => bail!("unsupported Android target arch: {arch}"),
        };
        config.define("ANDROID_ABI", android_abi);
        config.define("ANDROID_PLATFORM", "android-21");
        // cmake-rs defaults to the host-OS generator (MSBuild on Win);
        // the NDK toolchain only supports Ninja / Makefiles. Force
        // Ninja explicitly.
        config.generator("Ninja");
    }

    let prefix = config.build();
    write_manifest(&prefix, cfg)?;
    Ok(prefix)
}

/// Writes the manifest into the prefix, only when its content changes.
fn write_manifest(prefix: &Path, cfg: &NativeConfig) -> anyhow::Result<()> {
    let path = prefix.join(MANIFEST_FILE);
    let text = manifest_text(cfg);
    if fs::read_to_string(&path).ok().as_deref() != Some(text.as_str()) {
        fs::write(&path, text).with_context(|| format!("cannot write {}", path.display()))?;
    }
    Ok(())
}

/// Validates a prebuilt prefix from `JOLTC_LIB_DIR` and returns it. Never runs CMake.
fn use_prebuilt(dir: PathBuf, cfg: &NativeConfig) -> anyhow::Result<PathBuf> {
    let header = dir.join("include").join("joltc.h");
    let ext_header = dir.join("include").join("joltc_ext.h");
    let manifest = dir.join(MANIFEST_FILE);
    let archives = [
        dir.join("lib").join(cfg.archive_name(cfg.joltc_lib())),
        dir.join("lib").join(cfg.archive_name("Jolt")),
    ];

    let inputs: Vec<&PathBuf> = [&header, &ext_header, &manifest]
        .into_iter()
        .chain(&archives)
        .collect();
    for input in &inputs {
        println!("cargo:rerun-if-changed={}", input.display());
    }
    // A configuration mismatch explains a missing archive (for example
    // joltc_double without double precision), so it is reported first.
    if manifest.is_file() {
        check_manifest(&manifest, cfg)?;
    }
    let missing: Vec<String> = inputs
        .iter()
        .filter(|path| !path.is_file())
        .map(|path| format!("  {}", path.display()))
        .collect();
    if !missing.is_empty() {
        bail!(
            "JOLTC_LIB_DIR={} is not a usable joltphysics-sys prefix; missing:\n{}",
            dir.display(),
            missing.join("\n")
        );
    }

    check_header(&header)?;
    check_ext_header(&ext_header)?;
    Ok(dir)
}

/// Fails unless the prebuilt manifest equals the one this build would write.
fn check_manifest(path: &Path, cfg: &NativeConfig) -> anyhow::Result<()> {
    let text =
        fs::read_to_string(path).with_context(|| format!("cannot read {}", path.display()))?;
    let mut found = BTreeMap::new();
    for line in text.lines().filter(|line| !line.trim().is_empty()) {
        let (key, value) = line
            .split_once('=')
            .with_context(|| format!("{}: malformed line {line:?}", path.display()))?;
        found.insert(key.trim().to_owned(), value.trim().to_owned());
    }

    let expected = manifest_entries(cfg);
    let mut differences = Vec::new();
    for (key, value) in &expected {
        match found.remove(*key) {
            Some(prebuilt) if prebuilt == *value => {}
            Some(prebuilt) => {
                differences.push(format!("  {key}: prebuilt={prebuilt} expected={value}"))
            }
            None => differences.push(format!("  {key}: prebuilt=<missing> expected={value}")),
        }
    }
    for (key, prebuilt) in found {
        differences.push(format!(
            "  {key}: prebuilt={prebuilt} expected=<unknown key>"
        ));
    }

    if !differences.is_empty() {
        bail!(
            "the prebuilt joltc in {} was built for a different configuration:\n{}\n\
             Linking a mismatched native library is undefined behaviour at the FFI boundary. \
             Rebuild the prefix for this configuration or unset JOLTC_LIB_DIR.",
            path.display(),
            differences.join("\n")
        );
    }
    Ok(())
}

/// Fails if the prebuilt header differs from the vendored one, when the submodule is present.
fn check_header(prebuilt: &Path) -> anyhow::Result<()> {
    let vendored = manifest_dir()?
        .join("vendor")
        .join("joltc")
        .join("include")
        .join("joltc.h");
    if !vendored.is_file() {
        println!(
            "cargo:warning=vendor/joltc is not checked out; the prebuilt joltc.h could not be cross-checked"
        );
        return Ok(());
    }
    println!("cargo:rerun-if-changed={}", vendored.display());
    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));
    if read(prebuilt)? != read(&vendored)? {
        bail!(
            "{} differs from {}: the prebuilt prefix is from another joltc version. \
             Rebuild it or unset JOLTC_LIB_DIR.",
            prebuilt.display(),
            vendored.display()
        );
    }
    Ok(())
}

/// Fails if the prebuilt `joltc_ext.h` differs from the one in `native/joltc_ext/`, which
/// always ships with the crate.
fn check_ext_header(prebuilt: &Path) -> anyhow::Result<()> {
    let ours = manifest_dir()?
        .join("native")
        .join("joltc_ext")
        .join("joltc_ext.h");
    println!("cargo:rerun-if-changed={}", ours.display());
    let read =
        |path: &Path| fs::read(path).with_context(|| format!("cannot read {}", path.display()));
    if read(prebuilt)? != read(&ours)? {
        bail!(
            "{} differs from {}: the prebuilt prefix is from another revision of the joltc              additions. Rebuild it or unset JOLTC_LIB_DIR.",
            prebuilt.display(),
            ours.display()
        );
    }
    Ok(())
}

/// Links joltc and Jolt statically and exports the prefix to dependants.
fn link(prefix: &Path, cfg: &NativeConfig) {
    println!(
        "cargo:rustc-link-search=native={}",
        prefix.join("lib").display()
    );
    // joltc depends on Jolt, so it comes first for single-pass linkers.
    println!("cargo:rustc-link-lib=static={}", cfg.joltc_lib());
    println!("cargo:rustc-link-lib=static=Jolt");

    match cfg.target_os.as_str() {
        "macos" | "ios" => println!("cargo:rustc-link-lib=dylib=c++"),
        "linux" => println!("cargo:rustc-link-lib=dylib=stdc++"),
        _ => {}
    }

    println!("cargo:include={}", prefix.join("include").display());
    println!("cargo:root={}", prefix.display());
}

/// Generates `OUT_DIR/bindings.rs` from `joltc_ext.h`, which includes `joltc.h`.
fn generate_bindings(header: &Path, cfg: &NativeConfig) -> anyhow::Result<()> {
    let include_dir = header
        .parent()
        .context("joltc_ext.h has no parent directory")?;
    let mut builder = bindgen::Builder::default()
        .header(header.display().to_string())
        .clang_arg(format!("-I{}", include_dir.display()))
        .allowlist_item("JPH_.*")
        .allowlist_item("JobSystemThreadPoolConfig")
        .default_enum_style(bindgen::EnumVariation::Consts)
        .prepend_enum_name(false)
        // The explicit rerun rules cover the header's sources; the header the
        // CMake path binds lives in OUT_DIR, whose timestamps we do not control.
        .parse_callbacks(Box::new(
            bindgen::CargoCallbacks::new().rerun_on_header_files(false),
        ));

    // The header's only ABI switch. JPH_DEBUG_RENDERER does not change the header; it only
    // decides which functions joltc defines, which DEBUG_RENDERER_FUNCTIONS handles.
    if cfg.double_precision {
        builder = builder.clang_arg("-DJPH_DOUBLE_PRECISION");
    }
    for function in EXCLUDED_FUNCTIONS {
        builder = builder.blocklist_function(function);
    }
    if !cfg.debug_renderer {
        for function in DEBUG_RENDERER_FUNCTIONS {
            builder = builder.blocklist_function(function);
        }
    }

    let bindings = builder
        .generate()
        .context("failed to generate joltc bindings")?;
    let out_path =
        PathBuf::from(env::var_os("OUT_DIR").context("OUT_DIR is not set")?).join("bindings.rs");
    bindings
        .write_to_file(&out_path)
        .with_context(|| format!("cannot write {}", out_path.display()))
}
