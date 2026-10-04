//! CMake cache entries that follow from the crate features and the target: the options the
//! features switch in Jolt and joltc, and the Android NDK cross toolchain.
//!
//! Pure functions, so the `xtask` tests can check them without running CMake. Shared with
//! those tests through `#[path]`; it has no dependencies.

use std::fmt;

/// The crate features that change how Jolt and joltc are compiled.
pub struct FeatureSwitches {
    pub double_precision: bool,
    pub asserts: bool,
    pub cross_platform_deterministic: bool,
}

impl FeatureSwitches {
    /// The CMake options for these features, as `(option, enabled)` in configure order.
    ///
    /// joltc builds Jolt as a subdirectory of one CMake project, and both declare these options
    /// under the same names, so one cache entry configures both libraries.
    /// `CROSS_PLATFORM_DETERMINISTIC` makes Jolt compile with strict floating-point rules (no
    /// FMA instructions, MSVC `/fp:precise`, GCC and Clang `-ffp-contract=off`) and define
    /// `JPH_CROSS_PLATFORM_DETERMINISTIC` for everything built against it.
    pub fn options(&self) -> [(&'static str, bool); 3] {
        [
            ("DOUBLE_PRECISION", self.double_precision),
            ("USE_ASSERTS", self.asserts),
            (
                "CROSS_PLATFORM_DETERMINISTIC",
                self.cross_platform_deterministic,
            ),
        ]
    }
}

/// Environment variables that may name the Android NDK root, in the order they are read.
pub const NDK_ROOT_VARS: [&str; 3] = ["ANDROID_NDK_HOME", "ANDROID_NDK_ROOT", "ANDROID_NDK"];

/// Lowest Android API level the native libraries are built for.
pub const ANDROID_API_LEVEL: &str = "android-21";

/// CMake generator for Android: the NDK toolchain file supports Ninja and Makefiles, not the
/// Visual Studio generator a Windows host would pick by default.
pub const ANDROID_GENERATOR: &str = "Ninja";

/// Why the Android cross toolchain could not be set up.
#[derive(Debug, PartialEq)]
pub enum AndroidError {
    /// None of [`NDK_ROOT_VARS`] is set.
    NdkNotFound,
    /// The Rust target architecture has no Android ABI.
    UnsupportedArch(String),
}

impl fmt::Display for AndroidError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AndroidError::NdkNotFound => write!(
                f,
                "building for Android needs the NDK: set one of {}",
                NDK_ROOT_VARS.join(", ")
            ),
            AndroidError::UnsupportedArch(arch) => {
                write!(f, "no Android ABI for target architecture {arch}")
            }
        }
    }
}

impl std::error::Error for AndroidError {}

/// The NDK toolchain CMake cross-compiles with for one Android target.
#[derive(Debug, PartialEq)]
pub struct AndroidNdk {
    /// NDK root directory, as read from the environment.
    root: String,
    /// Android ABI name of the Rust target architecture.
    abi: &'static str,
}

impl AndroidNdk {
    /// Finds the NDK through `env_var` (one lookup per name in [`NDK_ROOT_VARS`]) and the ABI
    /// of the Rust `target_arch`.
    pub fn locate(
        target_arch: &str,
        env_var: impl Fn(&str) -> Option<String>,
    ) -> Result<Self, AndroidError> {
        let root = NDK_ROOT_VARS
            .iter()
            .find_map(|name| env_var(name))
            .ok_or(AndroidError::NdkNotFound)?;
        let abi = android_abi(target_arch)
            .ok_or_else(|| AndroidError::UnsupportedArch(target_arch.to_owned()))?;
        Ok(AndroidNdk { root, abi })
    }

    /// The cache entries that route the build through the NDK, in configure order.
    ///
    /// The NDK's toolchain file picks the compiler and sysroot; it reads the ABI and API level
    /// from the `ANDROID_ABI` and `ANDROID_PLATFORM` cache entries, not from the environment.
    pub fn options(&self) -> [(&'static str, String); 3] {
        [
            (
                "CMAKE_TOOLCHAIN_FILE",
                format!("{}/build/cmake/android.toolchain.cmake", self.root),
            ),
            ("ANDROID_ABI", self.abi.to_owned()),
            ("ANDROID_PLATFORM", ANDROID_API_LEVEL.to_owned()),
        ]
    }
}

/// Rust `target_arch` values and the NDK ABI each one builds for.
const ANDROID_ABIS: [(&str, &str); 4] = [
    ("aarch64", "arm64-v8a"),
    ("arm", "armeabi-v7a"),
    ("x86_64", "x86_64"),
    ("x86", "x86"),
];

/// The NDK's ABI name for a Rust `target_arch`.
fn android_abi(target_arch: &str) -> Option<&'static str> {
    ANDROID_ABIS
        .iter()
        .find(|(arch, _)| *arch == target_arch)
        .map(|(_, abi)| *abi)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_feature_combination_switches_its_own_options() {
        for bits in 0..8u8 {
            let switches = FeatureSwitches {
                double_precision: bits & 1 != 0,
                asserts: bits & 2 != 0,
                cross_platform_deterministic: bits & 4 != 0,
            };
            assert_eq!(
                switches.options(),
                [
                    ("DOUBLE_PRECISION", bits & 1 != 0),
                    ("USE_ASSERTS", bits & 2 != 0),
                    ("CROSS_PLATFORM_DETERMINISTIC", bits & 4 != 0),
                ]
            );
        }
    }

    fn only(name: &'static str, value: &'static str) -> impl Fn(&str) -> Option<String> {
        move |asked| (asked == name).then(|| value.to_owned())
    }

    #[test]
    fn each_android_arch_gets_the_ndk_toolchain_and_its_abi() {
        for (arch, abi) in [
            ("aarch64", "arm64-v8a"),
            ("arm", "armeabi-v7a"),
            ("x86_64", "x86_64"),
            ("x86", "x86"),
        ] {
            let ndk = AndroidNdk::locate(arch, only("ANDROID_NDK_HOME", "/opt/ndk")).unwrap();
            assert_eq!(
                ndk.options(),
                [
                    (
                        "CMAKE_TOOLCHAIN_FILE",
                        "/opt/ndk/build/cmake/android.toolchain.cmake".to_owned()
                    ),
                    ("ANDROID_ABI", abi.to_owned()),
                    ("ANDROID_PLATFORM", "android-21".to_owned()),
                ]
            );
        }
        assert_eq!(ANDROID_GENERATOR, "Ninja");
    }

    #[test]
    fn the_ndk_root_comes_from_the_first_variable_set() {
        for name in NDK_ROOT_VARS {
            let ndk = AndroidNdk::locate("aarch64", only(name, "C:/ndk")).unwrap();
            assert_eq!(ndk.root, "C:/ndk");
        }
        let all_set = |name: &str| Some(format!("/{name}"));
        let ndk = AndroidNdk::locate("aarch64", all_set).unwrap();
        assert_eq!(ndk.root, "/ANDROID_NDK_HOME");
        let root_and_ndk = |name: &str| (name != "ANDROID_NDK_HOME").then(|| format!("/{name}"));
        let ndk = AndroidNdk::locate("aarch64", root_and_ndk).unwrap();
        assert_eq!(ndk.root, "/ANDROID_NDK_ROOT");
    }

    #[test]
    fn a_missing_ndk_is_reported_before_the_architecture() {
        assert_eq!(
            AndroidNdk::locate("riscv64", |_| None),
            Err(AndroidError::NdkNotFound)
        );
        assert_eq!(
            AndroidNdk::locate("riscv64", only("ANDROID_NDK", "/ndk")),
            Err(AndroidError::UnsupportedArch("riscv64".to_owned()))
        );
    }
}
