//! Tests of the build script's archive decisions (`build/prebuilt.rs`) and source fingerprint
//! (`build/prebuilt_archive.rs`).

use std::fs;
use std::path::Path;

use crate::prebuilt::*;
use crate::prebuilt_archive::{sha256, sources_fingerprint};
use crate::test_support::{config, fake_crate, write_file, TempDir};

const WINDOWS: &str = "x86_64-pc-windows-msvc";
const LINUX: &str = "x86_64-unknown-linux-gnu";

fn v(major: u32, minor: u32) -> Version2 {
    Version2 { major, minor }
}

fn archive(name: &str, config: ArchiveConfig) -> Archive {
    let msvc = config.target.ends_with("-msvc");
    Archive {
        name: name.to_owned(),
        sha256: sha256(name.as_bytes()),
        msvc: msvc.then_some(v(14, 44)),
        glibc: (!msvc).then_some(v(2, 35)),
        config,
    }
}

/// The 16 archives of a release, listed out of name order.
fn release_list() -> ArchiveList {
    let mut archives = Vec::new();
    for target in [LINUX, WINDOWS] {
        for subset in (0..8).rev() {
            archives.push(archive(
                &format!("oxijolt-sys-1.2.0-{target}-{subset}"),
                config(target, subset),
            ));
        }
    }
    ArchiveList {
        version: Some("1.2.0".to_owned()),
        url: Some("https://example.com/releases/download/v1.2.0".to_owned()),
        sources: Some(sha256(b"sources")),
        archives,
    }
}

const HEADER: &str = "format=1\nversion=1.2.0\nurl=https://x\nsources=\
                      0000000000000000000000000000000000000000000000000000000000000000\n";
const LINE: &str =
    "archive=a sha256=0000000000000000000000000000000000000000000000000000000000000001 \
                    target=x86_64-pc-windows-msvc crt=MultiThreadedDLL double_precision=OFF \
                    cross_platform_deterministic=OFF debug_renderer=OFF asserts=OFF msvc=14.44";

#[test]
fn a_release_list_round_trips_sorted_with_lf() {
    let list = release_list();
    let text = write(&list);
    assert!(!text.contains('\r'));
    let parsed = parse(&text).expect("parses");
    let mut sorted = list.clone();
    sorted.archives.sort_by(|a, b| a.name.cmp(&b.name));
    assert_eq!(parsed, sorted);
    assert_eq!(write(&parsed), text);
}

#[test]
fn the_committed_placeholder_lists_no_archives() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../crates/oxijolt-sys/prebuilt.txt");
    let text = fs::read_to_string(path).expect("prebuilt.txt is committed");
    let list = parse(&text).expect("parses");
    assert_eq!(list, ArchiveList::default());
    assert_eq!(
        select(&list, "1.2.0", &config(WINDOWS, 0)),
        Err(Unavailable::NoArchives)
    );
    assert_eq!(write(&list), text.replace("\r\n", "\n"));
}

#[test]
fn select_finds_every_release_configuration() {
    let list = release_list();
    for target in [WINDOWS, LINUX] {
        for subset in 0..8 {
            let found = select(&list, "1.2.0+jolt-5.6.0", &config(target, subset)).expect("listed");
            assert_eq!(found.name, format!("oxijolt-sys-1.2.0-{target}-{subset}"));
        }
    }
}

#[test]
fn select_refuses_unlisted_configurations_and_versions() {
    let list = release_list();
    let mut asserts = config(WINDOWS, 0);
    asserts.asserts = true;
    let mut static_crt = config(WINDOWS, 0);
    static_crt.crt = "MultiThreaded".to_owned();
    for unlisted in [asserts, static_crt, config("aarch64-pc-windows-msvc", 0)] {
        assert_eq!(
            select(&list, "1.2.0", &unlisted),
            Err(Unavailable::NoArchive)
        );
    }
    for other in ["1.2.1", "1.2.0-rc.1", "1.2"] {
        assert_eq!(
            select(&list, other, &config(WINDOWS, 0)),
            Err(Unavailable::VersionDiffers {
                list: "1.2.0".to_owned(),
                crate_: other.to_owned()
            })
        );
    }
    let mut prerelease = list.clone();
    prerelease.version = Some("1.2.0-rc.1+jolt-5.6.0".to_owned());
    assert!(select(&prerelease, "1.2.0-rc.1", &config(WINDOWS, 0)).is_ok());
    assert!(select(&prerelease, "1.2.0", &config(WINDOWS, 0)).is_err());
}

#[test]
fn a_well_formed_line_parses() {
    let list = parse(&format!("# comment\n\n{HEADER}{LINE}\n")).expect("parses");
    assert_eq!(list.archives.len(), 1);
    assert_eq!(list.archives[0].msvc, Some(v(14, 44)));
    assert_eq!(list.archives[0].sha256[31], 1);
}

#[test]
fn malformed_lists_are_refused() {
    let line = |from: &str, to: &str| {
        assert!(LINE.contains(from), "{from}");
        format!("{HEADER}{}\n", LINE.replace(from, to))
    };
    let second = LINE.replace("archive=a ", "archive=b ");
    let cases = [
        ("bad hex", line("0001 ", "000G ")),
        ("short hex", line("0001 ", "01 ")),
        ("uppercase hex", line("0001 ", "000A ")),
        ("missing field", line(" asserts=OFF", "")),
        ("unknown key", line(" msvc=", " rustc=")),
        (
            "repeated key",
            line(" asserts=OFF", " asserts=OFF asserts=OFF"),
        ),
        ("flag spelling", line("asserts=OFF", "asserts=off")),
        ("msvc without version", line(" msvc=14.44", "")),
        ("bad version", line("msvc=14.44", "msvc=14")),
        ("bad name", line("archive=a ", "archive=a/b ")),
        ("format 2", HEADER.replace("format=1", "format=2") + LINE),
        ("no format", HEADER.replace("format=1\n", "") + LINE),
        ("unknown header", format!("{HEADER}channel=stable\n")),
        ("duplicate header", format!("{HEADER}version=1.2.0\n")),
        ("duplicate name", format!("{HEADER}{LINE}\n{LINE}\n")),
        ("duplicate config", format!("{HEADER}{LINE}\n{second}\n")),
        ("no header", format!("format=1\n{LINE}\n")),
        (
            "linux without glibc",
            line(
                "x86_64-pc-windows-msvc crt=MultiThreadedDLL",
                "x86_64-unknown-linux-gnu crt=none",
            ),
        ),
    ];
    for (case, text) in cases {
        assert!(parse(&text).is_err(), "{case} was accepted:\n{text}");
    }
}

#[test]
fn versions_compare_numerically() {
    assert!(Version2::parse("2.4") < Version2::parse("2.39"));
    assert!(Version2::parse("14.43") < Version2::parse("14.44"));
    assert!(Version2::parse("14.50") > Version2::parse("14.44"));
    assert_eq!(Version2::parse("14.44.35207"), Some(v(14, 44)));
    assert!(Version2::parse("14.44.35207") >= Version2::parse("14.44"));
    for bad in ["", "14", "14.", ".44", "a.b", "14.4x", "-1.2"] {
        assert_eq!(Version2::parse(bad), None, "{bad}");
    }
    assert_eq!(v(14, 44).to_string(), "14.44");
    assert_eq!(toolset_from_compiler(v(19, 44)), Some(v(14, 44)));
    assert_eq!(toolset_from_compiler(v(19, 51)), Some(v(14, 51)));
    assert_eq!(toolset_from_compiler(v(18, 0)), None);
}

#[test]
fn modes_parse_case_insensitively() {
    for (value, mode) in [
        (None, Mode::Auto),
        (Some(""), Mode::Auto),
        (Some("auto"), Mode::Auto),
        (Some("AUTO"), Mode::Auto),
        (Some("off"), Mode::Off),
        (Some("Off"), Mode::Off),
        (Some("0"), Mode::Off),
        (Some("false"), Mode::Off),
        (Some("require"), Mode::Require),
        (Some("REQUIRE"), Mode::Require),
    ] {
        assert_eq!(Mode::parse(value), Ok(mode), "{value:?}");
    }
    for bad in ["on", "1", "yes", "required", " off"] {
        let error = Mode::parse(Some(bad)).expect_err(bad);
        assert!(error.contains("auto, off, require"), "{error}");
    }
}

#[test]
fn skip_reasons_follow_their_order() {
    let dir = TempDir::new("skip");
    let reason = |pairs: &[(&'static str, &'static str)]| skip_reason(fake_env(pairs), dir.path());
    assert_eq!(reason(&[]), None);
    assert_eq!(reason(&[("IN_NIX_SHELL", "pure")]), None);
    assert_eq!(reason(&[("CARGO_NET_OFFLINE", "false")]), None);
    assert_eq!(
        reason(&[("CARGO_NET_OFFLINE", "TRUE")]),
        Some(Unavailable::Offline)
    );
    assert_eq!(
        reason(&[("CARGO_NET_OFFLINE", "1")]),
        Some(Unavailable::Offline)
    );
    assert_eq!(
        reason(&[("CARGO_NET_OFFLINE", "true"), ("NIX_BUILD_TOP", "/build")]),
        Some(Unavailable::Offline)
    );
    assert_eq!(
        reason(&[("NIX_BUILD_TOP", "/build")]),
        Some(Unavailable::NixBuild)
    );
    write_file(dir.path(), ".cargo-checksum.json", b"{}");
    assert_eq!(
        reason(&[("NIX_BUILD_TOP", "/build")]),
        Some(Unavailable::Vendored)
    );
}

/// A change to a fake crate directory.
type Edit = dyn Fn(&Path);

#[test]
fn the_fingerprint_ignores_crlf_and_sees_every_input() {
    let lf = TempDir::new("fp-lf");
    let crlf = TempDir::new("fp-crlf");
    fake_crate(lf.path(), false);
    fake_crate(crlf.path(), true);
    let base = sources_fingerprint(lf.path()).expect("fingerprint");
    assert_eq!(sources_fingerprint(crlf.path()), Ok(base));

    let changed = |edit: &dyn Fn(&Path)| {
        let dir = TempDir::new("fp-edit");
        fake_crate(dir.path(), false);
        edit(dir.path());
        sources_fingerprint(dir.path()).expect("fingerprint")
    };
    let edits: [(&str, &Edit); 5] = [
        ("native byte", &|d| {
            write_file(
                d,
                "native/CMakeLists.txt",
                b"native/CMakeLists.txt\nline twO\n",
            )
        }),
        ("vendor C++", &|d| {
            write_file(
                d,
                "vendor/joltc/src/joltc.cpp",
                b"vendor/joltc/src/joltc.cpp\nline two\n\n",
            )
        }),
        ("added file", &|d| {
            write_file(d, "vendor/JoltPhysics/Jolt/Core/New.h", b"")
        }),
        ("build.rs", &|d| {
            write_file(d, "build.rs", b"fn main() {}\n")
        }),
        ("rename", &|d| {
            fs::rename(
                d.join("native/joltc_ext/joltc_ext.cpp"),
                d.join("native/joltc_ext/renamed.cpp"),
            )
            .expect("rename")
        }),
    ];
    for (case, edit) in edits {
        assert_ne!(changed(edit), base, "{case} kept the fingerprint");
    }
    // Files outside the inputs do not count.
    assert_eq!(
        changed(&|d| write_file(d, "src/lib.rs", b"// edited\n")),
        base
    );
    assert_eq!(
        changed(&|d| write_file(d, "vendor/JoltPhysics/Assets/a.bin", b"x")),
        base
    );
}

#[test]
fn a_missing_input_is_reported() {
    let dir = TempDir::new("fp-missing");
    fake_crate(dir.path(), false);
    fs::remove_dir_all(dir.path().join("vendor/joltc/src")).expect("remove");
    assert_eq!(
        sources_fingerprint(dir.path()),
        Err(Unavailable::SourcesMissing("vendor/joltc/src".to_owned()))
    );
}

#[cfg(unix)]
#[test]
fn a_symlinked_input_is_refused() {
    let dir = TempDir::new("fp-link");
    fake_crate(dir.path(), false);
    std::os::unix::fs::symlink(
        dir.path().join("build.rs"),
        dir.path().join("native/link.rs"),
    )
    .expect("symlink");
    assert_eq!(
        sources_fingerprint(dir.path()),
        Err(Unavailable::SourcesMissing("native/link.rs".to_owned()))
    );
}

const LINK_EXE: &str = r"C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207\bin\HostX64\x64\link.exe";

fn fake_env<'a>(pairs: &'a [(&'static str, &'static str)]) -> impl Fn(&str) -> Option<String> + 'a {
    move |name| {
        pairs
            .iter()
            .find(|(k, _)| *k == name)
            .map(|(_, v)| (*v).to_owned())
    }
}

#[test]
fn rustflags_that_choose_a_linker_are_found() {
    for flags in [
        "-Clinker=lld-link",
        "-Copt-level=3\x1f-C\x1flinker=clang",
        "--codegen\x1flinker=cc",
        "--codegen=linker=cc",
    ] {
        assert!(rustflags_set_linker(flags), "{flags:?}");
    }
    for flags in [
        "",
        "-Clinker-plugin-lto",
        "-Clink-arg=-fuse-ld=lld",
        "-C\x1fopt-level=3",
    ] {
        assert!(!rustflags_set_linker(flags), "{flags:?}");
    }
    assert!(rustflags_set_sysroot("-Clink-arg=--sysroot=/opt/sys"));
    assert!(rustflags_set_sysroot("--sysroot\x1f/opt/rust"));
    assert!(!rustflags_set_sysroot("-Ctarget-cpu=native"));
}

#[test]
fn the_msvc_linker_comes_from_overrides_or_discovery() {
    let never = || -> Option<String> { panic!("discovery must not run") };
    assert_eq!(
        effective_msvc_linker(
            fake_env(&[("CARGO_ENCODED_RUSTFLAGS", "-Clinker=link.exe")]),
            never
        ),
        Err(Unavailable::LinkerOverride)
    );
    assert_eq!(
        effective_msvc_linker(fake_env(&[("RUSTC_LINKER", LINK_EXE)]), never),
        Ok(Some(v(14, 44)))
    );
    assert_eq!(
        effective_msvc_linker(fake_env(&[("RUSTC_LINKER", "lld-link")]), never),
        Err(Unavailable::LinkerOverride)
    );
    assert_eq!(
        effective_msvc_linker(fake_env(&[("RUSTC_LINKER", "LINK.EXE")]), never),
        Ok(None)
    );
    assert_eq!(
        effective_msvc_linker(fake_env(&[]), || Some(LINK_EXE.replace('\\', "/"))),
        Ok(Some(v(14, 44)))
    );
    assert_eq!(effective_msvc_linker(fake_env(&[]), || None), Ok(None));
}

#[test]
fn gnu_overrides_are_found() {
    assert!(!gnu_linker_overridden(fake_env(&[])));
    assert!(gnu_linker_overridden(fake_env(&[(
        "RUSTC_LINKER",
        "clang"
    )])));
    assert!(gnu_linker_overridden(fake_env(&[(
        "CARGO_ENCODED_RUSTFLAGS",
        "-Clinker=clang"
    )])));
    assert!(gnu_linker_overridden(fake_env(&[(
        "CARGO_ENCODED_RUSTFLAGS",
        "-Clink-arg=--sysroot=/sys"
    )])));
}

#[test]
fn the_linker_must_be_as_new_as_the_archive() {
    let windows = archive("w", config(WINDOWS, 0));
    let linux = archive("l", config(LINUX, 0));
    let msvc = |found| Linker::Msvc(found);
    assert_eq!(check_linker(&windows, &msvc(Ok(Some(v(14, 44))))), Ok(()));
    assert_eq!(check_linker(&windows, &msvc(Ok(Some(v(14, 50))))), Ok(()));
    assert_eq!(
        check_linker(&windows, &msvc(Ok(Some(v(14, 43))))),
        Err(Unavailable::MsvcTooOld {
            have: v(14, 43),
            need: v(14, 44)
        })
    );
    assert_eq!(
        check_linker(&windows, &msvc(Ok(None))),
        Err(Unavailable::MsvcUnknown)
    );
    assert_eq!(
        check_linker(&windows, &msvc(Err(Unavailable::LinkerOverride))),
        Err(Unavailable::LinkerOverride)
    );

    let gnu = |cross, overridden, glibc| Linker::Gnu {
        cross,
        overridden,
        glibc,
    };
    assert_eq!(
        check_linker(&linux, &gnu(false, false, Some(v(2, 35)))),
        Ok(())
    );
    assert_eq!(
        check_linker(&linux, &gnu(false, false, Some(v(2, 39)))),
        Ok(())
    );
    assert_eq!(
        check_linker(&linux, &gnu(false, false, Some(v(2, 31)))),
        Err(Unavailable::GlibcTooOld {
            have: v(2, 31),
            need: v(2, 35)
        })
    );
    assert_eq!(
        check_linker(&linux, &gnu(false, false, None)),
        Err(Unavailable::GlibcUnknown)
    );
    assert_eq!(
        check_linker(&linux, &gnu(false, true, Some(v(2, 39)))),
        Err(Unavailable::LinkerOverride)
    );
    assert_eq!(
        check_linker(&linux, &gnu(true, false, Some(v(2, 39)))),
        Err(Unavailable::CrossBuild)
    );
}

#[test]
fn msvc_toolsets_are_read_from_linker_paths() {
    assert_eq!(msvc_toolset_of_linker(LINK_EXE), Some(v(14, 44)));
    assert_eq!(
        msvc_toolset_of_linker("c:/vs/vc/tools/msvc/14.51.1/bin/link.exe"),
        Some(v(14, 51))
    );
    assert_eq!(msvc_toolset_of_linker(r"C:\tools\link.exe"), None);
}

#[test]
fn every_reason_reads_as_a_short_sentence() {
    let reasons = [
        Unavailable::Offline,
        Unavailable::Vendored,
        Unavailable::NixBuild,
        Unavailable::FeatureOff,
        Unavailable::NoArchives,
        Unavailable::List("line 2: x".to_owned()),
        Unavailable::SourcesDiffer,
        Unavailable::NoArchive,
        Unavailable::CrtStatic,
        Unavailable::LinkerOverride,
        Unavailable::MsvcTooOld {
            have: v(14, 43),
            need: v(14, 44),
        },
        Unavailable::GlibcTooOld {
            have: v(2, 31),
            need: v(2, 35),
        },
        Unavailable::Checksum,
    ];
    let words = [
        "offline",
        "vendored",
        "Nix",
        "prebuilt feature",
        "no archives",
        "list",
        "native sources",
        "no archive",
        "crt-static",
        "linker",
        "older",
        "older",
        "checksum",
    ];
    for (reason, word) in reasons.iter().zip(words) {
        let text = reason.to_string();
        assert!(text.contains(word), "{text:?} lacks {word:?}");
        assert!(text.len() < 80 && !text.contains('\n'), "{text:?}");
    }
}
