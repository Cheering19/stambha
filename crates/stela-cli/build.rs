//! With `STELA_EFI` naming a built `stela.efi`, as `cargo xtask dist`
//! sets it, `stela` embeds that image for setup to install (the
//! `stela_boot_image` cfg); without it setup takes `--efi FILE`.
//! `STELA_BUILD`, when set, names the build after the version that
//! `stela --version` prints (`0.5.0 (NAME)`), so a tester build tells itself
//! apart from a release; the version sticks record stays the package's.
use std::{env, fs, path::PathBuf};

const IMAGE: &str = "STELA_EFI";
const BUILD: &str = "STELA_BUILD";

fn main() {
    println!("cargo:rerun-if-env-changed={BUILD}");
    let package = env::var("CARGO_PKG_VERSION").expect("Cargo supplies CARGO_PKG_VERSION");
    let shown = match env::var(BUILD).ok().filter(|name| !name.trim().is_empty()) {
        Some(name) => format!("{package} ({})", name.trim()),
        None => package,
    };
    println!("cargo:rustc-env=STELA_SHOWN_VERSION={shown}");
    println!("cargo::rustc-check-cfg=cfg(stela_boot_image)");
    println!("cargo:rerun-if-env-changed={IMAGE}");
    let Some(image) = env::var_os(IMAGE).filter(|image| !image.is_empty()) else {
        return;
    };
    let image = PathBuf::from(image);
    println!("cargo:rerun-if-changed={}", image.display());
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("Cargo supplies OUT_DIR"));
    fs::copy(&image, out.join("stela.efi"))
        .unwrap_or_else(|error| panic!("cannot read {IMAGE} {}: {error}", image.display()));
    println!("cargo:rustc-cfg=stela_boot_image");
}
