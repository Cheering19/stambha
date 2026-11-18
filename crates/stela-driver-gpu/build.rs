#[cfg(feature = "amdgpu-port")]
#[path = "build/amdgpu.rs"]
mod amdgpu;
#[path = "src/drm_version.rs"]
mod drm_version;
#[cfg(feature = "linux-drm")]
#[path = "build/image.rs"]
mod image;
#[cfg(feature = "linux-drm")]
#[path = "build/imports.rs"]
mod imports;
#[cfg(any(feature = "nouveau-port", feature = "xe-port"))]
#[path = "build/kbuild.rs"]
mod kbuild;
#[cfg(feature = "linux-kernel")]
#[path = "build/native.rs"]
mod native;
#[cfg(feature = "nouveau-port")]
#[path = "build/nouveau.rs"]
mod nouveau;
#[path = "build/nouveau_firmware.rs"]
mod nouveau_firmware;
#[cfg(feature = "xe-port")]
#[path = "build/xe.rs"]
mod xe;

/// The Vulkan drivers `cargo xtask gpu nvk` linked into the image's archives,
/// as it lists them beside them: with `metal-linked`, Intel's sets
/// `stela_vulkan_intel` and AMD's `stela_vulkan_amd`, so the crate names only
/// loader entries the image holds. NVK is always there.
fn vulkan_drivers() {
    const OPTIONAL: [(&str, &str); 2] =
        [("intel", "stela_vulkan_intel"), ("amd", "stela_vulkan_amd")];
    for (_, cfg) in OPTIONAL {
        println!("cargo:rustc-check-cfg=cfg({cfg})");
    }
    if std::env::var_os("CARGO_FEATURE_METAL_LINKED").is_none() {
        return;
    }
    let list = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../target/gpu/nvk/vulkan-drivers");
    println!("cargo:rerun-if-changed={}", list.display());
    let listed = std::fs::read_to_string(&list).unwrap_or_default();
    for (name, cfg) in OPTIONAL {
        if listed.lines().any(|linked| linked == name) {
            println!("cargo:rustc-cfg={cfg}");
        }
    }
}

/// Each driver stack's DRM version, read from its driver's source
/// (`stack::Kernel::version`): `None` for a driver whose source this
/// checkout does not hold.
fn drm_versions() {
    const DRM: &str = "../../third_party/linux/drivers/gpu/drm";
    const DRIVERS: [(&str, &str, &str); 3] = [
        ("NOUVEAU", "nouveau/nouveau_drv.h", "DRIVER_"),
        ("AMDGPU", "amd/amdgpu/amdgpu_drv.c", "KMS_DRIVER_"),
        ("XE", "xe/xe_drv.h", "DRIVER_"),
    ];
    let mut text = String::new();
    for (name, file, prefix) in DRIVERS {
        let path = std::path::Path::new(DRM).join(file);
        println!("cargo:rerun-if-changed={}", path.display());
        let version = std::fs::read_to_string(&path)
            .ok()
            .and_then(|source| drm_version::version(&source, prefix));
        text.push_str(&format!(
            "/// `{file}`'s `{prefix}MAJOR`, `MINOR` and `PATCHLEVEL`.\n\
             pub const {name}: Option<(u32, u32, u32)> = {version:?};\n"
        ));
    }
    let output =
        std::path::PathBuf::from(std::env::var_os("OUT_DIR").expect("Cargo supplies OUT_DIR"));
    std::fs::write(output.join("drm_versions.rs"), text).expect("write the drivers' DRM versions");
}

fn main() {
    nouveau_firmware::validate();
    vulkan_drivers();
    drm_versions();
    // Active ports declare their native source inputs below. Without this
    // baseline Cargo watches every vendored file even when no C port is used.
    println!("cargo:rerun-if-changed=build.rs");
    // and its modules, which derive the ports' tables.
    println!("cargo:rerun-if-changed=build");
    #[cfg(feature = "amdgpu-port")]
    amdgpu::tables();
    #[cfg(feature = "xe-port")]
    xe::tables();
    // Clippy reads Rust and links nothing, so it needs no C objects.
    println!("cargo:rerun-if-env-changed=CLIPPY_ARGS");
    #[cfg(feature = "linux-kernel")]
    if std::env::var_os("CLIPPY_ARGS").is_some() {
        let tests = stela_c_build::linux_build::Tests::default();
        tests.write();
        #[cfg(feature = "linux-drm")]
        image::write_parameters(&[]);
        return;
    }
    #[cfg(feature = "linux-kernel")]
    let (linux, mut tests) = (
        native::Linux::new(),
        stela_c_build::linux_build::Tests::default(),
    );
    #[cfg(feature = "linux-kernel")]
    linux.tests(&mut tests);
    #[cfg(feature = "linux-drm")]
    {
        let drivers = vec![
            #[cfg(feature = "nouveau-port")]
            nouveau::port(&linux),
            #[cfg(feature = "xe-port")]
            xe::port(&linux),
            #[cfg(feature = "amdgpu-port")]
            amdgpu::port(&linux),
        ];
        image::link(&linux, drivers);
    }
    #[cfg(feature = "linux-kernel")]
    tests.write();
}
