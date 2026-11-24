//! A kernel driver's DRM version as its `struct drm_driver` states it
//! (`major`, `minor`, `patchlevel`): constants of its source, which every
//! client's `DRM_IOCTL_VERSION` answers alike. The build reads them from
//! the driver's macros ([`version`]), so the boot names a driver's version
//! without opening a file of its GPU. This file is also the build's.

/// The version `text` defines with `prefix` macros, as nouveau's
/// `DRIVER_MAJOR`, `DRIVER_MINOR` and `DRIVER_PATCHLEVEL` or amdgpu's
/// `KMS_DRIVER_*`: each `#define {prefix}{part} <number>`.
pub fn version(text: &str, prefix: &str) -> Option<(u32, u32, u32)> {
    let part = |name: &str| {
        text.lines().find_map(|line| {
            let mut words = line.split_whitespace();
            if words.next()? != "#define" {
                return None;
            }
            let macro_name = words.next()?.strip_prefix(prefix)?;
            if macro_name != name {
                return None;
            }
            words.next()?.parse::<u32>().ok()
        })
    };
    Some((part("MAJOR")?, part("MINOR")?, part("PATCHLEVEL")?))
}

#[cfg(test)]
mod tests {
    use super::version;

    #[test]
    fn a_drivers_version_is_read_from_its_macros() {
        let nouveau = "#define DRIVER_AUTHOR\t\t\"Nouveau Project\"\n\
                       #define DRIVER_MAJOR\t\t1\n\
                       #define DRIVER_MINOR\t\t4\n\
                       #define DRIVER_PATCHLEVEL\t2\n";
        assert_eq!(version(nouveau, "DRIVER_"), Some((1, 4, 2)));
        let amdgpu = "#define KMS_DRIVER_MAJOR\t3\n#define KMS_DRIVER_MINOR\t64\n\
                      #define KMS_DRIVER_PATCHLEVEL\t0\n";
        assert_eq!(version(amdgpu, "KMS_DRIVER_"), Some((3, 64, 0)));
        // A prefix that names no part, or a part missing, gives none.
        assert_eq!(version(amdgpu, "DRIVER_"), None);
        assert_eq!(version("#define DRIVER_MAJOR 1\n", "DRIVER_"), None);
    }
}
