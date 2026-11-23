//! Source fingerprints for build scripts.
//!
//! A fingerprint is the SHA-256 of every file under a crate's inputs, keyed
//! by its `/`-separated path relative to the crate root and hashed with
//! CRLF line endings folded to LF. Checkouts of one commit on Windows
//! (`core.autocrlf`), Linux and macOS therefore produce the same digest, so
//! translation caches keyed by it move between hosts. Hidden entries (names
//! starting with `.`, such as `.DS_Store` or editor swap files) are skipped.
//!
//! [`Options`] narrow a fingerprint to what a build's output depends on:
//! unit-test modules, which no non-test build compiles, and the locked
//! versions of named registry packages instead of whole lock files.
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

/// What a fingerprint leaves out or adds besides its source files.
#[derive(Debug, Clone, Copy, Default)]
pub struct Options<'a> {
    /// Leave out a Rust module named `tests` (a `tests.rs` and the `tests/`
    /// directory beside it) when its parent declares it only under
    /// `#[cfg(test)]` or `#[cfg(all(test, ...))]`. A test module declared any
    /// other way, or whose parent is not among the inputs, is hashed.
    ///
    /// Also leave out a module declared on the line after
    /// [`NOT_AN_INPUT`] (its `name.rs` and its `name/` directory), with that
    /// line and its declaration: code no output the fingerprint keys depends
    /// on, such as the compiler's HLE composites, which are never stored.
    /// Its parent then hashes as it would without the module.
    pub skip_test_modules: bool,
    /// Lock files, relative to the crate root, and the registry packages whose
    /// locked versions and checksums (with every package they depend on) the
    /// fingerprint covers. A missing lock file is hashed as missing.
    pub locked: &'a [(&'a str, &'a [&'a str])],
}

/// The comment that, alone on the line before a module's declaration
/// (`mod name;`, `pub mod name;` or `pub(crate) mod name;`), leaves the
/// module out of a fingerprint that skips test modules
/// ([`Options::skip_test_modules`]). Text after it on its line says why.
pub const NOT_AN_INPUT: &str = "// fingerprint: not a translation input";

/// Prints `cargo:rerun-if-changed` for each input of the crate whose build
/// script is running and sets the environment variable `variable` to the
/// inputs' fingerprint for the crate's compilation.
pub fn emit(variable: &str, inputs: &[&str]) {
    emit_with(variable, inputs, &Options::default());
}

/// [`emit`] narrowed by `options`.
pub fn emit_with(variable: &str, inputs: &[&str], options: &Options) {
    for input in inputs {
        println!("cargo:rerun-if-changed={input}");
    }
    for (lock, _) in options.locked {
        println!("cargo:rerun-if-changed={lock}");
    }
    let root = std::env::var_os("CARGO_MANIFEST_DIR").expect("build script environment");
    println!(
        "cargo:rustc-env={variable}={}",
        source_sha256_with(Path::new(&root), inputs, options)
    );
}

/// The lowercase hex SHA-256 of the files under `inputs`, each a file or
/// directory relative to `root`.
pub fn source_sha256(root: &Path, inputs: &[&str]) -> String {
    source_sha256_with(root, inputs, &Options::default())
}

/// [`source_sha256`] narrowed by `options`. Without options the digest is
/// [`source_sha256`]'s.
pub fn source_sha256_with(root: &Path, inputs: &[&str], options: &Options) -> String {
    let mut files = Vec::new();
    for input in inputs {
        collect(
            root,
            &mut input.split('/').map(str::to_owned).collect(),
            &mut files,
        );
    }
    files.sort();
    files.dedup();
    // Files whose marked module declarations are left out of their content.
    let mut marked = Vec::new();
    if options.skip_test_modules {
        let mut skipped: Vec<String> = files
            .iter()
            .filter_map(|file| test_module_directory(root, file))
            .collect();
        let mut skipped_files = Vec::new();
        for file in &files {
            let modules = unmarked_modules(root, file);
            if modules.is_empty() {
                continue;
            }
            let directory = module_directory(file);
            for name in modules {
                skipped_files.push(format!("{directory}/{name}.rs"));
                skipped.push(format!("{directory}/{name}"));
            }
            marked.push(file.clone());
        }
        files.retain(|file| {
            test_module_directory(root, file).is_none()
                && !skipped_files.contains(file)
                && !skipped
                    .iter()
                    .any(|directory| file.starts_with(&format!("{directory}/")))
        });
    }
    let mut hasher = Sha256::new();
    for file in files {
        let mut content = normalize(&fs::read(root.join(&file)).expect("readable source file"));
        if marked.contains(&file) {
            content = without_marked_modules(&content);
        }
        hasher.update(file.as_bytes());
        hasher.update([0]);
        hasher.update((content.len() as u64).to_le_bytes());
        hasher.update(&content);
    }
    for (lock, packages) in options.locked {
        hasher.update(b"lock\0");
        hasher.update(lock.as_bytes());
        hasher.update([0]);
        match fs::read_to_string(root.join(lock)) {
            Ok(text) => {
                for line in locked_closure(&normalize_text(&text), packages) {
                    hasher.update(line.as_bytes());
                    hasher.update([0]);
                }
            }
            Err(_) => hasher.update(b"missing\0"),
        }
    }
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// The directory a Rust file's submodules live in: its own directory for a
/// crate root or `mod.rs`, else the directory named as the file.
fn module_directory(file: &str) -> String {
    let (parent, name) = file.rsplit_once('/').unwrap_or(("", file));
    match name {
        "lib.rs" | "main.rs" | "mod.rs" => parent.to_owned(),
        _ => file.strip_suffix(".rs").unwrap_or(file).to_owned(),
    }
}

/// The name a declaration line `mod name;` (or `pub` / `pub(crate)` /
/// `pub(super)` before it) declares.
fn declared_module(line: &str) -> Option<&str> {
    let line = line.trim();
    let rest = ["mod ", "pub mod ", "pub(crate) mod ", "pub(super) mod "]
        .iter()
        .find_map(|prefix| line.strip_prefix(prefix))?;
    let name = rest.strip_suffix(';')?;
    name.chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_')
        .then_some(name)
}

/// The modules `file` (a `.rs`) declares on the line after a
/// [`NOT_AN_INPUT`] comment.
fn unmarked_modules(root: &Path, file: &str) -> Vec<String> {
    if !file.ends_with(".rs") {
        return Vec::new();
    }
    let Ok(text) = fs::read_to_string(root.join(file)) else {
        return Vec::new();
    };
    let text = normalize_text(&text);
    let lines: Vec<&str> = text.lines().collect();
    lines
        .windows(2)
        .filter(|pair| pair[0].trim().starts_with(NOT_AN_INPUT))
        .filter_map(|pair| declared_module(pair[1]).map(str::to_owned))
        .collect()
}

/// `content` without each [`NOT_AN_INPUT`] line and the module declaration
/// after it, every other byte as it was.
fn without_marked_modules(content: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(content.len());
    let lines: Vec<&[u8]> = content.split_inclusive(|&byte| byte == b'\n').collect();
    let mut index = 0;
    while index < lines.len() {
        let line = String::from_utf8_lossy(lines[index]);
        let next = lines
            .get(index + 1)
            .map(|next| String::from_utf8_lossy(next));
        if line.trim().starts_with(NOT_AN_INPUT)
            && next.as_deref().and_then(declared_module).is_some()
        {
            index += 2;
            continue;
        }
        out.extend_from_slice(lines[index]);
        index += 1;
    }
    out
}

/// When `file` is a `tests.rs` that only test builds compile, the path of
/// the `tests` directory its submodules live in.
fn test_module_directory(root: &Path, file: &str) -> Option<String> {
    let directory = file.strip_suffix("/tests.rs")?;
    let parents = match directory.rsplit_once('/') {
        // `src/tests.rs` belongs to the crate root.
        _ if directory == "src" => vec!["src/lib.rs".to_owned(), "src/main.rs".to_owned()],
        _ => vec![format!("{directory}.rs"), format!("{directory}/mod.rs")],
    };
    let declared = parents.iter().any(|parent| {
        fs::read_to_string(root.join(parent))
            .is_ok_and(|text| declares_test_only_module(&normalize_text(&text)))
    });
    declared.then(|| format!("{directory}/tests"))
}

/// Whether `text` declares `mod tests;` only for test builds: the attributes
/// directly before it include `#[cfg(test)]` or `#[cfg(all(test, ...))]`.
fn declares_test_only_module(text: &str) -> bool {
    let lines: Vec<&str> = text.lines().map(str::trim).collect();
    lines.iter().enumerate().any(|(index, line)| {
        let Some(declaration) = line.strip_suffix("mod tests;") else {
            return false;
        };
        if !declaration.is_empty() && !declaration.starts_with("pub") {
            return false;
        }
        // The attribute lines above it, up to the previous item's end, joined
        // without whitespace.
        let mut attributes = String::new();
        for above in lines[..index].iter().rev() {
            if above.is_empty()
                || above.starts_with("//")
                || above.ends_with(';')
                || above.ends_with('{')
                || above.ends_with('}')
            {
                break;
            }
            attributes.insert_str(0, above);
        }
        let attributes: String = attributes.split_whitespace().collect();
        attributes.contains("#[cfg(test)]") || attributes.contains("#[cfg(all(test,")
    })
}

/// The `name version checksum` lines of `packages` and every package they
/// depend on in the lock file `text`, sorted.
fn locked_closure(text: &str, packages: &[&str]) -> Vec<String> {
    struct Package {
        name: String,
        version: String,
        checksum: String,
        dependencies: Vec<String>,
    }
    let mut locked: Vec<Package> = Vec::new();
    let mut in_dependencies = false;
    for line in text.lines() {
        let line = line.trim();
        if line == "[[package]]" {
            locked.push(Package {
                name: String::new(),
                version: String::new(),
                checksum: String::new(),
                dependencies: Vec::new(),
            });
            in_dependencies = false;
            continue;
        }
        let Some(package) = locked.last_mut() else {
            continue;
        };
        let quoted = |value: &str| value.trim().trim_matches('"').to_owned();
        if in_dependencies {
            if line.starts_with(']') {
                in_dependencies = false;
            } else {
                package
                    .dependencies
                    .push(quoted(line.trim_end_matches(',')));
            }
        } else if let Some(value) = line.strip_prefix("name = ") {
            package.name = quoted(value);
        } else if let Some(value) = line.strip_prefix("version = ") {
            package.version = quoted(value);
        } else if let Some(value) = line.strip_prefix("checksum = ") {
            package.checksum = quoted(value);
        } else if let Some(items) = line.strip_prefix("dependencies = [") {
            match items.strip_suffix(']') {
                Some(items) => package
                    .dependencies
                    .extend(items.split(',').map(quoted).filter(|item| !item.is_empty())),
                None => in_dependencies = true,
            }
        }
    }
    // A dependency names a package, or a package and its version when the
    // lock holds several of that name.
    let matches = |reference: &str, package: &Package| match reference.split_once(' ') {
        Some((name, version)) => package.name == name && package.version == version,
        None => package.name == reference,
    };
    let mut wanted: Vec<String> = packages.iter().map(|name| (*name).to_owned()).collect();
    let mut taken = vec![false; locked.len()];
    while let Some(reference) = wanted.pop() {
        for (index, package) in locked.iter().enumerate() {
            if !taken[index] && matches(&reference, package) {
                taken[index] = true;
                wanted.extend(package.dependencies.iter().cloned());
            }
        }
    }
    let mut lines: Vec<String> = locked
        .iter()
        .zip(taken)
        .filter(|(_, taken)| *taken)
        .map(|(package, _)| format!("{} {} {}", package.name, package.version, package.checksum))
        .collect();
    lines.sort();
    lines
}

fn normalize_text(text: &str) -> String {
    text.replace("\r\n", "\n")
}

/// Adds the `/`-joined relative paths of the files at `components` to `files`.
fn collect(root: &Path, components: &mut Vec<String>, files: &mut Vec<String>) {
    let path = components
        .iter()
        .fold(root.to_owned(), |path, part| path.join(part));
    if path.is_dir() {
        for entry in fs::read_dir(&path).expect("readable source directory") {
            let name = entry.expect("source entry").file_name();
            let name = name.into_string().expect("UTF-8 source file name");
            if name.starts_with('.') {
                continue;
            }
            components.push(name);
            collect(root, components, files);
            components.pop();
        }
    } else {
        files.push(components.join("/"));
    }
}

/// `bytes` with each CRLF pair replaced by LF.
fn normalize(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len());
    for (index, &byte) in bytes.iter().enumerate() {
        if byte == b'\r' && bytes.get(index + 1) == Some(&b'\n') {
            continue;
        }
        out.push(byte);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree(name: &str, files: &[(&str, &[u8])]) -> std::path::PathBuf {
        let root =
            std::env::temp_dir().join(format!("stela-fingerprint-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        for (path, content) in files {
            let path = root.join(path);
            fs::create_dir_all(path.parent().unwrap()).unwrap();
            fs::write(path, content).unwrap();
        }
        root
    }

    #[test]
    fn line_endings_and_hidden_entries_leave_the_hash_unchanged() {
        type Files<'a> = &'a [(&'a str, &'a [u8])];
        let cases: [(&str, Files, Files); 2] = [
            (
                "crlf",
                &[("src/a.rs", b"fn a() {}\n"), ("src/b/c.rs", b"x\n")],
                &[("src/a.rs", b"fn a() {}\r\n"), ("src/b/c.rs", b"x\r\n")],
            ),
            (
                "hidden",
                &[("src/a.rs", b"a")],
                &[("src/a.rs", b"a"), ("src/.DS_Store", b"\0")],
            ),
        ];
        for (case, plain, variant) in cases {
            let plain = tree(&format!("{case}-plain"), plain);
            let variant = tree(&format!("{case}-variant"), variant);
            assert_eq!(
                source_sha256(&plain, &["src"]),
                source_sha256(&variant, &["src"]),
                "{case}"
            );
            fs::remove_dir_all(plain).unwrap();
            fs::remove_dir_all(variant).unwrap();
        }
    }

    #[test]
    fn content_and_path_changes_change_the_digest() {
        let base = tree("base", &[("src/a.rs", b"a")]);
        let edited = tree("edited", &[("src/a.rs", b"b")]);
        let moved = tree("moved", &[("src/b.rs", b"a")]);
        let digest = source_sha256(&base, &["src"]);
        assert_ne!(digest, source_sha256(&edited, &["src"]));
        assert_ne!(digest, source_sha256(&moved, &["src"]));
        for root in [base, edited, moved] {
            fs::remove_dir_all(root).unwrap();
        }
    }

    #[test]
    fn sibling_semantic_crates_invalidate_for_nested_source_and_manifest_changes() {
        let workspace = tree(
            "semantic-dependencies",
            &[
                ("engine/src/lib.rs", b"engine"),
                ("contracts/Cargo.toml", b"contracts"),
                ("contracts/src/generated/features.rs", b"feature-a\n"),
                ("formats/Cargo.toml", b"formats"),
                ("formats/src/payload/manifest.rs", b"format-a"),
            ],
        );
        let root = workspace.join("engine");
        let inputs = [
            "src",
            "../contracts/Cargo.toml",
            "../contracts/src",
            "../formats/Cargo.toml",
            "../formats/src",
        ];
        let mut digest = source_sha256(&root, &inputs);
        fs::write(
            workspace.join("contracts/src/generated/features.rs"),
            b"feature-a\r\n",
        )
        .unwrap();
        assert_eq!(digest, source_sha256(&root, &inputs));
        for (path, content) in [
            (
                "contracts/src/generated/features.rs",
                b"feature-b".as_slice(),
            ),
            ("formats/src/payload/manifest.rs", b"format-b".as_slice()),
            ("formats/Cargo.toml", b"new dependency".as_slice()),
            ("contracts/src/new_effect.rs", b"new module".as_slice()),
        ] {
            fs::write(workspace.join(path), content).unwrap();
            let changed = source_sha256(&root, &inputs);
            assert_ne!(digest, changed, "semantic input {path}");
            digest = changed;
        }
        fs::remove_dir_all(workspace).unwrap();
    }

    #[test]
    fn a_marked_module_is_left_out_with_its_declaration() {
        let options = Options {
            skip_test_modules: true,
            locked: &[],
        };
        let without = tree(
            "marked-without",
            &[
                ("src/lib.rs", b"mod a;\nmod c;\n"),
                ("src/a.rs", b"mod b;\nfn a() {}\n"),
                ("src/a/b.rs", b"b"),
            ],
        );
        let with = tree(
            "marked-with",
            &[
                ("src/lib.rs", b"mod a;\nmod c;\n"),
                (
                    "src/a.rs",
                    b"mod b;\n// fingerprint: not a translation input: never stored\npub mod hle;\nfn a() {}\n",
                ),
                ("src/a/b.rs", b"b"),
                ("src/a/hle.rs", b"one"),
                ("src/a/hle/deep.rs", b"two"),
            ],
        );
        let digest = source_sha256_with(&without, &["src"], &options);
        assert_eq!(digest, source_sha256_with(&with, &["src"], &options));
        // Its files change nothing; every other file still counts.
        fs::write(with.join("src/a/hle/deep.rs"), b"edited").unwrap();
        fs::write(with.join("src/a/hle/new.rs"), b"new").unwrap();
        assert_eq!(digest, source_sha256_with(&with, &["src"], &options));
        fs::write(with.join("src/a/b.rs"), b"edited").unwrap();
        assert_ne!(digest, source_sha256_with(&with, &["src"], &options));
        // Unmarked, or without the option, the module counts.
        let unmarked = tree(
            "marked-unmarked",
            &[
                ("src/lib.rs", b"mod a;\nmod c;\n"),
                ("src/a.rs", b"mod b;\npub mod hle;\nfn a() {}\n"),
                ("src/a/b.rs", b"b"),
                ("src/a/hle.rs", b"one"),
            ],
        );
        assert_ne!(digest, source_sha256_with(&unmarked, &["src"], &options));
        assert_ne!(
            source_sha256(&without, &["src"]),
            source_sha256(&with, &["src"])
        );
        // The marker before anything but a declaration leaves its line.
        assert_eq!(
            without_marked_modules(b"// fingerprint: not a translation input\nfn a() {}\n"),
            b"// fingerprint: not a translation input\nfn a() {}\n"
        );
        assert_eq!(module_directory("src/lib.rs"), "src");
        assert_eq!(module_directory("src/x86_64.rs"), "src/x86_64");
        assert_eq!(module_directory("src/a/mod.rs"), "src/a");
        for root in [without, with, unmarked] {
            fs::remove_dir_all(root).unwrap();
        }
    }

    #[test]
    fn test_modules_are_skipped_only_when_declared_for_tests() {
        let options = Options {
            skip_test_modules: true,
            locked: &[],
        };
        let files: &[(&str, &[u8])] = &[
            ("src/lib.rs", b"mod a;\n#[cfg(test)]\nmod tests;\n"),
            ("src/tests.rs", b"one"),
            (
                "src/a.rs",
                b"pub fn a() {}\n\n#[cfg(all(\n    test,\n    target_arch = \"x86_64\"\n))]\npub(super) mod tests;\n",
            ),
            ("src/a/tests.rs", b"two"),
            ("src/a/tests/deep.rs", b"three"),
            ("src/b.rs", b"#[cfg(any(test, feature = \"x\"))]\nmod tests;\n"),
            ("src/b/tests.rs", b"four"),
        ];
        let root = tree("tests-skipped", files);
        let digest = source_sha256_with(&root, &["src"], &options);
        assert_ne!(digest, source_sha256(&root, &["src"]));
        for (path, content) in [
            ("src/tests.rs", b"edited".as_slice()),
            ("src/a/tests.rs", b"edited"),
            ("src/a/tests/deep.rs", b"edited"),
            ("src/a/tests/new.rs", b"new"),
        ] {
            fs::write(root.join(path), content).unwrap();
            assert_eq!(
                digest,
                source_sha256_with(&root, &["src"], &options),
                "{path}"
            );
        }
        // A module that a non-test build may compile is hashed, and so is
        // every real source.
        for path in ["src/b/tests.rs", "src/a.rs"] {
            let before = source_sha256_with(&root, &["src"], &options);
            fs::write(root.join(path), b"edited\n#[cfg(test)]\nmod tests;\n").unwrap();
            assert_ne!(
                before,
                source_sha256_with(&root, &["src"], &options),
                "{path}"
            );
        }
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn locked_packages_cover_their_dependency_closure_alone() {
        const LOCK: &str = "version = 4\n\n\
            [[package]]\nname = \"apfloat\"\nversion = \"0.2.3\"\nsource = \"registry\"\nchecksum = \"aa\"\ndependencies = [\n \"bitflags 2.1.0\",\n \"smallvec\",\n]\n\n\
            [[package]]\nname = \"bitflags\"\nversion = \"1.3.2\"\nchecksum = \"b1\"\n\n\
            [[package]]\nname = \"bitflags\"\nversion = \"2.1.0\"\nchecksum = \"b2\"\n\n\
            [[package]]\nname = \"smallvec\"\nversion = \"1.0.0\"\nchecksum = \"s1\"\n\n\
            [[package]]\nname = \"tls\"\nversion = \"9.9.9\"\nchecksum = \"t9\"\ndependencies = [\"bitflags 1.3.2\"]\n";
        assert_eq!(
            locked_closure(LOCK, &["apfloat"]),
            ["apfloat 0.2.3 aa", "bitflags 2.1.0 b2", "smallvec 1.0.0 s1"]
        );
        let options = Options {
            skip_test_modules: false,
            locked: &[("Cargo.lock", &["apfloat"]), ("missing.lock", &["apfloat"])],
        };
        let root = tree(
            "locked",
            &[("src/a.rs", b"a"), ("Cargo.lock", LOCK.as_bytes())],
        );
        let digest = source_sha256_with(&root, &["src"], &options);
        // An unrelated package's update keeps the digest.
        let unrelated = LOCK.replace("9.9.9", "9.9.10").replace("b1", "b3");
        fs::write(root.join("Cargo.lock"), &unrelated).unwrap();
        assert_eq!(digest, source_sha256_with(&root, &["src"], &options));
        // A dependency of a named package changes it.
        fs::write(root.join("Cargo.lock"), unrelated.replace("s1", "s2")).unwrap();
        assert_ne!(digest, source_sha256_with(&root, &["src"], &options));
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn lone_carriage_returns_are_kept() {
        assert_eq!(normalize(b"a\r\nb\rc\r"), b"a\nb\rc\r");
    }
}
