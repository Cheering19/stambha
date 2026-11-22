use super::*;
use alloc::{string::ToString, vec};

fn vertex() -> ShaderImage {
    ShaderImage {
        stage: "vertex".to_string(),
        header: (0..32).map(|word| 0x0006_1060 + word).collect(),
        geometry_header: Vec::new(),
        code: (0..=255).collect(),
        statistics: vec![
            ("Instruction count".to_string(), 12),
            ("Number of GPRs".to_string(), 24),
            ("SLM size".to_string(), 0),
            ("Program header address".to_string(), 0x3_0000_0080),
            ("Uploaded by queue".to_string(), 1),
            ("Code size".to_string(), 256),
        ],
    }
}

/// The code's hash as another SHA-256 implementation gives it, which a log
/// reader's `sha256sum` of the hex matches.
fn reference_sha256(bytes: &[u8]) -> String {
    use sha2::Digest;
    sha2::Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// Only the first [`REPORTED_FAULTS`] faults of a boot are dumped, each
/// numbered from 1, and the limiter never wraps.
#[test]
fn the_limiter_dumps_the_first_faults_of_a_boot_only() {
    let limiter = Limiter::new();
    let taken: Vec<_> = (0..REPORTED_FAULTS + 5).map(|_| limiter.take()).collect();
    assert_eq!(taken[..3], [Some(1), Some(2), Some(3)]);
    assert!(taken[3..].iter().all(Option::is_none));
    // Each test's service gets a limiter of its own.
    assert_eq!(Limiter::boot().take(), Some(1));
    assert_eq!(Limiter::boot().take(), Some(1));
}

#[test]
fn the_opening_names_the_unfinished_work_and_its_pipelines() {
    let pipelines = ["render 7".to_string(), "compute 3".to_string()];
    let line = opening(1, &[812, 813, 814], &pipelines);
    assert_eq!(
        line,
        "fault dump 1/3: the device was lost running command buffers 812 to 814 (3), which \
         bound 2 pipelines: render 7, compute 3"
    );
    assert_eq!(opening_fault(&line), Some(1));
    assert_eq!(
        opening(2, &[9], &pipelines[..1]),
        "fault dump 2/3: the device was lost running command buffer 9, which bound 1 pipeline: \
         render 7"
    );
    assert_eq!(
        opening(3, &[9], &[]),
        "fault dump 3/3: the device was lost running command buffer 9, which bound no pipeline"
    );
    assert_eq!(
        opening(1, &[], &[]),
        "fault dump 1/3: the device was lost with no submitted command buffer unfinished"
    );
    let many: Vec<String> = (0..10).map(|id| format!("render {id}")).collect();
    assert!(opening(1, &[1], &many).ends_with("render 9 (the first 8 follow)"));
}

/// Only an opening line names a fault for the platform, and only its own.
#[test]
fn only_an_opening_line_names_its_fault() {
    assert_eq!(
        opening_fault("fault dump 2/3: the device was lost"),
        Some(2)
    );
    assert_eq!(
        opening_fault(&shader(1, "render 7", 0, "f", &vertex())),
        None
    );
    assert_eq!(opening_fault("DeviceLost in operation 9 running ..."), None);
    assert_eq!(opening_fault("fault dump x/3: ..."), None);
}

#[test]
fn a_stage_line_gives_its_code_hash_statistics_and_every_header_word() {
    let line = shader(1, "render 7", 0, "vertex_main+fragment_main", &vertex());
    let header: Vec<String> = (0..32)
        .map(|word| format!("{:08x}", 0x0006_1060 + word))
        .collect();
    assert_eq!(
        line,
        format!(
            "fault dump 1 render 7 variant 0 vertex (vertex_main+fragment_main): code=256 \
             sha256=40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880 \
             instructions=12 gprs=24 slm=0 sph_va=0x300000080 upload=queue sph={}",
            header.join(" ")
        )
    );
    assert_eq!(
        reference_sha256(&vertex().code),
        "40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880"
    );
    // Compute has no header, and a driver may give no statistics.
    let compute = ShaderImage {
        stage: "compute".to_string(),
        code: vec![0xde, 0xad],
        ..ShaderImage::default()
    };
    let line = shader(2, "compute 3", 1, "k", &compute);
    assert_eq!(
        line,
        format!(
            "fault dump 2 compute 3 variant 1 compute (k): code=2 sha256={} sph=none",
            reference_sha256(&[0xde, 0xad])
        )
    );
    let mapped = ShaderImage {
        statistics: vec![("Uploaded by queue".to_string(), 0)],
        geometry_header: vec![1, 2],
        ..vertex()
    };
    let line = shader(1, "render 8", 0, "m", &mapped);
    assert!(line.contains(" upload=mapped sph="));
    assert!(line.ends_with(" gs_sph=00000001 00000002"));
}

#[test]
fn header_bytes_read_as_little_endian_words() {
    assert_eq!(
        words(&[0x62, 0x10, 0x06, 0x00, 1, 0, 0, 0, 0xff]),
        [0x0006_1062, 1]
    );
}

/// The first fault's hex stops at its budget, across stages, and says how
/// much it left out; a later fault's budget gives none.
#[test]
fn hex_lines_stop_at_the_budget_and_count_the_rest() {
    let image = vertex();
    let mut budget = HexBudget::new(HEX_LINE_BYTES * 3 + 10);
    let lines = budget.code(1, "render 7", 0, &image);
    assert_eq!(lines.len(), 4, "three whole lines and part of a fourth");
    assert_eq!(
        lines[1],
        format!(
            "fault dump 1 render 7 variant 0 vertex code+0x0040: {}",
            (64..128u32)
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>()
        )
    );
    assert_eq!(lines[3].split(": ").nth(1).unwrap().len(), 20);
    assert!(budget.code(1, "render 7", 0, &image).is_empty());
    assert_eq!(
        budget.end(1, HEX_LINE_BYTES * 3 + 10).as_deref(),
        Some("fault dump 1 hex: 310 bytes of code past the 202-byte budget not shown")
    );

    let mut none = HexBudget::new(0);
    assert!(none.code(2, "render 7", 0, &image).is_empty());
    assert_eq!(none.end(2, 0), None, "a later fault gives no hex to miss");

    let mut whole = HexBudget::new(HEX_BUDGET);
    assert_eq!(whole.code(1, "render 7", 0, &image).len(), 4);
    assert_eq!(whole.end(1, HEX_BUDGET), None);
}

#[test]
fn every_line_starts_with_the_prefix_the_platform_logs_whole() {
    let image = vertex();
    let lines = [
        opening(1, &[1], &["render 1".to_string()]),
        released(1, "render 1"),
        no_shaders(1, "render 1", 0, "f"),
        more_variants(1, "render 1", 2),
        shader(1, "render 1", 0, "f", &image),
        HexBudget::new(64).code(1, "render 1", 0, &image).remove(0),
    ];
    for line in lines {
        assert!(line.starts_with(PREFIX), "{line}");
        assert!(!line.contains('\n'), "{line}");
    }
}
