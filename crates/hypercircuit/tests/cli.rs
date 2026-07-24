#![cfg(all(feature = "drc", feature = "interchange"))]

use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::{SystemTime, UNIX_EPOCH};

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, DifferentialPair, DifferentialPairId,
    Net, NetClass, NetClassId, NetId, PcbDesignRules, PcbLayout, PcbRoute, PcbStackup, Real,
    RouteId, SemanticDocument, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn project_fixture() -> SemanticDocument {
    let circuit = Circuit::new(
        CircuitId::new("cli-fixture").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    );
    let pcb = PcbLayout {
        id: BoardId::new("cli-fixture").unwrap(),
        outline: BoardOutline::rectangle(Real::from(20), Real::from(10)),
        stackup: PcbStackup::single_layer(Real::one(), None),
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(pcb)
        .unwrap()
}

fn controlled_impedance_project_fixture() -> SemanticDocument {
    let signal = NetId::new("RF").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("controlled-cli-fixture").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let exact_ratio =
        |numerator, denominator| (Real::from(numerator) / Real::from(denominator)).unwrap();
    let mut pcb = PcbLayout::new(
        BoardId::new("controlled-cli-fixture").unwrap(),
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::two_layer(
            exact_ratio(35, 1_000),
            exact_ratio(18, 100),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    );
    pcb.routes.push(PcbRoute {
        id: RouteId::new("rf-route").unwrap(),
        net: signal.clone(),
        layer: TraceLayer(0),
        width: exact_ratio(32, 100),
        segments: vec![
            LinePathSegment::new(
                Point2::new(Real::one(), Real::from(5)),
                Point2::new(Real::from(19), Real::from(5)),
            )
            .into(),
        ],
    });
    pcb.rules.net_classes.push(NetClass {
        id: NetClassId::new("controlled-rf").unwrap(),
        parent: None,
        nets: vec![signal],
        min_trace_width: None,
        preferred_trace_width: None,
        min_clearance: None,
        preferred_via_land_diameter: None,
        preferred_via_drill_diameter: None,
        preferred_via_style: None,
        max_length: None,
        max_via_count: None,
        target_impedance_ohms: Some(Real::from(50)),
        impedance_tolerance_ohms: Some(Real::from(10)),
        requires_reference_plane: false,
    });
    SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(pcb)
        .unwrap()
}

fn differential_impedance_project_fixture() -> SemanticDocument {
    let positive = NetId::new("USB_D+").unwrap();
    let negative = NetId::new("USB_D-").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("differential-cli-fixture").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: positive.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: negative.clone(),
        is_ground: false,
    });
    let exact_ratio =
        |numerator, denominator| (Real::from(numerator) / Real::from(denominator)).unwrap();
    let mut pcb = PcbLayout::new(
        BoardId::new("differential-cli-fixture").unwrap(),
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::two_layer(
            exact_ratio(35, 1_000),
            exact_ratio(18, 100),
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    );
    for (id, net, y) in [
        ("usb-positive", positive.clone(), exact_ratio(4, 1)),
        ("usb-negative", negative.clone(), exact_ratio(45, 10)),
    ] {
        pcb.routes.push(PcbRoute {
            id: RouteId::new(id).unwrap(),
            net,
            layer: TraceLayer(0),
            width: exact_ratio(32, 100),
            segments: vec![
                LinePathSegment::new(
                    Point2::new(Real::one(), y.clone()),
                    Point2::new(Real::from(19), y),
                )
                .into(),
            ],
        });
    }
    pcb.rules.differential_pairs.push(DifferentialPair {
        id: DifferentialPairId::new("usb-data").unwrap(),
        positive,
        negative,
        spacing: exact_ratio(18, 100),
        max_skew: Some(exact_ratio(1, 10)),
        target_impedance_ohms: Some(Real::from(90)),
        impedance_tolerance_ohms: Some(Real::from(5)),
        neckdown: None,
    });
    SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(pcb)
        .unwrap()
}

fn temporary_directory() -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!("hypercircuit-cli-{}-{nonce}", std::process::id()))
}

fn invoke_in(directory: &Path, arguments: &[&Path]) -> Output {
    let mut command = Command::new(env!("CARGO_BIN_EXE_hypercircuit"));
    command.current_dir(directory);
    for argument in arguments {
        command.arg(argument);
    }
    command.output().unwrap()
}

fn assert_success(output: &Output) {
    assert!(
        output.status.success(),
        "status {:?}\nstdout:\n{}\nstderr:\n{}",
        output.status.code(),
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn retained_project_cli_checks_exports_and_releases_one_document() {
    let directory = temporary_directory();
    fs::create_dir_all(&directory).unwrap();
    let document = directory.join("design.json");
    fs::write(
        &document,
        project_fixture().to_json_pretty().unwrap().as_bytes(),
    )
    .unwrap();
    fs::write(
        directory.join("hypercircuit.toml"),
        r#"schema = "org.hypercircuit.project"
version = 1

[project]
name = "cli-project"
version = "0.1.0"
default-design = "main"

[designs.main]
provider = "command"
command = ["cat", "design.json"]
"#,
    )
    .unwrap();
    let nested = directory.join("nested");
    fs::create_dir(&nested).unwrap();

    let check = invoke_in(&directory, &[Path::new("check"), &document]);
    assert_success(&check);
    assert!(String::from_utf8_lossy(&check.stdout).contains("0 release blocker(s)"));

    let project_check = invoke_in(&nested, &[Path::new("check")]);
    assert_success(&project_check);
    assert!(String::from_utf8_lossy(&project_check.stdout).contains("0 release blocker(s)"));

    let provider_ir = directory.join("provider-ir.json");
    assert_success(&invoke_in(
        &directory,
        &[
            Path::new("ir"),
            Path::new("main"),
            Path::new("--out"),
            &provider_ir,
        ],
    ));
    assert_eq!(
        SemanticDocument::from_json(&fs::read_to_string(&provider_ir).unwrap()).unwrap(),
        project_fixture()
    );

    let snapshot = directory.join("snapshot.json");
    assert_success(&invoke_in(
        &directory,
        &[Path::new("snapshot"), Path::new("--out"), &snapshot],
    ));
    assert_eq!(
        fs::read(&provider_ir).unwrap(),
        fs::read(&snapshot).unwrap()
    );

    let bom = directory.join("provider-bom.csv");
    assert_success(&invoke_in(
        &directory,
        &[Path::new("bom"), Path::new("--out"), &bom],
    ));
    assert_eq!(
        fs::read_to_string(&bom).unwrap(),
        "quantity,references,part,model,land_pattern\n"
    );

    let board = directory.join("review.kicad_pcb");
    let kicad = invoke_in(
        &directory,
        &[Path::new("export-kicad"), Path::new("main"), &board],
    );
    assert_success(&kicad);
    assert!(fs::read_to_string(&board).unwrap().contains("(kicad_pcb"));

    let preview = directory.join("review.svg");
    assert_success(&invoke_in(
        &directory,
        &[Path::new("export-svg"), Path::new("main"), &preview],
    ));
    assert!(fs::read_to_string(&preview).unwrap().contains("<svg"));

    let release_directory = directory.join("release");
    let release = invoke_in(
        &directory,
        &[Path::new("release"), Path::new("main"), &release_directory],
    );
    assert_success(&release);
    assert!(release_directory.join("bom.csv").is_file());
    assert!(release_directory.join("pick-and-place.csv").is_file());
    assert!(release_directory.join("dnp.csv").is_file());
    assert!(
        fs::read_dir(&release_directory)
            .unwrap()
            .filter_map(Result::ok)
            .any(|entry| entry
                .file_name()
                .to_string_lossy()
                .ends_with("-manifest.json"))
    );

    fs::remove_dir_all(directory).unwrap();
}

#[test]
fn project_cli_uses_manifest_material_evidence_for_controlled_impedance_release() {
    let directory = temporary_directory();
    fs::create_dir_all(&directory).unwrap();
    let document = directory.join("controlled.json");
    fs::write(
        &document,
        controlled_impedance_project_fixture()
            .to_json_pretty()
            .unwrap()
            .as_bytes(),
    )
    .unwrap();
    fs::write(
        directory.join("hypercircuit.toml"),
        r#"schema = "org.hypercircuit.project"
version = 1

[project]
name = "controlled-project"
version = "0.1.0"
default-design = "main"

[designs.main]
provider = "command"
command = ["cat", "controlled.json"]

[[pcb-materials]]
handle = "hyperphysics:FR4"
relative-permittivity = "21/5"
loss-tangent = "0.018"
source-authority = "laminate-manufacturer"
source-locator = "datasheet/revision-a"
source-freshness = "2026-06"
condition = "1 MHz nominal"
"#,
    )
    .unwrap();

    let direct_check = invoke_in(&directory, &[Path::new("check"), &document]);
    assert_eq!(direct_check.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&direct_check.stdout).contains("DrcHandoffOmissions(2)"));

    let project_check = invoke_in(&directory, &[Path::new("check")]);
    assert_success(&project_check);
    assert!(String::from_utf8_lossy(&project_check.stdout).contains("0 release blocker(s)"));

    let release_directory = directory.join("controlled-release");
    let release = invoke_in(
        &directory,
        &[Path::new("release"), Path::new("main"), &release_directory],
    );
    assert_success(&release);
    assert!(String::from_utf8_lossy(&release.stdout).contains("0 release blocker(s)"));
    assert!(release_directory.join("bom.csv").is_file());

    let mut mismatched = controlled_impedance_project_fixture();
    mismatched.pcb.as_mut().unwrap().routes[0].width = (Real::from(8) / Real::from(100)).unwrap();
    fs::write(&document, mismatched.to_json_pretty().unwrap().as_bytes()).unwrap();
    let mismatch_check = invoke_in(&directory, &[Path::new("check")]);
    assert_eq!(mismatch_check.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&mismatch_check.stdout).contains("DrcErrors(1)"));

    fs::write(
        &document,
        differential_impedance_project_fixture()
            .to_json_pretty()
            .unwrap()
            .as_bytes(),
    )
    .unwrap();
    let direct_differential_check = invoke_in(&directory, &[Path::new("check"), &document]);
    assert_eq!(direct_differential_check.status.code(), Some(2));
    let direct_differential_stdout = String::from_utf8_lossy(&direct_differential_check.stdout);
    assert!(
        direct_differential_stdout.contains("DrcHandoffOmissions(2)"),
        "{direct_differential_stdout}"
    );
    let project_differential_check = invoke_in(&directory, &[Path::new("check")]);
    assert_success(&project_differential_check);
    assert!(
        String::from_utf8_lossy(&project_differential_check.stdout)
            .contains("0 release blocker(s)")
    );
    let differential_release_directory = directory.join("differential-release");
    let differential_release = invoke_in(
        &directory,
        &[
            Path::new("release"),
            Path::new("main"),
            &differential_release_directory,
        ],
    );
    assert_success(&differential_release);
    assert!(String::from_utf8_lossy(&differential_release.stdout).contains("0 release blocker(s)"));

    let mut mismatched = differential_impedance_project_fixture();
    mismatched.pcb.as_mut().unwrap().rules.differential_pairs[0].target_impedance_ohms =
        Some(Real::from(110));
    mismatched.pcb.as_mut().unwrap().rules.differential_pairs[0].impedance_tolerance_ohms =
        Some(Real::from(2));
    fs::write(&document, mismatched.to_json_pretty().unwrap().as_bytes()).unwrap();
    let mismatch_check = invoke_in(&directory, &[Path::new("check")]);
    assert_eq!(mismatch_check.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&mismatch_check.stdout).contains("DrcErrors(1)"));

    fs::remove_dir_all(directory).unwrap();
}
