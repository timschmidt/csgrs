use super::*;
use hyperdrc::checks::{apply_ipc356_nets, drills_to_region, ipc356_coverage};
use hyperdrc::ipc356::Ipc356Point;
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};

fn scalar(value: &str) -> Scalar {
    value.parse().unwrap()
}
fn drill(x: Scalar, diameter: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, Scalar::zero()],
        diameter,
        plated: true,
        net: None,
    }
}
fn copper(x: Scalar) -> CopperFeature {
    let source = drill(x, 2.into());
    CopperFeature {
        layer: "F.Cu".into(),
        net: None,
        kind: CopperKind::Pad,
        region: drills_to_region(&[source.clone()], "cost copper", "cost-construction")
            .unwrap(),
        location: source.location,
    }
}
fn board(copper: Vec<CopperFeature>, drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "native IPC cost".into(),
        copper,
        drills,
        board_outline: None,
        panel_features: None,
    }
}
fn point(x: Scalar, diameter: Option<Scalar>) -> Ipc356Point {
    Ipc356Point {
        net: "IPC".into(),
        reference: Some("TP1".into()),
        pin: None,
        location: [x, 0.into()],
        diameter,
        access_side: None,
        feature_type: None,
        soldermask: None,
    }
}
fn metadata(board: &BoardModel) -> Vec<u8> {
    let copper = board
        .copper
        .iter()
        .map(|copper| &copper.net)
        .collect::<Vec<_>>();
    let drills = board
        .drills
        .iter()
        .map(|drill| (&drill.net, drill.diameter.to_string()))
        .collect::<Vec<_>>();
    serde_json::to_vec(&(copper, drills)).unwrap()
}
fn annotation(
    label: &str,
    iterations: usize,
    source: &BoardModel,
    points: &[Ipc356Point],
    tolerance: &Scalar,
    expected_metadata: &[u8],
    expected_error: bool,
) {
    let original = metadata(source);
    measure_value(
        label,
        iterations,
        || {
            // Include a fresh metadata/board clone in every complete annotation timer.
            let mut board = source.clone();
            let result = apply_ipc356_nets(&mut board, points, tolerance);
            (board, result)
        },
        |(board, result)| {
            assert_eq!(result.is_err(), expected_error);
            let output = metadata(board);
            assert_eq!(output, expected_metadata);
            let error = match result {
                Ok(()) => "ok".to_string(),
                Err(error) => {
                    format!("{}:{:?}:{}", error.operation, error.source, error.detail)
                },
            };
            let mut hash = 0xcbf29ce484222325_u64;
            for byte in output.iter().chain(error.as_bytes()) {
                hash ^= u64::from(*byte);
                hash = hash.wrapping_mul(0x100000001b3);
            }
            format!(
                "{}:{}:{}:{hash}",
                board.copper.len(),
                board.drills.len(),
                output.len() + error.len()
            )
        },
    );
    assert_eq!(metadata(source), original);
}
fn coverage(
    label: &str,
    iterations: usize,
    source: &BoardModel,
    points: &[Ipc356Point],
    tolerance: &Scalar,
    count: usize,
    check: &str,
) {
    measure_value(
        label,
        iterations,
        || ipc356_coverage(source, points, tolerance),
        |findings| findings_witness(findings, count, check),
    );
}

pub(super) fn run() {
    let one = Scalar::one();
    let source = board(vec![copper(0.into())], vec![drill(0.into(), 0.into())]);
    let mut expected = source.clone();
    expected.copper[0].net = Some("IPC".into());
    expected.drills[0].net = Some("IPC".into());
    expected.drills[0].diameter = scalar("0.4");
    let expected_small = metadata(&expected);
    let points = [point(0.into(), Some(scalar("0.4")))];
    annotation(
        "ipc_annotation_small",
        100,
        &source,
        &points,
        &one,
        &expected_small,
        false,
    );
    annotation(
        "ipc_annotation_huge_tolerance",
        100,
        &source,
        &points,
        &scalar("1e400"),
        &expected_small,
        false,
    );
    let huge = board(
        vec![copper(scalar("1e400"))],
        vec![drill(scalar("1e400"), 0.into())],
    );
    annotation(
        "ipc_annotation_huge_centers",
        100,
        &huge,
        &[point(scalar("1e400"), Some(scalar("0.4")))],
        &one,
        &expected_small,
        false,
    );

    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let unknown = &sine * &sine + &cosine * &cosine - Scalar::one();
    let uncertain = board(
        vec![copper(0.into()), copper(10.into())],
        vec![drill(0.into(), 0.into())],
    );
    let points = [
        point(0.into(), Some(scalar("0.4"))),
        point(Scalar::from(11) + &unknown, None),
    ];
    annotation(
        "ipc_annotation_rollback",
        100,
        &uncertain,
        &points,
        &one,
        &metadata(&uncertain),
        true,
    );
    annotation(
        "ipc_annotation_invalid_diameter",
        100,
        &source,
        &[point(0.into(), Some((-1).into()))],
        &one,
        &metadata(&source),
        true,
    );

    let mut sparse = source.clone();
    for index in 1..2002 {
        let x = Scalar::from(index * 10);
        sparse.copper.push(copper(x.clone()));
        sparse.drills.push(drill(x, scalar("0.2")));
    }
    let mut expected = sparse.clone();
    expected.copper[0].net = Some("IPC".into());
    expected.drills[0].net = Some("IPC".into());
    expected.drills[0].diameter = scalar("0.4");
    annotation(
        "ipc_annotation_sparse_2002",
        20,
        &sparse,
        &[point(0.into(), Some(scalar("0.4")))],
        &one,
        &metadata(&expected),
        false,
    );
    coverage(
        "ipc_coverage_boundary",
        100,
        &source,
        &[
            point(1.into(), None),
            point(scalar("1.0000000000000001"), None),
        ],
        &one,
        1,
        "ipc356-coverage",
    );
    coverage(
        "ipc_coverage_unknown",
        100,
        &source,
        &[point(Scalar::one() + &unknown, None)],
        &one,
        1,
        "geometry-uncertainty",
    );
    coverage(
        "ipc_coverage_huge_centers",
        100,
        &huge,
        &[point(scalar("1e400"), None)],
        &one,
        0,
        "ipc356-coverage",
    );
    let alternatives = board(
        vec![copper(Scalar::one() + unknown), copper(0.into())],
        vec![],
    );
    coverage(
        "ipc_coverage_known_after_unknown",
        100,
        &alternatives,
        &[point(0.into(), None)],
        &one,
        0,
        "ipc356-coverage",
    );
    coverage(
        "ipc_coverage_sparse_2002",
        20,
        &sparse,
        &[point(0.into(), None)],
        &one,
        0,
        "ipc356-coverage",
    );
}
