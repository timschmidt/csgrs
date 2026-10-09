use super::*;
use hyperdrc::ipc356::Ipc356Point;
use hyperdrc::kicad::{BoardModel, DrillFeature};

fn scalar(value: &str) -> Scalar {
    value.parse().unwrap()
}
fn drill(x: Scalar, y: Scalar, diameter: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, y],
        diameter,
        plated: true,
        net: None,
    }
}
fn point(x: Scalar, diameter: Scalar) -> Ipc356Point {
    Ipc356Point {
        net: "test".into(),
        reference: None,
        pin: None,
        location: [x, 0.into()],
        diameter: Some(diameter),
        access_side: None,
        feature_type: None,
        soldermask: None,
    }
}
fn board(drills: Vec<DrillFeature>) -> BoardModel {
    BoardModel {
        source: "native-drill-cost".into(),
        copper: vec![],
        drills,
        board_outline: Some(polygons_to_profile(
            vec![rect_polygon([10.0, 10.0], [20.0, 20.0], 0.0)],
            None,
        )),
        panel_features: None,
    }
}
fn measure(
    label: &str,
    iterations: usize,
    count: usize,
    check: &str,
    query: impl FnMut() -> Vec<hyperdrc::Violation>,
) {
    measure_value(label, iterations, query, |findings| {
        findings_witness(findings, count, check)
    });
}

pub(super) fn run() {
    use hyperdrc::checks::{
        castellation_pitch_readiness, drill_spacing, drill_table_consistency,
        ipc356_drill_diameter, mounting_hole_spacing_readiness,
    };
    let pair = [
        drill(0.into(), 0.into(), 1.into()),
        drill(2.into(), 0.into(), 1.into()),
    ];
    let one = Scalar::one();
    let tenth = scalar("0.1");
    measure("drill_spacing_boundary", 100, 0, "drill-spacing", || {
        drill_spacing(&pair, &[], &one)
    });
    let huge = [
        drill(0.into(), 0.into(), scalar("1e400")),
        drill(10.into(), 0.into(), 1.into()),
    ];
    measure("drill_spacing_huge_diameter", 100, 1, "drill-spacing", || {
        drill_spacing(&huge, &[], &tenth)
    });
    let unprojectable = [
        drill(scalar("1e400"), 0.into(), 1.into()),
        drill(scalar("1e400"), 0.into(), 1.into()),
    ];
    measure(
        "drill_spacing_unprojectable_marker",
        100,
        1,
        "geometry-uncertainty",
        || drill_spacing(&unprojectable, &[], &tenth),
    );
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let unknown = &sine * &sine + &cosine * &cosine - Scalar::one();
    let threshold = Scalar::one() + &unknown;
    measure(
        "drill_spacing_unknown_threshold",
        100,
        1,
        "geometry-uncertainty",
        || drill_spacing(&pair, &[], &threshold),
    );
    let mut sparse = (0..2_000)
        .map(|index| drill((10 + 2 * index).into(), 10.into(), scalar("0.3")))
        .collect::<Vec<_>>();
    sparse.extend([
        drill(0.into(), 0.into(), scalar("0.4")),
        drill(scalar("0.55"), 0.into(), scalar("0.4")),
    ]);
    let clearance = scalar("0.2");
    measure("drill_spacing_sparse_2002", 20, 1, "drill-spacing", || {
        drill_spacing(&sparse, &[], &clearance)
    });
    let extra = [drill(0.into(), 0.into(), 1.into())];
    let ipc = [point(scalar("1e400"), 3.into()), point(0.into(), 2.into())];
    let isolated =
        serde_json::to_value(drill_table_consistency(&[], &extra, &ipc[1..], &tenth)).unwrap();
    assert_eq!(
        serde_json::to_value(drill_table_consistency(&[], &extra, &ipc, &tenth)).unwrap(),
        isolated
    );
    measure(
        "drill_table_preserved_ipc_index",
        100,
        1,
        "drill-table-consistency",
        || drill_table_consistency(&[], &extra, &ipc, &tenth),
    );
    let unresolved = [drill(0.into(), 0.into(), Scalar::from(2) + unknown)];
    measure(
        "drill_table_unknown_diameter",
        100,
        1,
        "geometry-uncertainty",
        || drill_table_consistency(&extra, &unresolved, &[], &one),
    );
    let plated = board(extra.to_vec());
    measure("ipc_drill_diameter", 100, 1, "ipc356-drill-diameter", || {
        ipc356_drill_diameter(&plated, &ipc[1..], &tenth)
    });
    let hardware = board(vec![
        drill(0.into(), 0.into(), 3.into()),
        drill(0.into(), scalar("3.25"), 3.into()),
    ]);
    let half = scalar("0.5");
    measure(
        "mounting_hole_spacing",
        100,
        1,
        "mounting-hole-spacing-readiness",
        || mounting_hole_spacing_readiness(&hardware, &half),
    );
    measure(
        "castellation_rect_spacing",
        100,
        1,
        "castellation-pitch-readiness",
        || castellation_pitch_readiness(&hardware, &half),
    );
    let mut circular = board(vec![
        drill(9.into(), 0.into(), 1.into()),
        drill(9.into(), 1.into(), 1.into()),
    ]);
    let spacing = scalar("0.6");
    let mut previous = None;
    for samples in [17, 128] {
        circular.board_outline = Some(polygons_to_profile(
            vec![circle_polygon([0.0, 0.0], 10.0, samples)],
            None,
        ));
        let findings = castellation_pitch_readiness(&circular, &spacing);
        let current = serde_json::to_value(&findings).unwrap();
        if let Some(previous) = previous {
            assert_eq!(previous, current);
        }
        previous = Some(current);
        measure(
            &format!("castellation_circle_{samples}"),
            100,
            1,
            "castellation-pitch-readiness",
            || castellation_pitch_readiness(&circular, &spacing),
        );
    }
}
