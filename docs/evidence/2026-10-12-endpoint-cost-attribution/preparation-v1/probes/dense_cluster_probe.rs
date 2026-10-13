use super::*;
use hyperdrc::Violation;
use hyperdrc::checks::*;
use hyperdrc::geometry::{polygons_to_profile, rect_polygon};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind};
fn scalar(text: &str) -> Scalar {
    text.parse().unwrap()
}

#[derive(Clone, Copy, Debug)]
enum Rule {
    Fiducial,
    Escape,
    ViaSpacing,
    MaskBridge,
}
const RULES: [Rule; 4] = [
    Rule::Fiducial,
    Rule::Escape,
    Rule::ViaSpacing,
    Rule::MaskBridge,
];

fn rectangle(x: f64, width: f64) -> hyperdrc::PcbRegion {
    polygons_to_profile(vec![rect_polygon([x, 0.0], [width, width], 0.0)], None)
}
fn feature(kind: CopperKind, x: Scalar, shape_x: f64, width: f64) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some("PAD".into()),
        kind,
        location: [x, Scalar::zero()],
        region: rectangle(shape_x, width),
    }
}
fn fiducial(x: Scalar) -> CopperFeature {
    let mut feature = feature(CopperKind::Pad, x, 0.0, 0.8);
    feature.net = None;
    feature
}
impl Rule {
    fn board(self, center: Scalar) -> BoardModel {
        let mut copper = (0..16)
            .map(|index| {
                feature(
                    CopperKind::Pad,
                    &center + Scalar::from(index) - scalar("7.5"),
                    f64::from(index) * 2.0,
                    0.2,
                )
            })
            .collect::<Vec<_>>();
        match self {
            Self::Fiducial => {
                copper.push(fiducial(&center - scalar("0.5")));
                copper.push(fiducial(&center + scalar("0.5")));
            },
            Self::Escape | Self::ViaSpacing => {
                copper.push(feature(CopperKind::Via, center + scalar("0.5"), 100.0, 0.2))
            },
            Self::MaskBridge => {},
        }
        BoardModel {
            source: "native dense cluster".into(),
            copper,
            drills: vec![],
            board_outline: None,
            panel_features: None,
        }
    }
    fn bad_board(self, center: Scalar) -> BoardModel {
        let mut board = self.board(center);
        match self {
            Self::Fiducial | Self::Escape => board.copper.truncate(16),
            Self::ViaSpacing => board.copper[16].region = board.copper[0].region.clone(),
            Self::MaskBridge => board.copper[1].region = board.copper[0].region.clone(),
        }
        board
    }
    fn run(self, board: &BoardModel, pitch: &Scalar, radius: &Scalar) -> Vec<Violation> {
        match self {
            Self::Fiducial => local_fiducial_readiness(board, &[], pitch, radius),
            Self::Escape => dense_pad_escape_readiness(board, &[], pitch, radius),
            Self::ViaSpacing => dense_pad_via_spacing_readiness(
                board,
                &[],
                pitch,
                radius,
                &scalar("0.1"),
                &Scalar::zero(),
            ),
            Self::MaskBridge => {
                dense_pad_mask_bridge_readiness(board, &[], pitch, &scalar("0.1"))
            },
        }
    }
    fn check(self) -> &'static str {
        match self {
            Self::Fiducial => "local-fiducial-readiness",
            Self::Escape => "dense-pad-escape-readiness",
            Self::ViaSpacing => "dense-pad-via-spacing-readiness",
            Self::MaskBridge => "dense-pad-mask-bridge-readiness",
        }
    }
}
fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
}

fn native_rectangle(width: Scalar, height: Scalar) -> hyperdrc::PcbRegion {
    native_rectangle_at(Scalar::zero(), width, height)
}

fn native_rectangle_at(x: Scalar, width: Scalar, height: Scalar) -> hyperdrc::PcbRegion {
    use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
    let points = [
        Point2::new(x.clone(), Scalar::zero()),
        Point2::new(&x + &width, Scalar::zero()),
        Point2::new(&x + width, height.clone()),
        Point2::new(x, height),
    ];
    let edges = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| Segment2::Line(LineSeg2::try_new(a, b).unwrap()))
        .collect();
    hyperdrc::PcbRegion::new(
        CurveRegion2::try_from_native_contours(
            vec![Contour2::try_new(edges).unwrap()],
            vec![],
        )
        .unwrap(),
        None,
    )
}

fn large_rectangle() -> PcbRegion {
    // Retain the original hard source at the same scale; native corners now survive.
    let region = rectangle(1e20, 16_384.0);
    assert!(region.try_bounding_box().unwrap().is_some());
    assert_eq!(region.try_area().unwrap(), Scalar::from(268_435_456));
    region
}

fn measure(
    label: &str,
    iterations: usize,
    count: usize,
    check: &str,
    query: impl FnMut() -> Vec<Violation>,
) {
    measure_value(label, iterations, query, |findings| {
        findings_witness(findings, count, check)
    });
}

pub(super) fn run() {
    let one = Scalar::one();
    let huge = scalar("1e400");
    for rule in RULES {
        let label = format!("dense_{rule:?}").to_lowercase();
        let board = rule.board(huge.clone());
        measure(&format!("{label}_huge_centers"), 25, 0, "unused", || {
            rule.run(&board, &one, &one)
        });
        let board = rule.board(Scalar::zero());
        measure(&format!("{label}_huge_pitch"), 25, 0, "unused", || {
            rule.run(&board, &huge, &one)
        });
        let board = rule.bad_board(Scalar::zero());
        let pitch = &one + unknown_zero();
        measure(
            &format!("{label}_unknown_pitch"),
            10,
            1,
            "geometry-uncertainty",
            || rule.run(&board, &pitch, &one),
        );
        let mut sparse = rule.board(Scalar::zero());
        sparse.copper = (0..2_000)
            .map(|index| {
                feature(
                    CopperKind::Pad,
                    Scalar::from(index) - scalar("999.5"),
                    f64::from(index) * 2.0,
                    0.2,
                )
            })
            .collect();
        match rule {
            Rule::Fiducial => sparse
                .copper
                .extend([fiducial(scalar("-0.5")), fiducial(scalar("0.5"))]),
            Rule::Escape | Rule::ViaSpacing => sparse.copper.extend([
                feature(CopperKind::Via, scalar("-0.5"), -100.0, 0.2),
                feature(CopperKind::Via, scalar("0.5"), -100.0, 0.2),
            ]),
            Rule::MaskBridge => sparse.copper.extend([
                feature(CopperKind::Artwork, scalar("-0.5"), -100.0, 0.2),
                feature(CopperKind::Artwork, scalar("0.5"), -100.0, 0.2),
            ]),
        }
        assert_eq!(sparse.copper.len(), 2_002);
        measure(&format!("{label}_sparse_2002"), 5, 0, "unused", || {
            rule.run(&sparse, &one, &one)
        });
    }
    for rule in [Rule::Fiducial, Rule::Escape, Rule::ViaSpacing] {
        let board = rule.board(Scalar::zero());
        let label = format!("dense_{rule:?}_huge_search_radius").to_lowercase();
        measure(&label, 25, 0, "unused", || rule.run(&board, &one, &huge));
    }
    let mut board = Rule::Fiducial.board(Scalar::zero());
    for feature in &mut board.copper[16..] {
        feature.region = native_rectangle(scalar("0.5") - scalar("1e-50"), scalar("0.5"));
    }
    measure(
        "dense_fid_dimension_below_half",
        25,
        1,
        Rule::Fiducial.check(),
        || Rule::Fiducial.run(&board, &one, &one),
    );
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[17].location[0] = one.clone();
    let radius = &one + unknown_zero();
    measure(
        "dense_fid_unknown_radius",
        10,
        1,
        "geometry-uncertainty",
        || Rule::Fiducial.run(&board, &one, &radius),
    );
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].location[0] = &one + unknown_zero();
    board.copper.push(fiducial(scalar("-0.5")));
    measure("dense_fid_count_after_unknown", 10, 0, "unused", || {
        Rule::Fiducial.run(&board, &one, &one)
    });
    let mut board = Rule::Escape.bad_board(Scalar::zero());
    board.copper[0].location[0] += unknown_zero();
    measure(
        "dense_pitch_optional_order",
        10,
        1,
        Rule::Escape.check(),
        || Rule::Escape.run(&board, &Scalar::from(2), &one),
    );
    for rule in [Rule::ViaSpacing, Rule::MaskBridge] {
        let mut board = rule.bad_board(Scalar::zero());
        board.copper[2].region = board.copper[1].region.clone();
        if matches!(rule, Rule::ViaSpacing) {
            board.copper[1].region = board.copper[16].region.clone();
        }
        board.copper[0].region = large_rectangle();
        let label = format!("dense_{rule:?}_optional_minimum").to_lowercase();
        measure(&label, 10, 1, rule.check(), || rule.run(&board, &one, &one));
        let mut board = rule.board(Scalar::zero());
        board.copper[0].region = large_rectangle();
        let label = format!("dense_{rule:?}_required_large_shape").to_lowercase();
        measure(&label, 10, 0, "unused", || rule.run(&board, &one, &one));
    }
}
