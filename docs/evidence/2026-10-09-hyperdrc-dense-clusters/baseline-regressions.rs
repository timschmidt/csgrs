use super::*;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::scalar::scalar;

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

fn rectangle(x: f64, width: f64) -> crate::PcbRegion {
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
            }
            Self::Escape | Self::ViaSpacing => {
                copper.push(feature(CopperKind::Via, center + scalar("0.5"), 100.0, 0.2))
            }
            Self::MaskBridge => {}
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
            Self::MaskBridge => dense_pad_mask_bridge_readiness(board, &[], pitch, &scalar("0.1")),
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
fn uncertain(findings: &[Violation]) {
    assert_eq!(findings.len(), 1, "{findings:?}");
    assert_eq!(findings[0].check, "geometry-uncertainty", "{findings:?}");
}
macro_rules! cluster {
    ($name:ident, $rule:ident) => {
        mod $name {
            use super::*;
            #[test]
            fn huge_centers_need_no_projection_for_a_passing_check() {
                let board = Rule::$rule.board(scalar("1e400"));
                assert!(
                    Rule::$rule
                        .run(&board, &Scalar::one(), &Scalar::one())
                        .is_empty()
                );
            }
            #[test]
            fn unknown_pitch_cannot_become_a_sparse_cluster() {
                let board = Rule::$rule.bad_board(Scalar::zero());
                uncertain(&Rule::$rule.run(
                    &board,
                    &(Scalar::one() + unknown_zero()),
                    &Scalar::one(),
                ));
            }
            #[test]
            fn required_large_report_marker_is_typed() {
                let board = Rule::$rule.bad_board(scalar("1e400"));
                uncertain(&Rule::$rule.run(&board, &Scalar::one(), &Scalar::one()));
            }
        }
    };
}
cluster!(fiducials, Fiducial);
cluster!(escape, Escape);
cluster!(via_spacing, ViaSpacing);
cluster!(mask_bridge, MaskBridge);

#[test]
fn finite_passes_and_findings_remain_distinct() {
    for rule in RULES {
        assert!(
            rule.run(&rule.board(Scalar::zero()), &Scalar::one(), &Scalar::one())
                .is_empty(),
            "{rule:?}"
        );
        let findings = rule.run(
            &rule.bad_board(Scalar::zero()),
            &Scalar::one(),
            &Scalar::one(),
        );
        assert_eq!(findings.len(), 1, "{rule:?}: {findings:?}");
        assert_eq!(findings[0].check, rule.check());
    }
}
#[test]
fn unknown_fiducial_radius_cannot_become_an_exact_count_of_one() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[17].location[0] = Scalar::one();
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}
#[test]
fn unknown_escape_radius_cannot_become_absence() {
    let mut board = Rule::Escape.board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one();
    uncertain(&Rule::Escape.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}
#[test]
fn unknown_via_radius_cannot_suppress_close_copper() {
    let mut board = Rule::ViaSpacing.bad_board(Scalar::zero());
    board.copper[16].location[0] = Scalar::one();
    uncertain(&Rule::ViaSpacing.run(&board, &Scalar::one(), &(Scalar::one() + unknown_zero())));
}

fn native_rectangle(width: Scalar, height: Scalar) -> crate::PcbRegion {
    use hypercurve::{Contour2, CurveRegion2, LineSeg2, Point2, Segment2};
    let points = [
        Point2::from_values(0, 0),
        Point2::new(width.clone(), Scalar::zero()),
        Point2::new(width, height.clone()),
        Point2::new(Scalar::zero(), height),
    ];
    let edges = points
        .iter()
        .cloned()
        .zip(points.iter().cloned().cycle().skip(1))
        .map(|(a, b)| Segment2::Line(LineSeg2::try_new(a, b).unwrap()))
        .collect();
    crate::PcbRegion::new(
        CurveRegion2::try_from_native_contours(vec![Contour2::try_new(edges).unwrap()], vec![])
            .unwrap(),
        None,
    )
}
#[test]
fn fiducial_minimum_dimension_is_native_below_a_float_ulp() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    for feature in &mut board.copper[16..] {
        feature.region = native_rectangle(scalar("0.5") - scalar("1e-50"), scalar("0.5"));
    }
    let findings = Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one());
    assert_eq!(findings.len(), 1);
    assert_eq!(findings[0].check, Rule::Fiducial.check());
}
#[test]
fn unresolved_fiducial_dimension_is_not_selected_or_dropped() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = native_rectangle(scalar("0.5") + unknown_zero(), scalar("0.5"));
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn failed_fiducial_source_cannot_become_an_ordinary_pad() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = board.copper[16]
        .region
        .clone()
        .with_exact_construction_error("retained fiducial source failure");
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn unknown_native_fiducial_bounds_are_required_evidence() {
    let mut board = Rule::Fiducial.board(Scalar::zero());
    board.copper[16].region = polygons_to_profile(
        vec![crate::bounds_regressions::uncertain_bounds_polygon()],
        None,
    );
    uncertain(&Rule::Fiducial.run(&board, &Scalar::one(), &Scalar::one()));
}
#[test]
fn huge_search_radius_is_supported() {
    for rule in RULES {
        assert!(
            rule.run(
                &rule.board(Scalar::zero()),
                &Scalar::one(),
                &scalar("1e400")
            )
            .is_empty(),
            "{rule:?}"
        );
    }
}
