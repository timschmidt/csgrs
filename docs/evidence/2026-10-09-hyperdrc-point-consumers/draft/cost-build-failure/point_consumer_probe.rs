use super::*;
use hyperdrc::Violation;
use hyperdrc::checks::*;
use hyperdrc::geometry::{polygons_to_profile, rect_polygon};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind, DrillFeature};
fn scalar(text: &str) -> Scalar {
    text.parse().unwrap()
}

#[derive(Clone, Copy, Debug)]
enum Rule {
    Edge,
    Chassis,
    Return,
    Power,
    PairProximity,
    PairReturn,
    Mouse,
}

const RULES: [Rule; 7] = [
    Rule::Edge,
    Rule::Chassis,
    Rule::Return,
    Rule::Power,
    Rule::PairProximity,
    Rule::PairReturn,
    Rule::Mouse,
];

fn copper(net: &str, x: Scalar) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some(net.into()),
        kind: CopperKind::Via,
        // Public anchors are independent of the carrier. Keep the edge-band
        // fixture finite while exercising native anchors at their own boundary.
        region: polygons_to_profile(vec![rect_polygon([0.95, 0.0], [0.1, 0.1], 0.0)], None),
        location: [x, Scalar::zero()],
    }
}

fn drill(x: Scalar) -> DrillFeature {
    DrillFeature {
        location: [x, Scalar::zero()],
        diameter: scalar("0.4"),
        plated: false,
        net: None,
    }
}

impl Rule {
    fn board(self, center: Scalar, neighbor: Scalar) -> BoardModel {
        let drills = if matches!(self, Self::Mouse) {
            vec![drill(center.clone()), drill(neighbor.clone())]
        } else {
            vec![]
        };
        let copper = match self {
            Self::Chassis => vec![copper("SHIELD", center), copper("GND", neighbor)],
            Self::Power => vec![copper("VCC", center), copper("VCC", neighbor)],
            Self::PairProximity => vec![copper("USB_DP", center), copper("USB_DM", neighbor)],
            Self::PairReturn => vec![
                copper("USB_DP", center.clone()),
                copper("USB_DM", center),
                copper("GND", neighbor),
            ],
            Self::Mouse => vec![],
            _ => vec![
                copper("USB_DP", center.clone()),
                copper("GND", neighbor.clone()),
            ],
        };
        BoardModel {
            source: "native point consumer cost".into(),
            copper,
            drills,
            board_outline: Some(polygons_to_profile(
                vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)],
                None,
            )),
            panel_features: None,
        }
    }

    fn run(self, board: &BoardModel, radius: &Scalar) -> Vec<Violation> {
        self.run_selected(board, radius, &[])
    }

    fn run_selected(
        self,
        board: &BoardModel,
        radius: &Scalar,
        layers: &[String],
    ) -> Vec<Violation> {
        match self {
            Self::Edge => edge_stitching_readiness(
                board,
                layers,
                &scalar("0.2"),
                radius,
                &Scalar::zero(),
            ),
            Self::Chassis => chassis_stitching_readiness(board, layers, radius),
            Self::Return => return_path_readiness(board, radius, layers),
            Self::Power => power_via_array_readiness(board, layers, radius),
            Self::PairProximity => {
                differential_pair_via_proximity_readiness(board, layers, radius)
            },
            Self::PairReturn => differential_pair_via_return_readiness(board, layers, radius),
            Self::Mouse => mouse_bite_readiness(
                board,
                &[],
                &scalar("0.2"),
                &scalar("0.5"),
                &Scalar::zero(),
                radius,
            ),
        }
    }
}

fn unknown_zero() -> Scalar {
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    &sine * &sine + &cosine * &cosine - Scalar::one()
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
    let two = Scalar::from(2);
    let huge_radius = scalar("1e400");
    let half = scalar("0.5");
    let minimum_diameter = scalar("0.2");
    let minimum_spacing = scalar("1.5");
    let one = Scalar::one();
    for rule in RULES {
        let label = format!("point_{rule:?}").to_lowercase();
        let huge = scalar("1e400");
        let board = rule.board(huge.clone(), huge + Scalar::one());
        measure(&format!("{label}_huge_centers"), 50, 0, "unused", || {
            rule.run(&board, &two)
        });
        let board = rule.board(0.into(), 1.into());
        measure(&format!("{label}_huge_radius"), 50, 0, "unused", || {
            rule.run(&board, &huge_radius)
        });
        let radius = Scalar::one() + unknown_zero();
        measure(
            &format!("{label}_unknown_radius"),
            25,
            1,
            "geometry-uncertainty",
            || rule.run(&board, &radius),
        );

        let mut sparse = rule.board(0.into(), 1.into());
        match rule {
            Rule::Mouse => {
                sparse
                    .drills
                    .extend((0..2_000).map(|i| drill((10 + 2 * i).into())));
                assert_eq!(sparse.drills.len(), 2_002);
            },
            Rule::PairProximity => {
                for i in 0..1_000 {
                    sparse.copper.push(copper("USB_DP", (10 + 6 * i).into()));
                    sparse.copper.push(copper("USB_DM", (11 + 6 * i).into()));
                }
                assert_eq!(sparse.copper.len(), 2_002);
            },
            _ => {
                let net = if matches!(rule, Rule::Power) {
                    "VCC"
                } else {
                    "GND"
                };
                let count = 2_002 - sparse.copper.len();
                sparse
                    .copper
                    .extend((0..count).map(|i| copper(net, Scalar::from(10 + 2 * i))));
                assert_eq!(sparse.copper.len(), 2_002);
            },
        }
        measure(&format!("{label}_sparse_2002"), 10, 0, "unused", || {
            rule.run(&sparse, &two)
        });
    }

    let mut board = Rule::Mouse.board(0.into(), 1.into());
    board.drills.push(drill(Scalar::one() + unknown_zero()));
    measure(
        "point_mouse_optional_order",
        25,
        3,
        "mouse-bite-readiness",
        || mouse_bite_readiness(&board, &[], &minimum_diameter, &half, &minimum_spacing, &two),
    );
    let mut board = Rule::Power.board(0.into(), 0.into());
    board.copper[1].layer = "B.Cu".into();
    measure(
        "point_power_layer_copies",
        100,
        1,
        "power-via-array-readiness",
        || Rule::Power.run(&board, &one),
    );
    for (rule, check) in [
        (Rule::Edge, "edge-stitching-readiness"),
        (Rule::Chassis, "chassis-stitching-readiness"),
    ] {
        let center = scalar("1e20");
        let board = rule.board(center.clone(), center + Scalar::one());
        measure(
            &format!("point_{rule:?}_sub_ulp_outside").to_lowercase(),
            100,
            1,
            check,
            || rule.run(&board, &half),
        );
    }
    for (label, value, expected) in [
        (
            "unknown_distance",
            Scalar::hypot2(&unknown_zero(), &Scalar::zero()).unwrap(),
            "0.000000",
        ),
        ("pi", Scalar::pi(), "3.141593"),
        ("tiny_pi", Scalar::pi() * scalar("1e-100"), "0.000000"),
    ] {
        measure_value(
            &format!("fixed_decimal_{label}"),
            1_000,
            || format!("{:.6}", value.fixed_decimal()),
            |text| {
                assert_eq!(text, expected);
                text.clone()
            },
        );
    }
}
