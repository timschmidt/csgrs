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
    Connector,
    Decoupling,
    RfFence,
    Plating,
    MountGround,
    MountPlating,
}

const RULES: [Rule; 6] = [
    Rule::Connector,
    Rule::Decoupling,
    Rule::RfFence,
    Rule::Plating,
    Rule::MountGround,
    Rule::MountPlating,
];

fn copper(net: &str, x: Scalar, kind: CopperKind) -> CopperFeature {
    CopperFeature {
        layer: "F.Cu".into(),
        net: Some(net.into()),
        kind,
        location: [x, Scalar::zero()],
        // Public authored centers are independent of the shape. Only the
        // connector edge-selection rule needs this finite edge-touching carrier.
        region: polygons_to_profile(vec![rect_polygon([0.95, 0.0], [0.1, 0.1], 0.0)], None),
    }
}

impl Rule {
    fn board(self, center: Scalar, neighbor: Scalar) -> BoardModel {
        let mut features = vec![self.neighbor(neighbor)];
        let mut drills = Vec::new();
        match self {
            Self::Connector => features.insert(0, copper("USB_DP", center, CopperKind::Pad)),
            Self::Decoupling => features.insert(0, copper("VCC", center, CopperKind::Pad)),
            Self::RfFence => features.insert(0, copper("RF", center, CopperKind::Segment)),
            _ => drills.push(DrillFeature {
                location: [center, Scalar::zero()],
                diameter: 2.into(),
                net: Some("SIG".into()),
                plated: !matches!(self, Self::MountGround),
            }),
        }
        BoardModel {
            source: "authored center fixture".into(),
            copper: features,
            drills,
            board_outline: Some(polygons_to_profile(
                vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)],
                None,
            )),
            panel_features: None,
        }
    }

    fn neighbor(self, x: Scalar) -> CopperFeature {
        copper(
            if matches!(self, Self::Plating) {
                "SIG"
            } else {
                "GND"
            },
            x,
            if matches!(self, Self::RfFence) {
                CopperKind::Via
            } else {
                CopperKind::Pad
            },
        )
    }

    fn boundary(self) -> Scalar {
        if matches!(self, Self::MountGround | Self::MountPlating) {
            2.into()
        } else {
            1.into()
        }
    }

    #[allow(dead_code)]
    fn check(self) -> &'static str {
        match self {
            Self::Connector => "connector-return-path-readiness",
            Self::Decoupling => "decoupling-proximity-readiness",
            Self::RfFence => "rf-via-fence-readiness",
            Self::Plating => "plating-intent",
            Self::MountGround => "mounting-hole-grounding-readiness",
            Self::MountPlating => "mounting-hole-plating-intent-readiness",
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
            Self::Connector => {
                connector_return_path_readiness(board, layers, &scalar("0.1"), radius)
            },
            Self::Decoupling => decoupling_proximity_readiness(board, layers, radius),
            Self::RfFence => rf_via_fence_readiness(board, layers, radius),
            Self::Plating => plating_intent(board, layers, radius),
            Self::MountGround => mounting_hole_grounding_readiness(board, layers, radius),
            Self::MountPlating => {
                mounting_hole_plating_intent_readiness(board, layers, radius)
            },
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
    query: impl FnMut() -> Vec<Violation>,
) {
    measure_value(label, iterations, query, |findings| {
        findings_witness(findings, count, check)
    });
}
pub(super) fn run() {
    let one = Scalar::one();
    let huge_radius = scalar("1e400");
    for rule in RULES {
        let label = format!("authored_{rule:?}").to_lowercase();
        let huge = scalar("1e400");
        let board = rule.board(huge.clone(), huge + Scalar::one());
        measure(&format!("{label}_huge_centers"), 50, 0, "unused", || {
            rule.run(&board, &one)
        });
        let mut board = rule.board(0.into(), 1.into());
        if matches!(rule, Rule::MountGround | Rule::MountPlating) {
            board.drills[0].diameter = huge_radius.clone();
        }
        measure(&format!("{label}_huge_radius"), 50, 0, "unused", || {
            rule.run(&board, &huge_radius)
        });
        let board = rule.board(0.into(), rule.boundary());
        let radius = Scalar::one() + unknown_zero();
        measure(
            &format!("{label}_unknown_radius"),
            25,
            1,
            "geometry-uncertainty",
            || rule.run(&board, &radius),
        );
        let mut sparse = rule.board(0.into(), 1.into());
        let count = i32::try_from(2_002 - sparse.copper.len() - sparse.drills.len()).unwrap();
        sparse
            .copper
            .extend((0..count).map(|i| rule.neighbor(Scalar::from(10 + 2 * i))));
        assert_eq!(sparse.copper.len() + sparse.drills.len(), 2_002);
        measure(&format!("{label}_sparse_2002"), 10, 0, "unused", || {
            rule.run(&sparse, &one)
        });
    }
    let mut board = Rule::Plating.board(0.into(), 2.into());
    board.drills[0].plated = false;
    board.copper[0].kind = CopperKind::Segment;
    measure(
        "authored_npth_closed_radius",
        100,
        1,
        "plating-intent",
        || Rule::Plating.run(&board, &one),
    );
    let mut board = Rule::MountPlating.board(scalar("1e400"), scalar("-1e400"));
    board.drills[0].net = Some("GND".into());
    board.drills[0].diameter = Scalar::one() + unknown_zero();
    let zero = unknown_zero();
    measure("authored_plated_ground_metadata", 100, 0, "unused", || {
        Rule::MountPlating.run(&board, &zero)
    });
    let board = Rule::MountGround.board(0.into(), scalar("0.5"));
    let radius = Scalar::one() + unknown_zero();
    measure(
        "authored_mount_unordered_thresholds",
        100,
        0,
        "unused",
        || Rule::MountGround.run(&board, &radius),
    );
    let mut board = Rule::Connector.board(0.into(), 0.into());
    board.board_outline = Some(board.copper[0].region.clone());
    measure(
        "authored_connector_unknown_edge",
        25,
        1,
        "geometry-uncertainty",
        || connector_return_path_readiness(&board, &[], &zero, &one),
    );
    let board = Rule::Decoupling.board(0.into(), 0.into());
    let negative = Scalar::from(-3);
    measure(
        "authored_negative_radius",
        100,
        1,
        "decoupling-proximity-readiness",
        || Rule::Decoupling.run(&board, &negative),
    );
}
