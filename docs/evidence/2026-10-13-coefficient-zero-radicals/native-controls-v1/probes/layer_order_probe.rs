use super::*;
use hyperdrc::LayerMetadata;
use hyperdrc::checks::duplicate_layer_geometry_readiness;
use hyperdrc::geometry::{LineString, Polygon};
use hyperdrc::kicad::{BoardModel, CopperFeature, CopperKind};

pub(super) fn run() {
    let board = BoardModel {
        source: "layer-order-cost".into(),
        copper: (0..1024)
            .rev()
            .map(|index| CopperFeature {
                layer: format!("L{:02}", index % 32),
                net: None,
                kind: CopperKind::Pad,
                region: polygons_to_profile(
                    vec![rect_polygon([f64::from(index * 3), 0.0], [1.0, 1.0], 0.0)],
                    None,
                ),
                location: [Scalar::from(index * 3), Scalar::zero()],
            })
            .collect(),
        drills: vec![],
        board_outline: None,
        panel_features: None,
    };
    measure_value(
        "board_copper_layers_32x32",
        20,
        || board.copper_layers(&[]).unwrap(),
        |layers| {
            assert_eq!(layers.len(), 32);
            let mut hash = 0xcbf29ce484222325_u64;
            for (index, (name, region)) in layers.iter().enumerate() {
                assert_eq!(name, &format!("L{index:02}"));
                assert_eq!(region.try_to_multipolygon().unwrap().0.len(), 32);
                let geometry = witness(region, &Scalar::from(32));
                for byte in name.bytes().chain(geometry.bytes()) {
                    hash ^= u64::from(byte);
                    hash = hash.wrapping_mul(0x100000001b3);
                }
            }
            format!("32:1024:{hash}")
        },
    );

    let whole = polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)], None);
    let split = polygons_to_profile(
        vec![
            rect_polygon([-0.5, 0.0], [1.0, 2.0], 0.0),
            rect_polygon([0.5, 0.0], [1.0, 2.0], 0.0),
        ],
        None,
    );
    assert!(whole.try_xor(&split).unwrap().try_is_empty().unwrap());
    let forward = [("A".into(), &split), ("Z".into(), &whole)];
    let reverse = [("Z".into(), &whole), ("A".into(), &split)];
    let query = |layers: &[(String, &PcbRegion)]| {
        duplicate_layer_geometry_readiness(layers, &Scalar::zero())
    };
    assert_eq!(
        serde_json::to_vec(&query(&forward)).unwrap(),
        serde_json::to_vec(&query(&reverse)).unwrap()
    );
    for (label, layers) in [
        ("duplicate_layers_forward", &forward),
        ("duplicate_layers_reversed", &reverse),
    ] {
        measure_value(
            label,
            100,
            || query(layers),
            |findings| {
                assert_eq!(findings[0].layers, ["A", "Z"]);
                assert_eq!(findings[0].locations, [[-0.5, 0.0], [0.5, 0.0]]);
                findings_witness(findings, 1, "layer-sanity")
            },
        );
    }
    let failed = |name: &str| {
        polygons_to_profile(
            vec![Polygon::new(LineString(Vec::new()), Vec::new())],
            Some(LayerMetadata { name: name.into() }),
        )
    };
    let a = failed("A");
    let z = failed("Z");
    let forward = [("A".into(), &a), ("Z".into(), &z)];
    let reverse = [("Z".into(), &z), ("A".into(), &a)];
    assert_eq!(
        serde_json::to_vec(&query(&forward)).unwrap(),
        serde_json::to_vec(&query(&reverse)).unwrap()
    );
    measure_value(
        "duplicate_layers_source_failure",
        100,
        || query(&reverse),
        |findings| {
            assert_eq!(findings[0].layers, ["A", "Z"]);
            findings_witness(findings, 1, "geometry-uncertainty")
        },
    );
}
