use super::layer::duplicate_layer_geometry_readiness;
use crate::geometry::{polygons_to_profile, rect_polygon};
use crate::kicad::{BoardModel, CopperFeature, CopperKind};
use crate::{LayerMetadata, PcbRegion, Scalar};

fn whole() -> PcbRegion {
    polygons_to_profile(vec![rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)], None)
}
fn split() -> PcbRegion {
    polygons_to_profile(
        vec![
            rect_polygon([-0.5, 0.0], [1.0, 2.0], 0.0),
            rect_polygon([0.5, 0.0], [1.0, 2.0], 0.0),
        ],
        None,
    )
}
fn findings(layers: &[(String, &PcbRegion)]) -> Vec<u8> {
    serde_json::to_vec(&duplicate_layer_geometry_readiness(layers, &Scalar::zero())).unwrap()
}
fn board() -> BoardModel {
    BoardModel {
        source: "layer-order fixture".into(),
        copper: (0..20)
            .rev()
            .map(|index| CopperFeature {
                layer: format!("L{index:02}"),
                net: None,
                kind: CopperKind::Pad,
                region: whole(),
                location: [Scalar::zero(), Scalar::zero()],
            })
            .collect(),
        drills: vec![],
        board_outline: None,
        panel_features: None,
    }
}

#[test]
fn duplicate_pair_order_preserves_complete_serialized_finding() {
    let whole = whole();
    let split = split();
    assert!(whole.try_xor(&split).unwrap().try_is_empty().unwrap());
    let forward = [("A".into(), &split), ("B".into(), &whole)];
    let reversed = [("B".into(), &whole), ("A".into(), &split)];
    assert_eq!(findings(&forward), findings(&reversed));
}

#[test]
fn all_three_layer_permutations_preserve_pair_rows_and_ids() {
    let whole = whole();
    let split = split();
    let layers = [
        ("A".to_string(), &split),
        ("B".to_string(), &whole),
        ("C".to_string(), &split),
    ];
    let expected = findings(&layers);
    for order in [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ] {
        let actual = order.map(|index| layers[index].clone());
        assert_eq!(findings(&actual), expected);
    }
}

#[test]
fn source_failure_choice_and_layer_labels_are_order_independent() {
    let failed = |name: &str| {
        polygons_to_profile(
            vec![
                rect_polygon([0.0, 0.0], [2.0, 2.0], 0.0)
                    .with_exact_construction_error(format!("failed {name}")),
            ],
            Some(LayerMetadata { name: name.into() }),
        )
    };
    let a = failed("A");
    let z = failed("Z");
    assert_eq!(
        findings(&[("A".into(), &a), ("Z".into(), &z)]),
        findings(&[("Z".into(), &z), ("A".into(), &a)])
    );
}

#[test]
fn copper_layer_names_repeat_in_lexical_order() {
    let mut board = board();
    let expected = (0..20)
        .map(|index| format!("L{index:02}"))
        .collect::<Vec<_>>();
    for _ in 0..16 {
        let layers = board.copper_layers(&[]).unwrap();
        assert_eq!(
            layers.iter().map(|(name, _)| name).collect::<Vec<_>>(),
            expected.iter().collect::<Vec<_>>()
        );
        for (_, region) in layers {
            assert_eq!(region.try_area().unwrap(), Scalar::from(4));
        }
        board.copper.reverse();
    }
}

#[test]
fn layer_filter_order_and_duplicates_do_not_change_ordered_projection() {
    let board = board();
    for selection in [
        vec!["L19".into(), "L01".into(), "L10".into()],
        vec![
            "L10".into(),
            "L01".into(),
            "L19".into(),
            "L10".into(),
            "missing".into(),
        ],
    ] {
        for _ in 0..16 {
            let layers = board.copper_layers(&selection).unwrap();
            assert_eq!(
                layers
                    .iter()
                    .map(|(name, _)| name.as_str())
                    .collect::<Vec<_>>(),
                ["L01", "L10", "L19"]
            );
        }
    }
}

#[test]
fn canonical_first_layer_keeps_its_authored_optional_markers() {
    let whole = whole();
    let split = split();
    let rows = duplicate_layer_geometry_readiness(
        &[("A".into(), &split), ("B".into(), &whole)],
        &Scalar::zero(),
    );
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].locations, [[-0.5, 0.0], [0.5, 0.0]]);
    assert_eq!(rows[0].total_area, 4.0);
}

#[test]
fn canonical_order_does_not_create_duplicate_geometry() {
    let whole = whole();
    let other = polygons_to_profile(vec![rect_polygon([10.0, 0.0], [2.0, 2.0], 0.0)], None);
    assert_eq!(
        findings(&[("B".into(), &whole), ("A".into(), &other)]),
        b"[]"
    );
}
