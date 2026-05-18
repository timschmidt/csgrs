use hyperpack::{
    AxisBox3, Bin3, FeasibilityReplay3, FeasibilityStatus, FreeSpaceReport3, HeuristicFamily,
    Item3, ItemId, LowerBoundReport, PackError, PackingReport3, Placement3, Real,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn item(id: &str, x: i32, y: i32, z: i32) -> Item3 {
    Item3 {
        id: ItemId::new(id).unwrap(),
        size: AxisBox3::new(r(x), r(y), r(z)).unwrap(),
    }
}

fn placement(id: &str, x: i32, y: i32, z: i32) -> Placement3 {
    Placement3 {
        item: ItemId::new(id).unwrap(),
        x: r(x),
        y: r(y),
        z: r(z),
    }
}

#[test]
fn exact_replay_accepts_contained_non_overlapping_boxes() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("a", 5, 5, 5), item("b", 5, 5, 5)];
    let placements = [placement("a", 0, 0, 0), placement("b", 5, 0, 0)];

    let replay = FeasibilityReplay3::replay(&bin, &items, &placements).unwrap();

    assert_eq!(replay.status, FeasibilityStatus::Feasible);
    assert_eq!(replay.containment_checks, 2);
    assert_eq!(replay.no_overlap_checks, 1);
}

#[test]
fn exact_replay_rejects_overlap_and_outside_bin() {
    let bin = Bin3 {
        size: AxisBox3::new(r(10), r(10), r(10)).unwrap(),
    };
    let items = [item("a", 6, 6, 6), item("b", 6, 6, 6)];

    let overlap = FeasibilityReplay3::replay(
        &bin,
        &items,
        &[placement("a", 0, 0, 0), placement("b", 4, 0, 0)],
    )
    .unwrap();
    assert_eq!(overlap.status, FeasibilityStatus::Infeasible);
    assert!(overlap.facts[0].contains("overlaps"));

    let outside = FeasibilityReplay3::replay(&bin, &items, &[placement("a", 5, 5, 5)]).unwrap();
    assert_eq!(outside.status, FeasibilityStatus::Infeasible);
    assert!(outside.facts[0].contains("outside"));
}

#[test]
fn proposal_reports_keep_heuristic_lower_bound_and_free_space_evidence() {
    let report = PackingReport3 {
        heuristic: HeuristicFamily::MaxRects2,
        seed: 42,
        placements: vec![placement("a", 0, 0, 0)],
        free_space: FreeSpaceReport3 {
            boxes: vec![AxisBox3::new(r(5), r(10), r(1)).unwrap()],
            exact: true,
        },
        lower_bound: Some(LowerBoundReport {
            lower_bound: r(1),
            incumbent: Some(r(1)),
            method: "area".into(),
        }),
    };
    assert_eq!(report.seed, 42);
    assert_eq!(report.free_space.boxes.len(), 1);
    assert_eq!(report.lower_bound.unwrap().method, "area");
}

#[test]
fn invalid_dimensions_and_missing_items_are_rejected() {
    assert_eq!(
        AxisBox3::new(r(0), r(1), r(1)).unwrap_err(),
        PackError::NonPositiveDimension
    );
    let bin = Bin3 {
        size: AxisBox3::new(r(1), r(1), r(1)).unwrap(),
    };
    assert_eq!(
        FeasibilityReplay3::replay(&bin, &[], &[placement("missing", 0, 0, 0)]).unwrap_err(),
        PackError::MissingItem
    );
}

proptest! {
    #[test]
    fn empty_item_ids_are_rejected(id in "\\PC*") {
        if id.is_empty() {
            prop_assert!(ItemId::new(id).is_err());
        } else {
            prop_assert!(ItemId::new(id).is_ok());
        }
    }
}
