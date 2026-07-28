//! Packing APIs over every pair of Hyperreal representations.

#![no_main]

use hyperpack::{
    AxisBox3, Bin3, Item3, ItemId, Placement3, analyze_packing_3d, capacity_bounds_3d,
    cuboid_best_fit_decreasing_footprint_area_3d, cuboid_best_fit_decreasing_max_side_3d,
    cuboid_best_fit_decreasing_volume_3d, cuboid_extreme_point_decreasing_volume_3d,
    cuboid_first_fit_decreasing_footprint_area_3d, cuboid_first_fit_decreasing_max_side_3d,
    cuboid_first_fit_decreasing_volume_3d, cuboid_guillotine_best_volume_fit_3d,
    cuboid_laff_largest_area_fit_first_3d, cuboid_maximal_space_decreasing_volume_3d,
    export_no_overlap_model_3d, height_objective_3d, pair_incompatibilities_3d,
    snapshot_packing_3d_binary, snapshot_packing_3d_text, verify_clearance_3d, verify_packing_3d,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

type Heuristic = fn(&Bin3, &[Item3]) -> hyperpack::PackResult<hyperpack::CuboidHeuristicReport3>;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    let heuristics: [Heuristic; 10] = [
        cuboid_first_fit_decreasing_volume_3d,
        cuboid_best_fit_decreasing_volume_3d,
        cuboid_first_fit_decreasing_max_side_3d,
        cuboid_best_fit_decreasing_max_side_3d,
        cuboid_first_fit_decreasing_footprint_area_3d,
        cuboid_best_fit_decreasing_footprint_area_3d,
        cuboid_extreme_point_decreasing_volume_3d,
        cuboid_maximal_space_decreasing_volume_3d,
        cuboid_guillotine_best_volume_fit_3d,
        cuboid_laff_largest_area_fit_first_3d,
    ];

    for (left_index, left) in values.iter().enumerate() {
        for (right_index, right) in values.iter().enumerate() {
            let first_id = item_id("first", left_index, right_index);
            let second_id = item_id("second", left_index, right_index);
            let items = vec![
                Item3 {
                    id: first_id.clone(),
                    size: dimensions(left.clone(), right.clone(), Real::one()),
                },
                Item3 {
                    id: second_id.clone(),
                    size: dimensions(right.clone(), left.clone(), Real::one()),
                },
            ];
            let bin = Bin3 {
                size: dimensions(
                    left + right + Real::one(),
                    left + right + Real::one(),
                    Real::from(2),
                ),
            };
            let placements = vec![
                Placement3 {
                    item: first_id,
                    x: Real::zero(),
                    y: Real::zero(),
                    z: Real::zero(),
                },
                Placement3 {
                    item: second_id,
                    x: left.clone(),
                    y: Real::zero(),
                    z: Real::zero(),
                },
            ];

            let replay = verify_packing_3d(&bin, &items, &placements).expect("valid item ids");
            assert_eq!(replay.objective.placed_items, 2);
            assert_eq!(capacity_bounds_3d(&bin, &items).checked_items, 2);
            assert_eq!(pair_incompatibilities_3d(&bin, &items).checked_pairs, 1);
            assert_eq!(analyze_packing_3d(&bin, &items).scalar_value_count(), 9);
            assert_eq!(export_no_overlap_model_3d(&bin, &items).domains.len(), 2);
            assert_eq!(
                height_objective_3d(&bin, &items, &placements)
                    .expect("known items")
                    .checked_placements,
                2
            );
            assert_eq!(
                verify_clearance_3d(&items, &placements, Real::zero())
                    .expect("known items")
                    .pairs
                    .len(),
                1
            );
            assert!(!snapshot_packing_3d_text(&bin, &items, &placements).is_empty());
            assert!(!snapshot_packing_3d_binary(&bin, &items, &placements).is_empty());

            for heuristic in heuristics {
                let report = heuristic(&bin, &items).expect("valid dimensions and ids");
                assert_eq!(report.trace.considered_items, items.len());
            }
        }
    }
});

fn dimensions(x: Real, y: Real, z: Real) -> AxisBox3 {
    AxisBox3::new(x, y, z).expect("all representatives are positive")
}

fn item_id(prefix: &str, left: usize, right: usize) -> ItemId {
    ItemId::new(format!("{prefix}-{left}-{right}")).expect("nonempty id")
}

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
