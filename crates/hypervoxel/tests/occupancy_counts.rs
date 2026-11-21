use hyperreal::{Rational, Real};
use hypervoxel::{
    AggregateCertainty, GridFrame, MaterialRegionId, SvoVoxelGrid, VoxelAddress,
    VoxelAggregateFacts, VoxelCell, VoxelOccupancyInterval,
};

#[test]
#[cfg(target_pointer_width = "64")]
fn full_unsigned_occupancy_is_exactly_one() {
    for total in [1_usize << 63, usize::MAX] {
        let interval =
            VoxelOccupancyInterval::from_counts(total, total, total, AggregateCertainty::Exact);
        assert_eq!(interval.lower, Real::from(1));
        assert_eq!(interval.upper, Real::from(1));
    }
}

#[test]
#[cfg(target_pointer_width = "64")]
fn high_partial_occupancy_preserves_unsigned_numerators() {
    let total = usize::MAX;
    let definite = 1_usize << 63;
    let possible = total - 1;
    let interval = VoxelOccupancyInterval::from_counts(
        total,
        definite,
        possible,
        AggregateCertainty::Certified,
    );
    assert_eq!(
        &interval.lower * Real::from(total as u64),
        Real::from(definite as u64)
    );
    assert_eq!(
        &interval.upper * Real::from(total as u64),
        Real::from(possible as u64)
    );
}

#[test]
#[cfg(target_pointer_width = "64")]
fn compressed_depth_twenty_one_reports_full_occupancy() {
    let mut grid = SvoVoxelGrid::new(GridFrame::unit(21).unwrap());
    grid.set(
        VoxelAddress::root(),
        VoxelCell::material(MaterialRegionId(7)),
    )
    .unwrap();
    let aggregate = grid.aggregate();
    assert_eq!(aggregate.child_count, 1_usize << 63);
    assert!(aggregate.all_filled);
    assert_eq!(aggregate.occupancy_interval.lower, Real::from(1));
    assert_eq!(aggregate.occupancy_interval.upper, Real::from(1));
}

#[test]
fn small_count_oracle_preserves_metadata_and_certainty() {
    let mut cases = 0;
    for total in 1..=32_usize {
        for definite in 0..=total {
            for possible in definite..=total {
                for certainty in [
                    AggregateCertainty::Exact,
                    AggregateCertainty::Certified,
                    AggregateCertainty::Unknown,
                    AggregateCertainty::Lossy,
                ] {
                    let actual =
                        VoxelOccupancyInterval::from_counts(total, definite, possible, certainty);
                    let lower: Real = Rational::fraction(definite as i64, total as u64)
                        .unwrap()
                        .into();
                    let upper: Real = Rational::fraction(possible as i64, total as u64)
                        .unwrap()
                        .into();
                    assert_eq!(actual.lower, lower);
                    assert_eq!(actual.upper, upper);
                    assert_eq!(actual.total_cells, total);
                    assert_eq!(actual.definite_filled_cells, definite);
                    assert_eq!(actual.possible_occupied_cells, possible);
                    assert_eq!(actual.certainty, certainty);
                    assert_eq!(actual.is_point_interval(), definite == possible);
                    cases += 1;
                }
            }
        }
    }
    assert_eq!(cases, 26176);
}

#[test]
fn unsigned_boundary_ratios_satisfy_exact_count_relations() {
    let mut values = vec![
        0,
        1,
        2,
        usize::MAX / 2,
        usize::MAX / 2 + 1,
        usize::MAX - 1,
        usize::MAX,
    ];
    values.sort_unstable();
    values.dedup();
    for &total in values.iter().filter(|&&n| n != 0) {
        for &definite in values.iter().filter(|&&n| n <= total) {
            for &possible in values.iter().filter(|&&n| definite <= n && n <= total) {
                let actual = VoxelOccupancyInterval::from_counts(
                    total,
                    definite,
                    possible,
                    AggregateCertainty::Certified,
                );
                let divisor = Real::from(total as u64);
                assert_eq!(&actual.lower * &divisor, Real::from(definite as u64));
                assert_eq!(&actual.upper * &divisor, Real::from(possible as u64));
                assert_eq!(actual.certainty, AggregateCertainty::Certified);
                assert_eq!(actual.is_point_interval(), definite == possible);
            }
        }
    }
}

#[test]
fn absent_counts_remain_unknown_unit_interval() {
    for certainty in [
        AggregateCertainty::Exact,
        AggregateCertainty::Certified,
        AggregateCertainty::Unknown,
        AggregateCertainty::Lossy,
    ] {
        let actual = VoxelOccupancyInterval::from_counts(0, 0, 0, certainty);
        assert_eq!(actual.lower, Real::zero());
        assert_eq!(actual.upper, Real::one());
        assert_eq!(actual.certainty, AggregateCertainty::Unknown);
        assert!(!actual.is_point_interval());
    }
}

#[test]
fn compressed_occupancy_survives_parent_composition_and_edits() {
    let depth = ((usize::BITS - 1) / 3).min(21) as u8;
    let total = 1_usize << (3 * u32::from(depth));
    let cell = VoxelCell::material(MaterialRegionId(7));
    let mut grid = SvoVoxelGrid::new(GridFrame::unit(depth).unwrap());
    grid.set(VoxelAddress::root(), cell).unwrap();
    let original = grid.aggregate().clone();
    let parent = VoxelAggregateFacts::from_aggregates([&original]);
    assert_eq!(parent, original);
    assert_eq!(parent.child_count, total);
    assert_eq!(parent.occupancy_interval.lower, Real::one());
    let child = VoxelAddress::root().child(0).unwrap();
    grid.set(child, VoxelCell::empty()).unwrap();
    let partial = grid.aggregate();
    let seven_eighths: Real = Rational::fraction(7, 8).unwrap().into();
    assert_eq!(partial.occupancy_interval.lower, seven_eighths);
    assert_eq!(partial.occupancy_interval.upper, seven_eighths);
    assert_eq!(
        partial.occupancy_interval.definite_filled_cells,
        total / 8 * 7
    );
    grid.set(child, cell).unwrap();
    assert_eq!(grid.aggregate(), &original);
}
