use hyperreal::{Rational, Real};
use hypervoxel::{
    ExactBox, GridFrame, HypervoxelError, LengthUnit, MaterialRegionId, OccupancyState,
    VoxelAddress, VoxelizationPolicy, voxelize_exact_box,
};
use proptest::prelude::*;

fn real_fraction(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

#[test]
fn rejects_depths_that_would_overflow_octree_address_arithmetic() {
    let err = GridFrame::new(
        [0.into(), 0.into(), 0.into()],
        [1.into(), 1.into(), 1.into()],
        22,
        LengthUnit::Micrometer,
    )
    .unwrap_err();

    assert_eq!(
        err,
        HypervoxelError::DepthTooLarge {
            depth: 22,
            max_supported: 21
        }
    );
}

#[test]
fn rejects_addresses_outside_their_depth_cube() {
    assert!(VoxelAddress::new(3, [7, 7, 7]).is_ok());
    assert_eq!(
        VoxelAddress::new(3, [8, 0, 0]),
        Err(HypervoxelError::AddressOverflow)
    );
}

#[test]
fn exact_bounds_survive_large_negative_origin_and_prime_denominator_pitch() {
    let frame = GridFrame::new(
        [(-10_000).into(), 0.into(), 5.into()],
        [
            real_fraction(1, 97),
            real_fraction(2, 97),
            real_fraction(3, 97),
        ],
        6,
        LengthUnit::Unitless,
    )
    .unwrap();

    let bounds = VoxelAddress::new(6, [63, 62, 61])
        .unwrap()
        .bounds(&frame)
        .unwrap();

    assert_eq!(bounds.min[0], real_fraction(-10_000 * 97 + 63, 97));
    assert_eq!(bounds.max[2], real_fraction(5 * 97 + 62 * 3, 97));
}

#[test]
fn integer_aligned_exact_box_uses_half_open_cell_volume() {
    let frame = GridFrame::unit(2).unwrap();
    let exact_box = ExactBox::new(
        [Real::from(1), Real::from(1), Real::from(1)],
        [Real::from(3), Real::from(3), Real::from(3)],
    );

    let (grid, report) = voxelize_exact_box(
        frame,
        &exact_box,
        MaterialRegionId(4),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(grid.len(), 8);
    assert_eq!(report.boundary_cells, 0);
    assert_eq!(report.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.inside_cells, 8);
    assert_eq!(report.predicate_certificates.boundary_cells, 0);
    assert_eq!(report.predicate_certificates.unknown_cells, 0);
    assert_eq!(report.predicate_certificates.outside_cells, 56);
    assert!(report.exact_topology_ready());

    let outside_low = VoxelAddress::new(2, [0, 1, 1]).unwrap();
    let outside_high = VoxelAddress::new(2, [3, 1, 1]).unwrap();
    let inside = VoxelAddress::new(2, [1, 1, 1]).unwrap();
    assert_eq!(
        grid.get(outside_low).unwrap().occupancy,
        OccupancyState::Empty
    );
    assert_eq!(
        grid.get(outside_high).unwrap().occupancy,
        OccupancyState::Empty
    );
    assert_eq!(grid.get(inside).unwrap().occupancy, OccupancyState::Filled);
}

#[test]
fn fractional_exact_box_keeps_conservative_boundary_cells() {
    let frame = GridFrame::unit(2).unwrap();
    let exact_box = ExactBox::new(
        [
            real_fraction(1, 2),
            real_fraction(1, 2),
            real_fraction(1, 2),
        ],
        [
            real_fraction(5, 2),
            real_fraction(5, 2),
            real_fraction(5, 2),
        ],
    );

    let (grid, report) = voxelize_exact_box(
        frame,
        &exact_box,
        MaterialRegionId(5),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();

    assert_eq!(grid.len(), 27);
    assert_eq!(report.predicate_certificates.inside_cells, 1);
    assert_eq!(report.predicate_certificates.boundary_cells, 26);
    assert_eq!(report.boundary_cells, 26);
    assert_eq!(report.unknown_cells, 0);
    assert!(report.exact_topology_ready());
}

proptest! {
    #[test]
    fn generated_addresses_round_trip_through_morton_codes(
        depth in 0_u8..=21,
        x in any::<u64>(),
        y in any::<u64>(),
        z in any::<u64>(),
    ) {
        let mask = (1_u64 << depth) - 1;
        let address = VoxelAddress::new(depth, [x & mask, y & mask, z & mask]).unwrap();
        prop_assert_eq!(
            VoxelAddress::from_morton_code(depth, address.morton_code()).unwrap(),
            address,
        );
    }

    #[test]
    fn generated_addresses_round_trip_through_parent_child(depth in 1_u8..10, x in 0_u64..1024, y in 0_u64..1024, z in 0_u64..1024) {
        let cells = 1_u64 << depth;
        let address = VoxelAddress::new(depth, [x % cells, y % cells, z % cells]).unwrap();
        let parent = address.parent().unwrap();
        prop_assert_eq!(parent.depth, depth - 1);
        prop_assert_eq!(parent.xyz, [address.xyz[0] / 2, address.xyz[1] / 2, address.xyz[2] / 2]);

        let child_index = ((address.xyz[0] & 1) as u8)
            | (((address.xyz[1] & 1) as u8) << 1)
            | (((address.xyz[2] & 1) as u8) << 2);
        prop_assert_eq!(parent.child(child_index).unwrap(), address);
    }

    #[test]
    fn generated_exact_bounds_have_positive_unit_extent(depth in 0_u8..8, x in 0_u64..512, y in 0_u64..512, z in 0_u64..512) {
        let frame = GridFrame::unit(depth).unwrap();
        let cells = 1_u64 << depth;
        let address = VoxelAddress::new(depth, [x % cells, y % cells, z % cells]).unwrap();
        let bounds = address.bounds(&frame).unwrap();

        prop_assert_eq!(bounds.extent(0), Real::from(1));
        prop_assert_eq!(bounds.extent(1), Real::from(1));
        prop_assert_eq!(bounds.extent(2), Real::from(1));
    }
}
