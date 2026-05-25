use hyperreal::{Rational, Real};
use hypervoxel::{
    ContinuousFieldVoxelCell, ContinuousFieldVoxelInterchangeManifest,
    ContinuousFieldVoxelManifest, ContinuousFieldVoxelRowOrder, FreshnessStatus,
    GridCoordinateSystem, GridFrame, GridSource, HypervoxelError, LengthUnit, VoxelAddress,
    VoxelCell, continuous_field_address,
};
use proptest::prelude::*;

fn real_fraction(n: i64, d: u64) -> Real {
    Rational::fraction(n, d).unwrap().into()
}

#[test]
fn rejects_depths_that_would_overflow_octree_address_arithmetic() {
    let err = GridFrame::builder()
        .depth(22)
        .units(LengthUnit::Micrometer)
        .build()
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
    let frame = GridFrame::builder()
        .origin([(-10_000).into(), 0.into(), 5.into()])
        .pitch([
            real_fraction(1, 97),
            real_fraction(2, 97),
            real_fraction(3, 97),
        ])
        .depth(6)
        .build()
        .unwrap();

    let bounds = VoxelAddress::new(6, [63, 62, 61])
        .unwrap()
        .bounds(&frame)
        .unwrap();

    assert_eq!(bounds.min[0], real_fraction(-10_000 * 97 + 63, 97));
    assert_eq!(bounds.max[2], real_fraction(5 * 97 + 62 * 3, 97));
}

proptest! {
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
        let frame = GridFrame::builder().depth(depth).build().unwrap();
        let cells = 1_u64 << depth;
        let address = VoxelAddress::new(depth, [x % cells, y % cells, z % cells]).unwrap();
        let bounds = address.bounds(&frame).unwrap();

        prop_assert_eq!(bounds.extent(0), Real::from(1));
        prop_assert_eq!(bounds.extent(1), Real::from(1));
        prop_assert_eq!(bounds.extent(2), Real::from(1));
    }

    #[test]
    fn generated_continuous_field_intake_readiness_tracks_exact_rows(depth in 1_u8..5, n in 1_usize..32) {
        let frame = GridFrame::builder()
            .depth(depth)
            .source(GridSource::new("sdf:generated", 1))
            .build()
            .unwrap();
        let cells_per_axis = 1_u64 << depth;
        let rows = (0..n)
            .map(|i| {
                let i = i as u64;
                let address = continuous_field_address(
                    &frame,
                    [i % cells_per_axis, (i / cells_per_axis) % cells_per_axis, 0],
                )
                .unwrap();
                ContinuousFieldVoxelCell::new(address, VoxelCell::material(hypervoxel::MaterialRegionId(1)))
            })
            .collect::<Vec<_>>();
        let manifest = ContinuousFieldVoxelManifest {
            frame: frame.clone(),
            source: frame.source().cloned(),
            expected_source: frame.source().cloned(),
            expected_cell_count: rows.len(),
            cells: rows,
        };
        let report = manifest.report();

        prop_assert_eq!(report.freshness, FreshnessStatus::Current);
        prop_assert!(report.finest_depth_only);
        prop_assert!(report.exact_cell_evidence_ready);
        prop_assert_eq!(
            report.exact_materialization_ready,
            report.duplicate_address_count == 0
                && n == (cells_per_axis * cells_per_axis * cells_per_axis) as usize
        );

        let cells_per_axis = 1_u64 << depth;
        let interchange = ContinuousFieldVoxelInterchangeManifest {
            source: frame.source().cloned(),
            expected_source: frame.source().cloned(),
            coordinate_system: GridCoordinateSystem::HyperGrid,
            row_order: ContinuousFieldVoxelRowOrder::ExplicitAddresses,
            declared_depth: depth,
            declared_dimensions: [cells_per_axis, cells_per_axis, cells_per_axis],
            declared_cell_count: n,
        };
        let interchange_report = manifest.interchange_report(&interchange);
        prop_assert_eq!(
            interchange_report.exact_interchange_ready,
            n == (cells_per_axis * cells_per_axis * cells_per_axis) as usize
        );
    }
}
