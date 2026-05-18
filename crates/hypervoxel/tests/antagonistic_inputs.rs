use hyperreal::{Rational, Real};
use hypervoxel::{GridFrame, HypervoxelError, LengthUnit, VoxelAddress};
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
}
