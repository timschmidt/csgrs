//! Grid and exact-box queries over every pair of Hyperreal representations.

#![no_main]

use hyperreal::{Rational, Real, StructuralKind};
use hypervoxel::{
    ExactBox, GridFrame, LengthUnit, MaterialRegionId, VoxelAddress, VoxelizationPolicy,
    voxelize_exact_box,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|_data: &[u8]| {
    let values = representative_values();
    for tx in &values {
        for ty in &values {
            let frame = GridFrame::new(
                [tx.clone(), ty.clone(), Real::zero()],
                [Real::one(), Real::one(), Real::one()],
                1,
                LengthUnit::Unitless,
            )
            .expect("positive rational pitches");
            assert_eq!(frame.origin()[0], *tx);
            assert_eq!(frame.origin()[1], *ty);
            assert_eq!(frame.facts().exact_scalars.len, 6);

            let address = VoxelAddress::new(1, [0, 0, 0]).expect("valid address");
            let bounds = address.bounds(&frame).expect("matching depth");
            assert_eq!(bounds.min[0], *tx);
            assert_eq!(bounds.min[1], *ty);

            let exact_box = ExactBox::new(bounds.min.clone(), bounds.max.clone());
            assert!(exact_box.report().exact_box_ready);
            let (_, report) = voxelize_exact_box(
                frame.clone(),
                &exact_box,
                MaterialRegionId(1),
                VoxelizationPolicy::conservative_cover(),
            )
            .expect("translated exact one-cell box");
            assert_eq!(report.predicate_certificates.inside_cells, 1);
            assert_eq!(report.predicate_certificates.unknown_cells, 0);

            let center = bounds.center();
            assert_eq!(
                (center[0].clone() - tx).certified_sign_until(-512).sign(),
                Some(hyperreal::RealSign::Positive)
            );
            assert_eq!(
                (center[1].clone() - ty).certified_sign_until(-512).sign(),
                Some(hyperreal::RealSign::Positive)
            );
        }
    }
});

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
