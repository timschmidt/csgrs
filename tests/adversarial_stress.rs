//! Fixed-size composition smoke witnesses for separated boxes and three sampled profiles.
//! These nonemptiness assertions do not measure resource growth or certify analytic fidelity.

use csgrs::{
    curve,
    solid::{self, SolidExt},
};
use hyperlattice::Real;

/// Converts a finite binary64 fixture value exactly and panics on NaN or infinity.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

#[test]
/// Left-folds union over eight separated translated quarter-unit cubes and requires triangles.
/// Exercises repeated result consumption without an independent component-count or set oracle.
fn many_small_hyperreal_transforms_remain_composable() {
    let base = solid::center(&solid::cube(r(0.25)).expect("valid primitive fixture"))
        .expect("valid modeling fixture");
    let combined = (0..8)
        .map(|i| {
            base.translated(r(i as f64) * r(0.5), r(0.0), r(0.0))
                .expect("valid transform fixture")
        })
        .reduce(|left, right| left.try_union(&right).expect("certified union"))
        .expect("nonempty input");

    assert!(!combined.triangles.is_empty());
}

#[test]
/// Requires nonempty strict extrusions of a 24-chord circle, rectangle and sampled five-point star.
/// Extracts each local outcome without inspecting its certainty or construction evidence.
fn native_curve_catalog_subset_extrudes_with_hyperreal_scalars() {
    let curves = [
        curve::circle(r(0.5), 24).expect("valid planar fixture"),
        curve::rectangle(r(0.75), r(0.4)).expect("valid planar fixture"),
        curve::star(5, r(0.5), r(0.2)).expect("valid sampled profile fixture"),
    ];

    for region in curves {
        assert!(
            !curve::try_extrude(&region, r(0.2), csgrs::GeometryContext::STRICT)
                .expect("extrude")
                .into_value()
                .triangles
                .is_empty()
        );
    }
}
