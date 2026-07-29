use csgrs::{
    curve,
    solid::{self, SolidExt},
};
use hyperlattice::Real;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

#[test]
fn many_small_hyperreal_transforms_remain_composable() {
    let base = solid::center(&solid::cube(r(0.25)));
    let combined = (0..8)
        .map(|i| base.translated(r(i as f64) * r(0.5), r(0.0), r(0.0)))
        .reduce(|left, right| left.try_union(&right).expect("certified union"))
        .expect("nonempty input");

    assert!(!combined.triangles.is_empty());
}

#[test]
fn native_curve_catalog_subset_extrudes_with_hyperreal_scalars() {
    let curves = [
        curve::circle(r(0.5), 24),
        curve::rectangle(r(0.75), r(0.4)),
        curve::star(5, r(0.5), r(0.2)),
    ];

    for region in curves {
        assert!(!curve::extrude(&region, r(0.2)).triangles.is_empty());
    }
}
