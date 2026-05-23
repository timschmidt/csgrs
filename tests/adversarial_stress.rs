use csgrs::{csg::CSG, mesh::Mesh, sketch::Profile};
use hyperlattice::Real;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

#[test]
fn many_small_hyperreal_transforms_remain_composable() {
    let base = Mesh::<()>::cube(r(0.25), ()).center();
    let combined = (0..8)
        .map(|i| base.clone().translate(r(i as f64) * r(0.2), r(0.0), r(0.0)))
        .reduce(|a, b| a.union(&b))
        .expect("nonempty input");

    assert!(!combined.polygons.is_empty());
}

#[test]
fn sketch_catalog_subset_extrudes_with_hyperreal_scalars() {
    let sketches = [
        Profile::<()>::circle(r(0.5), 24, ()),
        Profile::<()>::rectangle(r(0.75), r(0.4), ()),
        Profile::<()>::star(5, r(0.5), r(0.2), ()),
    ];

    for sketch in sketches {
        assert!(!sketch.extrude(r(0.2)).polygons.is_empty());
    }
}
