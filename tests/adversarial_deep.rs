use csgrs::{csg::CSG, mesh::Mesh, sketch::Profile};
use hyperlattice::Real;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

#[test]
fn repeated_boolean_and_transform_sequence_stays_nonempty() {
    let base = Mesh::<()>::cube(r(1.0), ()).center();
    let mut acc = base.clone();

    for i in 0..3 {
        let shift = r(i as f64) * r(0.18);
        let part = base.clone().translate(shift.clone(), r(0.0), r(0.0)).rotate(
            r(0.0),
            r(90.0) * r(i as f64),
            r(0.0),
        );
        acc = acc.union(&part);
    }

    assert!(!acc.polygons.is_empty());
    assert!(acc.bounding_box().maxs.x > acc.bounding_box().mins.x);
}

#[test]
fn curve_boolean_outputs_can_be_extruded() {
    let left = Profile::rectangle(r(2.0), r(1.0));
    let right = Profile::circle(r(0.75), 32).translate(r(1.0), r(0.5), r(0.0));
    let sketch = left.union(&right);
    let mesh = sketch.extrude(r(0.4), ());

    assert!(!mesh.polygons.is_empty());
}
