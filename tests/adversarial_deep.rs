use csgrs::{
    curve::{self, CurveRegionExt},
    solid::{self, SolidExt},
};
use hypercurve::CurvePolicy;
use hyperlattice::Real;
use hyperlimit::PredicatePolicy;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("test values must be finite")
}

#[test]
fn repeated_boolean_and_transform_sequence_stays_nonempty() {
    let base = solid::center(&solid::cube(r(1.0)));
    let mut acc = base.clone();

    for i in 0..3 {
        let shift = r(i as f64) * r(0.18);
        let part = solid::rotate(
            &base.translated(shift, r(0.0), r(0.0)),
            r(0.0),
            r(90.0) * r(i as f64),
            r(0.0),
        );
        acc = acc.try_union(&part).expect("certified union");
    }

    assert!(!acc.triangles.is_empty());
    let bounds = solid::bounding_box(&acc);
    assert_eq!(
        hyperlimit::compare_reals(&bounds.maxs.x, &bounds.mins.x, PredicatePolicy::STRICT)
            .value(),
        Some(std::cmp::Ordering::Greater)
    );
}

#[test]
fn native_curve_boolean_outputs_can_be_extruded() {
    let left = curve::rectangle(r(2.0), r(1.0));
    let right = curve::transformed(
        &curve::circle(r(0.75), 32),
        &hyperlattice::Matrix4::affine_translation([r(1.0), r(0.5), r(0.0)]),
    );
    let region = left
        .try_union(&right, &CurvePolicy::STRICT)
        .expect("curve union")
        .into_value();
    let mesh = curve::try_extrude(&region, r(0.4), &csgrs::GeometryContext::STRICT)
        .expect("extrude")
        .into_value();

    assert!(!mesh.triangles.is_empty());
}
