#![no_main]

mod support;

use hypercurve::{
    Curve2, CurveContext, HomogeneousControl2, NurbsCurve2, Point2, PolynomialSplineCurve2, Real,
    SplinePeriodicity2,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(r(x as i32 - 128), r(y as i32 - 128))
}

/// Exercises the unified exact Bezier decomposition of one spline curve.
fn touch_native_fragments(curve: Curve2, policy: &CurveContext) {
    if let Ok(fragments) = support::under(policy, || curve.native_bezier_fragments()) {
        for fragment in fragments {
            let _ = fragment.parameter_range();
            let _ = fragment.curve();
        }
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let policy = CurveContext::STRICT;
    let degree = if data[0] & 1 == 0 { 2 } else { 3 };
    let control_count = degree + 2;
    let mut controls = Vec::new();
    for chunk in data[1..].chunks(2).take(control_count) {
        if chunk.len() < 2 {
            return;
        }
        controls.push(point(chunk[0], chunk[1]));
    }

    let mut knots = vec![Real::zero(); degree + 1];
    knots.push(Real::one());
    knots.extend(std::iter::repeat_n(Real::from(2_i8), degree + 1));
    if let Ok(spline) =
        PolynomialSplineCurve2::try_new(degree, controls.clone(), knots.clone(), &policy)
    {
        touch_native_fragments(Curve2::from(spline.into_value()), &policy);
    }
    let weights = controls
        .iter()
        .enumerate()
        .map(|(index, _)| Real::from(((data[index % data.len()] % 7) as i32) + 1))
        .collect::<Vec<_>>();
    let authored = NurbsCurve2::try_new(degree, controls.clone(), weights, knots.clone(), &policy);
    let homogeneous = NurbsCurve2::from_homogeneous_controls(
        degree,
        controls
            .iter()
            .enumerate()
            .map(|(index, point)| {
                let weight = if index == 0 || index + 1 == controls.len() {
                    Real::one()
                } else {
                    r(i32::from(data[index] % 7) - 3)
                };
                HomogeneousControl2::new(point.x().clone(), point.y().clone(), weight)
            })
            .collect(),
        knots,
        SplinePeriodicity2::NonPeriodic,
        &policy,
    );
    for construction in [authored, homogeneous] {
        let Ok(spline) = construction else {
            continue;
        };
        let spline = spline.into_value();
        touch_native_fragments(Curve2::from(spline.clone()), &policy);
        if let Ok(refined) = spline.insert_knot(Real::one(), &policy) {
            let _ = refined.into_value().remove_knot(Real::one(), &policy);
        }
    }
});
