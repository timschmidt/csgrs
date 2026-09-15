#![no_main]

use hypercurve::{
    Classification, CurveContext, HomogeneousControl2, Point2, PolynomialBSplineCurve2,
    RationalBSplineCurve2, Real, RetainedBSplineSpanFactEvidence2,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(r(x as i32 - 128), r(y as i32 - 128))
}

fn touch_span_fact_evidence(evidence: &RetainedBSplineSpanFactEvidence2) {
    for span in evidence.span_facts() {
        let _ = span.span_index();
        let _ = span.knot_interval();
        let _ = span.bounds();
        let _ = span.x_monotonicity();
        let _ = span.y_monotonicity();
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
    if let Ok(Classification::Decided(spline)) =
        PolynomialBSplineCurve2::try_new(degree, controls.clone(), knots.clone(), &policy)
    {
        let _ = spline.extract_bezier_spans(&policy).map(|classification| {
            let Classification::Decided(extraction) = classification else {
                return;
            };
            let _ = extraction
                .span_fact_evidence(&policy)
                .map(|classification| {
                    let Classification::Decided(evidence) = classification else {
                        return;
                    };
                    touch_span_fact_evidence(&evidence);
                });
        });
    }
    let weights = controls
        .iter()
        .enumerate()
        .map(|(index, _)| Real::from(((data[index % data.len()] % 7) as i32) + 1))
        .collect::<Vec<_>>();
    let authored =
        RationalBSplineCurve2::try_new(degree, controls.clone(), weights, knots.clone(), &policy);
    let homogeneous = RationalBSplineCurve2::from_homogeneous_controls(
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
        &policy,
    );
    for construction in [authored, homogeneous] {
        let Ok(Classification::Decided(spline)) = construction else {
            continue;
        };
        if let Ok(Classification::Decided(extraction)) = spline.extract_bezier_spans(&policy) {
            if let Ok(Classification::Decided(facts)) = extraction.span_fact_evidence(&policy) {
                touch_span_fact_evidence(&facts);
            }
            for span in extraction.spans() {
                let _ = span.knot_interval();
                let _ = span.curve().homogeneous_controls();
                let _ = span
                    .curve()
                    .point_at(&((Real::one() / r(2)).unwrap()), &policy);
            }
            let _ = extraction.native_subcurves(&policy);
        }
        if let Ok(curve) = hypercurve::NurbsCurve2::from_homogeneous_controls(
            degree,
            spline.homogeneous_controls().to_vec(),
            spline.knots().to_vec(),
            hypercurve::SplinePeriodicity2::NonPeriodic,
            &policy,
        ) {
            if let Ok(refined) = curve.into_value().insert_knot(Real::one(), &policy) {
                let _ = refined.into_value().remove_knot(Real::one(), &policy);
            }
        }
    }
});
