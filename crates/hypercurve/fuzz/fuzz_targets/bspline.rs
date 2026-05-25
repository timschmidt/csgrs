#![no_main]

use hypercurve::{
    Classification, CurvePolicy, Point2, PolynomialBSplineCurve2, RationalBSplineCurve2,
    RationalQuadraticBSplineCurve2, Real, RetainedBSplineSpanFactReport2,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i32) -> Real {
    value.into()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(r(x as i32 - 128), r(y as i32 - 128))
}

fn touch_span_fact_report(report: &RetainedBSplineSpanFactReport2) {
    for span in report.span_facts() {
        let _ = span.span_index();
        let _ = span.knot_interval();
        let _ = span.bounds();
        let _ = span.x_monotonicity();
        let _ = span.y_monotonicity();
        let _ = span.topology_status();
        if let Some(weights) = span.weight_domain() {
            let _ = weights.weight_count();
            let _ = weights.certified_nonzero_count();
            let _ = weights.all_weights_certified_nonzero();
        }
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let policy = CurvePolicy::certified();
    let degree = if data[0] & 1 == 0 { 2 } else { 3 };
    let control_count = degree + 3;
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
        let _ = spline.retained_curve_profile(0, &policy).map(|classification| {
            let Classification::Decided(profile) = classification else {
                return;
            };
            let _ = profile.identity();
            let _ = profile.domain();
            let _ = profile.trim();
            let _ = profile.endpoints();
            let _ = profile.cache_summary();
        });
        let _ = spline.extract_bezier_spans(&policy).map(|classification| {
            let Classification::Decided(extraction) = classification else {
                return;
            };
            let _ = extraction.span_fact_report(&policy).map(|classification| {
                let Classification::Decided(report) = classification else {
                    return;
                };
                touch_span_fact_report(&report);
            });
        });
    }
    let weights = controls
        .iter()
        .enumerate()
        .map(|(index, _)| Real::from(((data[index % data.len()] % 7) as i32) + 1))
        .collect::<Vec<_>>();
    if let Ok(Classification::Decided(spline)) = RationalBSplineCurve2::try_new(
        degree,
        controls.clone(),
        weights.clone(),
        knots.clone(),
        &policy,
    )
    {
        if let Ok(Classification::Decided(extraction)) = spline.extract_bezier_spans(&policy) {
            let _ = spline.retained_curve_profile(1, &policy);
            let _ = extraction.span_fact_report(&policy).map(|classification| {
                let Classification::Decided(report) = classification else {
                    return;
                };
                touch_span_fact_report(&report);
            });
            let _ = extraction.native_topology_report(&policy).map(|classification| {
                let Classification::Decided(report) = classification else {
                    return;
                };
                for span in report.span_reports() {
                    let _ = span.span_index();
                    let _ = span.degree();
                    let _ = span.knot_interval();
                    let _ = span.status();
                    let _ = span.native_subcurve();
                }
                let _ = report.is_fully_native_exact();
            });
            let _ = extraction.native_subcurves(&policy);
        }
    }
    if degree == 2
        && let Ok(Classification::Decided(spline)) =
            RationalQuadraticBSplineCurve2::try_new(controls, weights, knots, &policy)
    {
        let _ = spline.retained_curve_profile(2, &policy);
        let _ = spline.extract_bezier_spans(&policy).map(|classification| {
            let Classification::Decided(extraction) = classification else {
                return;
            };
            let _ = extraction.span_fact_report(&policy).map(|classification| {
                let Classification::Decided(report) = classification else {
                    return;
                };
                touch_span_fact_report(&report);
            });
        });
    }
});
