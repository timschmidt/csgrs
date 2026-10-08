use hypertri::{
    Point2, PointD, PolygonInput, PredicatePolicy, Rational, Real, TriangulationContext,
    TriangulationOptions,
};

const STRICT: TriangulationContext = TriangulationContext::new(PredicatePolicy::STRICT);

fn trace<T>(name: &str, workload: impl FnOnce() -> hypertri::Result<T>) -> T {
    hyperreal::dispatch_trace::reset();
    let value = hyperreal::dispatch_trace::with_recording(workload)
        .unwrap_or_else(|error| panic!("{name} must remain strictly certified: {error}"));
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "{name} emitted no exact-computation trace"
    );
    println!("{name}: {correlation:?}");
    value
}

fn main() {
    let polygon = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(4), Real::from(0)),
        Point2::new(Real::from(4), Real::from(4)),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::from(0), Real::from(4)),
    ];
    trace("earcut", || hypertri::earcut(&STRICT, &polygon, &[]));

    let points = vec![
        Point2::new(Real::from(0), Real::from(0)),
        Point2::new(Real::from(3), Real::from(0)),
        Point2::new(Real::from(3), Real::from(3)),
        Point2::new(Real::from(0), Real::from(3)),
        Point2::new(Real::from(1), Real::from(2)),
    ];
    trace("delaunay", || hypertri::cdt::delaunay(&STRICT, &points));

    let quarter = || Real::from(Rational::fraction(1, 4).unwrap());
    let nd_points = vec![
        PointD::new(vec![Real::from(0), Real::from(0), Real::from(0)]),
        PointD::new(vec![Real::from(1), Real::from(0), Real::from(0)]),
        PointD::new(vec![Real::from(0), Real::from(1), Real::from(0)]),
        PointD::new(vec![Real::from(0), Real::from(0), Real::from(1)]),
        PointD::new(vec![quarter(), quarter(), quarter()]),
    ];
    trace("nd_delaunay", || {
        hypertri::nd::delaunay_complex(&STRICT, &nd_points)
    });

    let input = PolygonInput::new(polygon, vec![]);
    trace("runtime_select", || {
        hypertri::triangulate_polygon(&STRICT, &input, TriangulationOptions::default())
    });
}
