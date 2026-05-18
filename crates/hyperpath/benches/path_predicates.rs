use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, NetId, PcbCircularPad, PcbRectPad, PcbTrace, PcbViaStack, SweptLineSegment,
    TraceLayer, build_length_match_problem, certify_length_extension, check_trace_clearance,
    check_trace_pad_clearance, check_trace_rect_pad_clearance, check_trace_via_clearance,
};
use hyperreal::{Rational, Real};

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn trace(net: u32, start: Point2, end: Point2) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(0),
        SweptLineSegment::new(LinePathSegment::new(start, end), r(2)).unwrap(),
    )
}

fn path_predicates(c: &mut Criterion) {
    let first = trace(1, p(0, 0), p(1000, 0));
    let second = trace(2, p(0, 6), p(1000, 6));
    c.bench_function("axis_aligned_trace_clearance_exact", |b| {
        b.iter(|| check_trace_clearance(&first, &second, &r(3), PredicatePolicy::default()))
    });

    let crossing = trace(2, p(500, -100), p(500, 100));
    c.bench_function("trace_no_short_exact_segment_predicate", |b| {
        b.iter(|| check_trace_clearance(&first, &crossing, &r(1), PredicatePolicy::default()))
    });

    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(500, 6), r(2)).unwrap();
    c.bench_function("trace_pad_clearance_exact", |b| {
        b.iter(|| check_trace_pad_clearance(&first, &pad, &r(3), PredicatePolicy::default()))
    });

    let via = PcbViaStack::new(NetId(2), TraceLayer(0), TraceLayer(2), p(500, 6), r(2)).unwrap();
    c.bench_function("trace_via_clearance_exact", |b| {
        b.iter(|| check_trace_via_clearance(&first, &via, &r(3), PredicatePolicy::default()))
    });

    let rect = PcbRectPad::new(NetId(2), TraceLayer(0), p(500, 6), r(10), r(2)).unwrap();
    c.bench_function("trace_rect_pad_clearance_exact", |b| {
        b.iter(|| check_trace_rect_pad_clearance(&first, &rect, &r(3), PredicatePolicy::default()))
    });

    let model = build_length_match_problem(r(1000), r(1250), r(250));
    c.bench_function("length_match_hypersolve_certification", |b| {
        b.iter(|| certify_length_extension(&model))
    });
}

criterion_group!(benches, path_predicates);
criterion_main!(benches);
