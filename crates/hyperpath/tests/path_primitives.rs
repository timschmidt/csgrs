use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    Axis, ClearanceStatus, LinePathSegment, NetId, PathProvenance, PathSourceFormat,
    PcbCircularPad, PcbRectPad, PcbTrace, PcbViaStack, SegmentParameterOrder, SweptLineSegment,
    TraceLayer, ViaAnnularRingReport, build_length_match_problem, certify_length_extension,
    check_trace_clearance, check_trace_pad_clearance, check_trace_rect_pad_clearance,
    check_trace_via_clearance,
};
use hyperreal::{Rational, Real};
use proptest::prelude::*;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn trace(net: u32, layer: u16, start: Point2, end: Point2, width: i64) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(layer),
        SweptLineSegment::new(LinePathSegment::new(start, end), r(width)).unwrap(),
    )
}

#[test]
fn line_segment_caches_axis_and_exact_length_facts() {
    let segment = LinePathSegment::new(p(0, 4), p(9, 4));

    assert_eq!(segment.facts().axis_aligned, Some(Axis::X));
    assert_eq!(segment.facts().known_degenerate, Some(false));
    assert!(segment.facts().endpoint_exact.all_exact_rational);
    assert_eq!(segment.axis_length(PredicatePolicy::default()), Some(r(9)));
    assert_eq!(segment.length_squared(), r(81));
}

#[test]
fn line_segment_retains_source_grid_provenance_and_prepared_bounds() {
    let provenance = PathProvenance::fixed_grid(PathSourceFormat::Gerber, 1_000_000).unwrap();
    let segment = LinePathSegment::with_provenance(p(9, -2), p(3, 4), provenance);
    let bounds = segment.prepared_bounds();

    assert_eq!(segment.provenance(), provenance);
    assert_eq!(segment.bounds_min(), &p(3, -2));
    assert_eq!(segment.bounds_max(), &p(9, 4));
    assert_eq!(bounds.min(), segment.bounds_min());
    assert_eq!(bounds.max(), segment.bounds_max());
    assert!(bounds.contains_point(&p(6, 0)).value().unwrap());
    assert!(!bounds.contains_point(&p(10, 0)).value().unwrap());
}

#[test]
fn segment_parameter_order_respects_reversed_direction() {
    let segment = LinePathSegment::new(p(10, 0), p(0, 0));

    assert_eq!(
        segment.compare_points_along(&p(8, 0), &p(2, 0), PredicatePolicy::default()),
        SegmentParameterOrder::Before
    );
    assert_eq!(
        segment.compare_points_along(&p(2, 0), &p(8, 0), PredicatePolicy::default()),
        SegmentParameterOrder::After
    );
}

#[test]
fn pcb_clearance_certifies_same_layer_parallel_gap() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 2);
    let second = trace(2, 0, p(0, 5), p(10, 5), 2);

    let clear = check_trace_clearance(&first, &second, &r(3), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);
    assert_eq!(clear.axis_gap, Some(r(5)));

    let violation = check_trace_clearance(&first, &second, &r(4), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_clearance_reports_no_short_before_spacing() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 1);
    let second = trace(2, 0, p(5, -5), p(5, 5), 1);

    let report = check_trace_clearance(&first, &second, &r(1), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_clearance_ignores_same_net_and_different_layer_pairs() {
    let first = trace(1, 0, p(0, 0), p(10, 0), 1);
    let same_net = trace(1, 0, p(5, -5), p(5, 5), 1);
    let other_layer = trace(2, 1, p(5, -5), p(5, 5), 1);

    assert_eq!(
        check_trace_clearance(&first, &same_net, &r(1), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
    assert_eq!(
        check_trace_clearance(&first, &other_layer, &r(1), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
}

#[test]
fn pcb_trace_pad_clearance_certifies_round_pad_gap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 5), r(2)).unwrap();

    let clear = check_trace_pad_clearance(&trace, &pad, &r(3), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);

    let violation = check_trace_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_pad_clearance_reports_copper_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbCircularPad::new(NetId(2), TraceLayer(0), p(5, 1), r(2)).unwrap();

    let report = check_trace_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_pad_rejects_negative_diameter() {
    let error = PcbCircularPad::new(NetId(1), TraceLayer(0), p(0, 0), r(-1))
        .expect_err("negative pad diameter must be rejected");
    assert_eq!(error, "pad diameter must be nonnegative");
}

#[test]
fn pcb_trace_via_clearance_respects_layer_span() {
    let trace = trace(1, 1, p(0, 0), p(10, 0), 2);
    let via = PcbViaStack::new(NetId(2), TraceLayer(0), TraceLayer(2), p(5, 5), r(2)).unwrap();
    let off_layer =
        PcbViaStack::new(NetId(2), TraceLayer(2), TraceLayer(3), p(5, 5), r(2)).unwrap();

    assert_eq!(
        check_trace_via_clearance(&trace, &via, &r(3), PredicatePolicy::default()).status,
        ClearanceStatus::CertifiedClear
    );
    assert_eq!(
        check_trace_via_clearance(&trace, &off_layer, &r(3), PredicatePolicy::default()).status,
        ClearanceStatus::NotApplicable
    );
}

#[test]
fn pcb_via_rejects_reversed_layer_span() {
    let error = PcbViaStack::new(NetId(1), TraceLayer(3), TraceLayer(2), p(0, 0), r(1))
        .expect_err("reversed via layer span must be rejected");
    assert_eq!(error, "via start layer must not be above end layer");
}

#[test]
fn pcb_via_annular_ring_certifies_fabrication_requirement() {
    let via = PcbViaStack::with_drill(NetId(1), TraceLayer(0), TraceLayer(2), p(0, 0), r(10), r(4))
        .unwrap();

    assert_eq!(
        via.certify_annular_ring(&r(3), PredicatePolicy::default()),
        ViaAnnularRingReport::Certified
    );
    assert_eq!(
        via.certify_annular_ring(&r(4), PredicatePolicy::default()),
        ViaAnnularRingReport::Violation
    );
}

#[test]
fn pcb_via_annular_ring_reports_missing_and_invalid_inputs() {
    let without_drill =
        PcbViaStack::new(NetId(1), TraceLayer(0), TraceLayer(1), p(0, 0), r(10)).unwrap();
    let negative_drill = PcbViaStack::with_drill(
        NetId(1),
        TraceLayer(0),
        TraceLayer(1),
        p(0, 0),
        r(10),
        r(-1),
    )
    .expect_err("negative drill diameter must be rejected");

    assert_eq!(
        without_drill.certify_annular_ring(&r(1), PredicatePolicy::default()),
        ViaAnnularRingReport::UnknownNoDrill
    );
    assert_eq!(
        without_drill.certify_annular_ring(&r(-1), PredicatePolicy::default()),
        ViaAnnularRingReport::UnknownNoDrill
    );
    assert_eq!(negative_drill, "via drill diameter must be nonnegative");
}

#[test]
fn pcb_trace_rect_pad_clearance_certifies_non_circular_pad_gap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRectPad::new(NetId(2), TraceLayer(0), p(5, 6), r(4), r(2)).unwrap();

    let clear = check_trace_rect_pad_clearance(&trace, &pad, &r(4), PredicatePolicy::default());
    assert_eq!(clear.status, ClearanceStatus::CertifiedClear);

    let violation = check_trace_rect_pad_clearance(&trace, &pad, &r(5), PredicatePolicy::default());
    assert_eq!(violation.status, ClearanceStatus::ClearanceViolation);
}

#[test]
fn pcb_trace_rect_pad_clearance_reports_overlap() {
    let trace = trace(1, 0, p(0, 0), p(10, 0), 2);
    let pad = PcbRectPad::new(NetId(2), TraceLayer(0), p(5, 1), r(4), r(2)).unwrap();

    let report = check_trace_rect_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());
    assert_eq!(report.status, ClearanceStatus::NoShortViolation);
}

#[test]
fn pcb_rect_pad_rejects_negative_extent() {
    let error = PcbRectPad::new(NetId(1), TraceLayer(0), p(0, 0), r(-1), r(1))
        .expect_err("negative rectangular pad width must be rejected");
    assert_eq!(error, "rect pad width must be nonnegative");
}

#[test]
fn swept_segment_rejects_negative_width() {
    let error = SweptLineSegment::new(LinePathSegment::new(p(0, 0), p(1, 0)), r(-1))
        .expect_err("negative trace/cutter width must be rejected");
    assert_eq!(error, "swept path width must be nonnegative");
}

#[test]
fn length_match_problem_certifies_exact_extension_candidate() {
    let model = build_length_match_problem(r(100), r(125), r(25));
    let report = certify_length_extension(&model);

    assert!(report.all_satisfied());
    assert_eq!(model.extra_length_symbol.0, 0);
}

#[test]
fn length_match_problem_reports_wrong_extension_as_violation() {
    let model = build_length_match_problem(r(100), r(125), r(20));
    let report = certify_length_extension(&model);

    assert!(report.has_certified_violation());
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn horizontal_parallel_gap_matches_integer_coordinate_difference(
        y0 in -100_i16..=100,
        gap in 0_i16..=100,
        width in 0_i16..=10,
    ) {
        let first = trace(1, 0, p(0, i64::from(y0)), p(10, i64::from(y0)), i64::from(width));
        let second_y = i64::from(y0) + i64::from(gap);
        let second = trace(2, 0, p(0, second_y), p(10, second_y), i64::from(width));
        let report = check_trace_clearance(&first, &second, &r(0), PredicatePolicy::default());
        if gap == 0 {
            prop_assert_eq!(report.status, ClearanceStatus::NoShortViolation);
        } else {
            prop_assert_eq!(report.axis_gap, Some(r(i64::from(gap))));
        }
    }

    #[test]
    fn trace_pad_clearance_handles_generated_axis_gaps(
        gap in 0_i16..=100,
        trace_width in 0_i16..=10,
        pad_diameter in 0_i16..=10,
    ) {
        let trace = trace(1, 0, p(0, 0), p(20, 0), i64::from(trace_width));
        let pad = PcbCircularPad::new(
            NetId(2),
            TraceLayer(0),
            p(10, i64::from(gap)),
            r(i64::from(pad_diameter)),
        ).unwrap();
        let report = check_trace_pad_clearance(&trace, &pad, &r(0), PredicatePolicy::default());

        let doubled_gap = i64::from(gap) * 2;
        let overlap = i64::from(trace_width) + i64::from(pad_diameter);
        if doubled_gap <= overlap {
            prop_assert_eq!(report.status, ClearanceStatus::NoShortViolation);
        } else {
            prop_assert_eq!(report.status, ClearanceStatus::CertifiedClear);
        }
    }
}
