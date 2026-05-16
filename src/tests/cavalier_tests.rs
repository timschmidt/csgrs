use crate::{
    cavalier::Cavalier,
    csg::CSG,
    float_types::{PI, Real},
    sketch::Sketch,
};

fn assert_close(actual: Real, expected: Real, eps: Real) {
    assert!(
        (actual - expected).abs() <= eps,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn cavalier_circle_preserves_exact_arc_metrics() {
    let circle = Cavalier::circle(2.0, ());

    assert_eq!(circle.ccw_loop_count(), 1);
    assert_eq!(circle.cw_loop_count(), 0);
    assert_eq!(circle.arc_segment_count(), 2);
    assert_eq!(circle.line_segment_count(), 0);
    assert_close(circle.signed_area(), PI * 4.0, 1e-9);
    assert_close(circle.path_length(), 2.0 * PI * 2.0, 1e-9);
}

#[test]
fn cavalier_boolean_difference_keeps_hole_loop() {
    let outer = Cavalier::rectangle(10.0, 10.0, ());
    let inner = Cavalier::rectangle(4.0, 4.0, ()).translate(3.0, 3.0, 0.0);

    let donut = outer.difference(&inner);

    assert_eq!(donut.ccw_loop_count(), 1);
    assert_eq!(donut.cw_loop_count(), 1);
    assert_close(donut.signed_area(), 84.0, 1e-9);
    assert!(donut.contains_point(1.0, 1.0));
    assert!(!donut.contains_point(5.0, 5.0));
}

#[test]
fn cavalier_transform_preserves_arcs_only_for_similarity_transforms() {
    let circle = Cavalier::circle(1.0, ());

    let uniform = circle.scale(2.0, 2.0, 1.0);
    assert_eq!(uniform.arc_segment_count(), 2);
    assert_close(uniform.signed_area(), PI * 4.0, 1e-9);

    let non_uniform = circle.scale(2.0, 1.0, 1.0);
    assert_eq!(non_uniform.arc_segment_count(), 0);
    assert!(non_uniform.line_segment_count() > 2);
}

#[test]
fn cavalier_converts_to_and_from_sketch() {
    let original = Cavalier::rounded_rectangle(8.0, 4.0, 1.0, ());
    let sketch: Sketch<()> = original.clone().into();
    let round_trip: Cavalier<()> = sketch.into();

    assert!(original.arc_segment_count() > 0);
    assert_eq!(round_trip.arc_segment_count(), 0);
    assert_close(round_trip.signed_area(), original.signed_area(), 0.05);
}
