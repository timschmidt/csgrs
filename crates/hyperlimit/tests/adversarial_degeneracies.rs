use hyperlimit::{
    LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, Sign, classify_point_line,
    classify_point_oriented_plane, classify_point_plane, incircle2d, insphere3d, orient2d,
    orient3d,
};

fn decided<T: Copy>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("case should decide")
}

fn assert_sign_or_unknown(outcome: PredicateOutcome<Sign>, expected: Sign) {
    if let Some(actual) = outcome.value() {
        assert_eq!(actual, expected);
    }
}

#[test]
fn orient2d_translation_and_scaling_invariance_on_near_collinear_rows() {
    let a = Point2::new(-1.0e6, -1.0e6);
    let b = Point2::new(1.0e6, 1.0e6);
    let c = Point2::new(0.0, 1.0e-6);
    let offset = Point2::new(4096.0, -8192.0);
    let scale = 8.0;

    let sign = decided(orient2d(&a, &b, &c));
    let translated = decided(orient2d(
        &Point2::new(a.x + offset.x, a.y + offset.y),
        &Point2::new(b.x + offset.x, b.y + offset.y),
        &Point2::new(c.x + offset.x, c.y + offset.y),
    ));
    let scaled = decided(orient2d(
        &Point2::new(a.x * scale, a.y * scale),
        &Point2::new(b.x * scale, b.y * scale),
        &Point2::new(c.x * scale, c.y * scale),
    ));

    assert_eq!(sign, Sign::Positive);
    assert_eq!(translated, sign);
    assert_eq!(scaled, sign);
}

#[test]
fn classify_point_line_is_consistent_for_reversed_line_orientation() {
    let a = Point2::new(-3.0, 2.0);
    let b = Point2::new(5.0, -7.0);
    let p = Point2::new(11.0, 13.0);

    assert_eq!(decided(classify_point_line(&a, &b, &p)), LineSide::Left);
    assert_eq!(decided(classify_point_line(&b, &a, &p)), LineSide::Right);
}

#[test]
fn orient3d_translation_invariance_and_swap_reversal_on_small_height() {
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0e6, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0e6, 0.0);
    let d = Point3::new(0.25e6, 0.25e6, 1.0e-6);
    let offset = Point3::new(4096.0, -8192.0, 16384.0);

    let sign = decided(orient3d(&a, &b, &c, &d));
    let translated = decided(orient3d(
        &Point3::new(a.x + offset.x, a.y + offset.y, a.z + offset.z),
        &Point3::new(b.x + offset.x, b.y + offset.y, b.z + offset.z),
        &Point3::new(c.x + offset.x, c.y + offset.y, c.z + offset.z),
        &Point3::new(d.x + offset.x, d.y + offset.y, d.z + offset.z),
    ));
    let swapped = decided(orient3d(&b, &a, &c, &d));

    assert_eq!(translated, sign);
    assert_eq!(swapped, sign.reversed());
}

#[test]
fn plane_classification_matches_oriented_plane_for_axis_aligned_case() {
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);
    let above = Point3::new(0.25, 0.25, 1.0e-12);
    let below = Point3::new(0.25, 0.25, -1.0e-12);
    let plane = Plane3::new(Point3::new(0.0, 0.0, 1.0), 0.0);

    assert_eq!(
        decided(classify_point_plane(&above, &plane)),
        PlaneSide::Above
    );
    assert_eq!(
        decided(classify_point_plane(&below, &plane)),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_oriented_plane(&a, &b, &c, &above)),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_oriented_plane(&a, &b, &c, &below)),
        PlaneSide::Above
    );
}

#[test]
fn circle_and_sphere_predicates_distinguish_on_boundary_from_tiny_offsets() {
    let a = Point2::new(1.0, 0.0);
    let b = Point2::new(0.0, 1.0);
    let c = Point2::new(-1.0, 0.0);

    assert!(matches!(
        incircle2d(&a, &b, &c, &Point2::new(0.0, -1.0)).value(),
        None | Some(Sign::Zero)
    ));
    assert_sign_or_unknown(
        incircle2d(&a, &b, &c, &Point2::new(0.0, -1.0 + 1.0e-6)),
        Sign::Positive,
    );
    assert_sign_or_unknown(
        incircle2d(&a, &b, &c, &Point2::new(0.0, -1.0 - 1.0e-6)),
        Sign::Negative,
    );

    let s0 = Point3::new(1.0, 0.0, 0.0);
    let s1 = Point3::new(-1.0, 0.0, 0.0);
    let s2 = Point3::new(0.0, 1.0, 0.0);
    let s3 = Point3::new(0.0, 0.0, 1.0);
    assert_sign_or_unknown(
        insphere3d(&s0, &s1, &s2, &s3, &Point3::new(0.0, 0.0, 0.0)),
        Sign::Positive,
    );
}
