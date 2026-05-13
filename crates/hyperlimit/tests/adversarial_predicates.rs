use hyperlimit::orient::orient2d_with_policy;
use hyperlimit::{
    LineSide, Plane3, PlaneSide, Point2, Point3, PredicateOutcome, PredicatePolicy, Sign,
    classify_point_line, classify_point_oriented_plane, classify_point_plane, incircle2d, orient2d,
    orient2d_batch, orient3d,
};

fn decided<T: Copy>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("adversarial case should decide")
}

fn assert_sign_or_unknown(outcome: PredicateOutcome<Sign>, expected: Sign) {
    if let Some(actual) = outcome.value() {
        assert_eq!(actual, expected);
    }
}

#[test]
fn orient2d_handles_exact_degenerate_and_near_degenerate_cases() {
    let a = Point2::new(0.0, 0.0);
    let b = Point2::new(1.0, 0.0);

    assert_eq!(
        decided(orient2d(&a, &b, &Point2::new(0.0, 1.0))),
        Sign::Positive
    );
    assert_eq!(
        decided(orient2d(&a, &b, &Point2::new(0.0, -1.0))),
        Sign::Negative
    );
    assert_sign_or_unknown(orient2d(&a, &b, &Point2::new(0.5, 0.0)), Sign::Zero);
    assert_eq!(
        decided(orient2d_with_policy(
            &a,
            &b,
            &Point2::new(0.5, f64::from_bits(1)),
            PredicatePolicy::APPROXIMATE,
        )),
        Sign::Positive
    );
}

#[test]
fn orientation_sign_changes_under_coordinate_permutation() {
    let a = Point2::new(-3.0, 2.0);
    let b = Point2::new(5.0, -7.0);
    let c = Point2::new(11.0, 13.0);

    assert_eq!(decided(orient2d(&a, &b, &c)), decided(orient2d(&b, &c, &a)));
    assert_eq!(
        decided(orient2d(&a, &b, &c)).reversed(),
        decided(orient2d(&a, &c, &b))
    );
    assert_eq!(decided(classify_point_line(&a, &b, &c)), LineSide::Left);
}

#[test]
fn batch_predicates_match_scalar_predicates_on_hostile_rows() {
    let cases = vec![
        (
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.5, 0.5 + 1.0e-15),
        ),
        (
            Point2::new(1.0e150, 1.0e150),
            Point2::new(1.0e150 + 1.0, 1.0e150),
            Point2::new(1.0e150, 1.0e150 + 1.0),
        ),
        (
            Point2::new(-1.0e-150, 0.0),
            Point2::new(0.0, 1.0e-150),
            Point2::new(1.0e-150, 0.0),
        ),
    ];

    assert_eq!(
        orient2d_batch(&cases),
        cases
            .iter()
            .map(|(a, b, c)| orient2d(a, b, c))
            .collect::<Vec<_>>()
    );
}

#[test]
fn orient3d_and_plane_classification_agree_on_sides_and_degeneracy() {
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(0.0, 1.0, 0.0);

    assert_eq!(
        decided(orient3d(&a, &b, &c, &Point3::new(0.0, 0.0, 1.0))),
        Sign::Negative
    );
    assert_eq!(
        decided(orient3d(&a, &b, &c, &Point3::new(0.0, 0.0, -1.0))),
        Sign::Positive
    );
    assert_sign_or_unknown(
        orient3d(&a, &b, &c, &Point3::new(0.25, 0.25, 0.0)),
        Sign::Zero,
    );

    assert_eq!(
        decided(classify_point_oriented_plane(
            &a,
            &b,
            &c,
            &Point3::new(0.0, 0.0, 1.0)
        )),
        PlaneSide::Below
    );
    assert_eq!(
        decided(classify_point_plane(
            &Point3::new(0.0, 0.0, 3.0),
            &Plane3::new(Point3::new(0.0, 0.0, 1.0), -2.0),
        )),
        PlaneSide::Above
    );
}

#[test]
fn incircle_detects_inside_outside_and_on_circle_cases() {
    let a = Point2::new(1.0, 0.0);
    let b = Point2::new(0.0, 1.0);
    let c = Point2::new(-1.0, 0.0);

    assert_sign_or_unknown(
        incircle2d(&a, &b, &c, &Point2::new(0.0, 0.0)),
        Sign::Positive,
    );
    assert_sign_or_unknown(
        incircle2d(&a, &b, &c, &Point2::new(0.0, 2.0)),
        Sign::Negative,
    );
    assert_sign_or_unknown(incircle2d(&a, &b, &c, &Point2::new(0.0, -1.0)), Sign::Zero);
}
