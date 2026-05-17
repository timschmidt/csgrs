use hypercurve::{
    ArcArcIntersection, CircleCircleRelation, CircularArc2, CurvePolicy, LineArcIntersection,
    LineCircleRelation, LineSeg2, Point2, Real,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn circle_arc() -> CircularArc2 {
    CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap()
}

#[test]
fn supporting_line_circle_relation_classifies_disjoint_tangent_and_secant_cases() {
    let circle = circle_arc();

    let disjoint = LineSeg2::try_new(p(-10, 6), p(10, 6)).unwrap();
    assert!(
        disjoint
            .supporting_line_circle_relation(&circle, &policy())
            .unwrap()
            .is_disjoint(),
        "line above the radius-5 circle must be disjoint"
    );

    let tangent = LineSeg2::try_new(p(-10, 5), p(10, 5)).unwrap();
    match tangent
        .supporting_line_circle_relation(&circle, &policy())
        .unwrap()
    {
        LineCircleRelation::Tangent { point, line_param } => {
            assert_eq!(point, p(0, 5));
            assert_eq!(line_param, (Real::one() / Real::from(2_i8)).unwrap());
        }
        other => panic!("expected tangent relation, got {other:?}"),
    }

    let secant = LineSeg2::try_new(p(-10, 0), p(10, 0)).unwrap();
    match secant
        .supporting_line_circle_relation(&circle, &policy())
        .unwrap()
    {
        LineCircleRelation::Secant {
            first_point,
            first_param,
            second_point,
            second_param,
        } => {
            assert_eq!(first_point, p(-5, 0));
            assert_eq!(first_param, (Real::one() / Real::from(4_i8)).unwrap());
            assert_eq!(second_point, p(5, 0));
            assert_eq!(second_param, (Real::from(3_i8) / Real::from(4_i8)).unwrap());
        }
        other => panic!("expected secant relation, got {other:?}"),
    }
}

#[test]
fn line_arc_intersection_reuses_supporting_circle_relation_roots() {
    let arc = circle_arc();
    let line = LineSeg2::try_new(p(-10, 0), p(10, 0)).unwrap();
    let relation = line
        .supporting_line_circle_relation(&arc, &policy())
        .unwrap();
    let LineCircleRelation::Secant {
        first_param,
        second_param,
        ..
    } = relation
    else {
        panic!("expected line support to meet circle at arc endpoints");
    };

    match line.intersect_arc(&arc, &policy()).unwrap() {
        LineArcIntersection::TwoPoints { first, second } => {
            assert_eq!(first.line_param, first_param);
            assert_eq!(first.point, p(-5, 0));
            assert_eq!(second.line_param, second_param);
            assert_eq!(second.point, p(5, 0));
        }
        other => panic!("expected two endpoint hits after finite arc filtering, got {other:?}"),
    }
}

#[test]
fn circle_circle_relation_classifies_coincident_disjoint_tangent_and_secant_cases() {
    let base = circle_arc();
    let same = CircularArc2::try_from_center(p(0, 5), p(0, -5), p(0, 0), false).unwrap();
    assert!(
        base.circle_relation(&same, &policy())
            .unwrap()
            .is_coincident(),
        "arcs on the same center and radius must expose coincident full circles"
    );

    let disjoint = CircularArc2::try_from_center(p(17, 0), p(7, 0), p(12, 0), false).unwrap();
    assert!(
        base.circle_relation(&disjoint, &policy())
            .unwrap()
            .is_disjoint(),
        "radius-5 circles twelve units apart must be disjoint"
    );

    let tangent = CircularArc2::try_from_center(p(15, 0), p(5, 0), p(10, 0), false).unwrap();
    match base.circle_relation(&tangent, &policy()).unwrap() {
        CircleCircleRelation::Tangent { point } => assert_eq!(point, p(5, 0)),
        other => panic!("expected tangent full-circle relation, got {other:?}"),
    }

    let secant = CircularArc2::try_from_center(p(4, -3), p(4, 3), p(8, 0), true).unwrap();
    match base.circle_relation(&secant, &policy()).unwrap() {
        CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => {
            assert_eq!(first_point, p(4, 3));
            assert_eq!(second_point, p(4, -3));
        }
        other => panic!("expected secant full-circle relation, got {other:?}"),
    }
}

#[test]
fn arc_arc_intersection_reuses_circle_relation_witnesses_before_sweep_filtering() {
    let first = CircularArc2::try_from_center(p(4, 3), p(4, -3), p(0, 0), true).unwrap();
    let second = CircularArc2::try_from_center(p(4, -3), p(4, 3), p(8, 0), true).unwrap();
    let relation = first.circle_relation(&second, &policy()).unwrap();
    let CircleCircleRelation::Secant {
        first_point,
        second_point,
    } = relation
    else {
        panic!("expected secant circle relation for crossing arcs");
    };

    match first.intersect_arc(&second, &policy()).unwrap() {
        ArcArcIntersection::TwoPoints { first, second } => {
            assert_eq!(first.point, first_point);
            assert_eq!(second.point, second_point);
        }
        other => panic!("expected two arc hits from circle witnesses, got {other:?}"),
    }
}
