use cavalier_contours::core::math::{CircleCircleIntr::*, Vector2, circle_circle_intr};

macro_rules! assert_case_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (NoIntersect, NoIntersect) => {}
            (TangentIntersect { point: a1 }, TangentIntersect { point: a2 }) if a1.fuzzy_eq(a2) => {
            }
            (
                TwoIntersects {
                    point1: a1,
                    point2: b1,
                },
                TwoIntersects {
                    point1: a2,
                    point2: b2,
                },
            ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
            (Overlapping, Overlapping) => (),
            _ => panic!(
                "intersect cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        };
    };
}

#[test]
fn no_intersect_outside() {
    let r1 = 1.0;
    let c1 = Vector2::new(-1.0, -1.0);
    let r2 = 0.5;
    let c2 = Vector2::new(0.0, 5.0);
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn no_intersect_inside() {
    let r1 = 5.0;
    let c1 = Vector2::new(-1.0, -1.0);
    let r2 = 0.5;
    let c2 = Vector2::new(1.0, 1.0);
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn tangent_intersect_outside() {
    let r1 = 1.0;
    let c1 = Vector2::new(-1.0, 1.0);
    let r2 = 0.5;
    let c2 = Vector2::new(0.5, 1.0);
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    assert_case_eq!(
        result,
        TangentIntersect {
            point: Vector2::new(0.0, 1.0)
        }
    );
}

#[test]
fn tangent_intersect_inside() {
    let r1 = 3.0;
    let c1 = Vector2::new(0.0, 1.0);
    let r2 = 4.0;
    let c2 = Vector2::new(0.0, 0.0);
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    assert_case_eq!(
        result,
        TangentIntersect {
            point: Vector2::new(0.0, 4.0)
        }
    );
}

#[test]
fn two_intersects() {
    let r1 = 3.0;
    let c1 = Vector2::new(0.0, 1.0);
    let r2 = 4.0;
    let c2 = Vector2::new(5.0, 5.0);
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    let expected_point1 = Vector2::new(2.945782625365772, 1.567771718292785);
    let expected_point2 = Vector2::new(1.2005588380488623, 3.749301452438922);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: expected_point1,
            point2: expected_point2
        }
    );
}

#[test]
fn overlapping() {
    let r1 = 1.0;
    let c1 = Vector2::new(-1.0, 1.0);
    let r2 = r1;
    let c2 = c1;
    let result = circle_circle_intr(r1, c1, r2, c2, 1e-5);
    assert_case_eq!(result, Overlapping::<f64>);
}
