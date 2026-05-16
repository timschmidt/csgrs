use cavalier_contours::core::{
    math::{LineCircleIntr::*, Vector2, line_circle_intr},
    traits::FuzzyEq,
};

macro_rules! assert_case_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (NoIntersect, NoIntersect) => {}
            (TangentIntersect { t0: a1 }, TangentIntersect { t0: a2 }) if a1.fuzzy_eq(a2) => {}
            (TwoIntersects { t0: a1, t1: b1 }, TwoIntersects { t0: a2, t1: b2 })
                if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
            _ => panic!(
                "intersect cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        };
    };
}

#[test]
fn no_intersect() {
    let p0 = Vector2::new(-1.0, -1.0);
    let p1 = Vector2::new(1.0, 1.0);
    let circle_center = Vector2::new(0.0, 5.0);
    let radius = 0.5;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn no_intersect_vertical() {
    let p0 = Vector2::new(0.0, -1.0);
    let p1 = Vector2::new(0.0, 1.0);
    let circle_center = Vector2::new(2.0, 0.0);
    let radius = 0.5;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn no_intersect_horizontal() {
    let p0 = Vector2::new(1.0, 1.0);
    let p1 = Vector2::new(3.0, 1.0);
    let circle_center = Vector2::new(2.0, -2.0);
    let radius = 0.5;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn two_intersects_true() {
    let p0 = Vector2::new(-1.0, -1.0);
    let p1 = Vector2::new(1.0, 1.0);
    // placing edge of circle at (0, 0)
    let radius = 0.5f64;
    let offset = (radius * radius / 2.0).sqrt();
    let circle_center = Vector2::new(offset, offset);
    let expected_t1_intr_point_x = 2.0 * offset;
    let expected_t1 = (expected_t1_intr_point_x - p0.x) / (p1.x - p0.x);
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            t0: 0.5,
            t1: expected_t1
        }
    );
}

#[test]
fn two_intersects_seg_inside_vertical() {
    let p0 = Vector2::new(0.0, -1.0);
    let p1 = Vector2::new(0.0, 1.0);
    let circle_center = Vector2::new(0.0, 0.0);
    let radius = 1.0;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
}

#[test]
fn two_intersects_seg_inside_horizontal() {
    let p0 = Vector2::new(-1.0, 0.0);
    let p1 = Vector2::new(1.0, 0.0);
    let circle_center = Vector2::new(0.0, 0.0);
    let radius = 1.0;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
}

#[test]
fn two_intersects_seg_touching() {
    let p0 = Vector2::new(0.0, -1.0);
    let p1 = Vector2::new(0.0, 1.0);
    let circle_center = Vector2::new(0.0, 0.0);
    let radius = 1.0;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TwoIntersects { t0: 0.0, t1: 1.0 });
}

#[test]
fn tangent_intersect_vertical() {
    let p0 = Vector2::new(0.0, -1.0);
    let p1 = Vector2::new(0.0, 1.0);
    let circle_center = Vector2::new(1.0, 0.0);
    let radius = 1.0;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TangentIntersect { t0: 0.5 });
}

#[test]
fn tangent_intersect_horizontal() {
    let p0 = Vector2::new(-1.0, 0.0);
    let p1 = Vector2::new(1.0, 0.0);
    let circle_center = Vector2::new(0.0, -1.0);
    let radius = 1.0;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TangentIntersect { t0: 0.5 });
}

#[test]
fn tangent_at_start_point() {
    // this is a case that previously failed due to numeric stability issues
    let p0 = Vector2::new(161.29, 113.665);
    let p1 = Vector2::new(167.64, 113.665);
    let circle_center = Vector2::new(161.29, 114.30000000000001);
    let radius = 0.634_999_999_999_998;
    let result = line_circle_intr(p0, p1, radius, circle_center, 1e-5);
    assert_case_eq!(result, TangentIntersect { t0: 0.0 });
}
