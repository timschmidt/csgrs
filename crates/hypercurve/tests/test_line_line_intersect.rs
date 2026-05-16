use cavalier_contours::core::{
    math::{LineLineIntr::*, Vector2, line_line_intr},
    traits::FuzzyEq,
};
use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};

macro_rules! assert_case_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (NoIntersect, NoIntersect) => {}
            (
                TrueIntersect {
                    seg1_t: a1,
                    seg2_t: b1,
                },
                TrueIntersect {
                    seg1_t: a2,
                    seg2_t: b2,
                },
            )
            | (
                FalseIntersect {
                    seg1_t: a1,
                    seg2_t: b1,
                },
                FalseIntersect {
                    seg1_t: a2,
                    seg2_t: b2,
                },
            )
            | (
                Overlapping {
                    seg2_t0: a1,
                    seg2_t1: b1,
                },
                Overlapping {
                    seg2_t0: a2,
                    seg2_t1: b2,
                },
            ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
            _ => panic!(
                "intersect cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        };
    };
}

const TEST_ROTATION_ANGLES: &[f64] = &[FRAC_PI_8, FRAC_PI_6, FRAC_PI_4, FRAC_PI_3, FRAC_PI_2];

#[test]
fn true_intersect() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(-1.0, 1.0);
    let v2 = Vector2::new(1.0, -1.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.5,
            seg2_t: 0.5
        }
    );
}

#[test]
fn end_point_start_point_touch_same_direction() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(1.0, 1.0);
    let v2 = Vector2::new(2.0, 2.0);

    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 1.0,
            seg2_t: 0.0
        }
    );

    // flip argument order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 1.0
        }
    );

    // rotate v1->v2 should get same result
    for &angle in TEST_ROTATION_ANGLES {
        let v2 = v2.rotate_about(v1, angle);
        let result = line_line_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 1.0,
                seg2_t: 0.0
            }
        );
    }
}

#[test]
fn start_points_touch_opposing_direction() {
    let u1 = Vector2::new(0.0, 0.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(0.0, 0.0);
    let v2 = Vector2::new(-1.0, -1.0);

    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.0
        }
    );

    // flip argument order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.0
        }
    );

    // rotate v1->v2 should get same result
    for &angle in TEST_ROTATION_ANGLES {
        let v2 = v2.rotate_about(v1, angle);
        let result = line_line_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );
    }
}

#[test]
fn false_intersect() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(-0.5, -0.5);
    let v1 = Vector2::new(-1.0, 1.0);
    let v2 = Vector2::new(1.0, -1.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        FalseIntersect {
            seg1_t: 2.0,
            seg2_t: 0.5
        }
    );
}

#[test]
fn no_intersect() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(0.0, 1.0);
    let v2 = Vector2::new(1.0, 2.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn no_intersect_vertical() {
    let u1 = Vector2::new(2.0, 0.0);
    let u2 = Vector2::new(2.0, 1.0);
    let v1 = Vector2::new(-1.0, -1.0);
    let v2 = Vector2::new(-1.0, -2.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn no_intersect_horizontal() {
    let u1 = Vector2::new(-2.0, -1.0);
    let u2 = Vector2::new(2.0, -1.0);
    let v1 = Vector2::new(-1.0, 5.0);
    let v2 = Vector2::new(1.0, 5.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn overlapping_intersect() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(0.0, 0.0);
    let v2 = Vector2::new(0.5, 0.5);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        Overlapping {
            seg2_t0: 0.0,
            seg2_t1: 1.0
        }
    );
}

#[test]
fn point_intersect() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = Vector2::new(0.0, 0.0);
    let v2 = Vector2::new(0.0, 0.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.5,
            seg2_t: 0.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.5
        }
    );
}

#[test]
fn point_intersect_vertical() {
    let u1 = Vector2::new(0.0, -1.0);
    let u2 = Vector2::new(0.0, 1.0);
    let v1 = Vector2::new(0.0, 0.0);
    let v2 = Vector2::new(0.0, 0.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.5,
            seg2_t: 0.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.5
        }
    );
}

#[test]
fn point_intersect_horizontal() {
    let u1 = Vector2::new(-1.0, 0.0);
    let u2 = Vector2::new(1.0, 0.0);
    let v1 = Vector2::new(0.0, 0.0);
    let v2 = Vector2::new(0.0, 0.0);
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.5,
            seg2_t: 0.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.5
        }
    );
}

#[test]
fn point_intersect_at_end() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = u1;
    let v2 = u1;
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 0.0
        }
    );

    // other end
    let v1 = u2;
    let v2 = u2;
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 1.0,
            seg2_t: 0.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TrueIntersect {
            seg1_t: 0.0,
            seg2_t: 1.0
        }
    );
}

#[test]
fn entirely_overlapping_same_direction() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = u1;
    let v2 = u2;
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        Overlapping {
            seg2_t0: 0.0,
            seg2_t1: 1.0
        }
    );

    // rotate both lines together
    for &angle in TEST_ROTATION_ANGLES {
        let u2 = u2.rotate_about(u1, angle);
        let v2 = v2.rotate_about(v1, angle);
        let result = line_line_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );
    }
}
#[test]
fn entirely_overlapping_opposing_direction() {
    let u1 = Vector2::new(-1.0, -1.0);
    let u2 = Vector2::new(1.0, 1.0);
    let v1 = u2;
    let v2 = u1;
    let result = line_line_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        Overlapping {
            seg2_t0: 0.0,
            seg2_t1: 1.0
        }
    );

    // flip arg order
    let result = line_line_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        Overlapping {
            seg2_t0: 0.0,
            seg2_t1: 1.0
        }
    );

    // rotate both lines together
    for &angle in TEST_ROTATION_ANGLES {
        let u2 = u2.rotate_about(u1, angle);
        let v1 = v1.rotate_about(v2, angle);
        let result = line_line_intr(u1, u2, v1, v2, 1e-5);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );
    }
}
