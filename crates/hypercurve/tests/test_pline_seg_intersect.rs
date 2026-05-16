use cavalier_contours::{
    core::math::{Vector2, bulge_from_angle},
    polyline::{PlineSegIntr::*, PlineVertex, pline_seg_intr},
};
use std::f64::consts::FRAC_PI_2;

macro_rules! assert_case_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (NoIntersect, NoIntersect) => {}
            (TangentIntersect { point: a1 }, TangentIntersect { point: a2 })
            | (OneIntersect { point: a1 }, OneIntersect { point: a2 })
                if a1.fuzzy_eq(a2) => {}
            (
                TwoIntersects {
                    point1: a1,
                    point2: b1,
                },
                TwoIntersects {
                    point1: a2,
                    point2: b2,
                },
            )
            | (
                OverlappingLines {
                    point1: a1,
                    point2: b1,
                },
                OverlappingLines {
                    point1: a2,
                    point2: b2,
                },
            )
            | (
                OverlappingArcs {
                    point1: a1,
                    point2: b1,
                },
                OverlappingArcs {
                    point1: a2,
                    point2: b2,
                },
            ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
            _ => panic!(
                "intersect cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        };
    };
}

#[test]
fn arc_line_no_intersect() {
    let v1 = PlineVertex::new(0.0, 0.0, 1.0);
    let v2 = PlineVertex::new(2.0, 0.0, 0.0);
    let u1 = PlineVertex::new(0.0, 1.0, 0.0);
    let u2 = PlineVertex::new(2.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn line_arc_no_intersect() {
    let v1 = PlineVertex::new(0.0, 1.0, 0.0);
    let v2 = PlineVertex::new(2.0, 3.0, 0.0);
    let u1 = PlineVertex::new(0.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn overlapping_lines() {
    let v1 = PlineVertex::new(3.0, 3.0, 0.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 0.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingLines {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(2.0, 2.0)
        }
    );
}

#[test]
fn overlapping_lines_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 0.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(2.0, 2.0, 0.0);
    let u2 = PlineVertex::new(1.0, 1.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingLines {
            point1: Vector2::new(2.0, 2.0),
            point2: Vector2::new(1.0, 1.0)
        }
    );
}

#[test]
fn overlapping_same_arcs() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn overlapping_same_arcs_reverse_dir() {
    let v1 = PlineVertex::new(3.0, 3.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn arc_arc_end_points_touch() {
    let v1 = PlineVertex::new(3.0, 3.0, 1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn arc_arc_end_points_touch_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, -1.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );

    // reverse parameter order should yield the same result
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );

    // changing direction of arc2 should yield the same result BUT point1/point2 ordered according to
    // second segment direction
    let u1 = PlineVertex::new(3.0, 3.0, -1.0);
    let u2 = PlineVertex::new(1.0, 1.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(3.0, 3.0),
            point2: Vector2::new(1.0, 1.0)
        }
    );
}

#[test]
fn arc2_within_arc1_overlapping() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(2.0, 0.0, bulge);
    let u2 = PlineVertex::new(3.0, 1.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc1_within_arc2_overlapping() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(2.0, 0.0, bulge);
    let u2 = PlineVertex::new(3.0, 1.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc2_within_arc1_overlapping_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(3.0, 1.0, -bulge);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}

#[test]
fn arc1_within_arc2_overlapping_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(3.0, 1.0, -bulge);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_flipped() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc2_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 2.0, -1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc2_reverse_dir_flipped() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 2.0, -1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc1_reverse_dir() {
    let v1 = PlineVertex::new(3.0, 1.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc1_reverse_dir_flipped() {
    let v1 = PlineVertex::new(3.0, 1.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}

#[test]
fn arc_arc_opposite_direction_touch_at_ends_bug() {
    // This test case reproduces the bug where arcs have the same radius and center but opposite
    // directions and only touch at the end points.
    // The bug was that when same_direction_arcs = false, the code would return u1.pos()
    // as the intersection point, but after direction adjustment, u1.pos() is actually
    // the END of arc2, not the start. The actual intersection should be at u2.pos().
    //
    // Original issue that found it: https://github.com/jbuckmccready/cavalier_contours/issues/42

    // Arc1
    let v1 = PlineVertex::new(-189.0, -196.91384910249, 0.553407781718062);
    let v2 = PlineVertex::new(-170.999999999999, -225.631646989572, -0.553407781718061);

    // Arc2
    let u1 = PlineVertex::new(-153.0, -196.91384910249, -0.553407781718061);
    let u2 = PlineVertex::new(-171.0, -225.631646989571, -0.553407781718061);

    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);

    // The arcs should intersect at u2.pos() (where arc1 and arc2 ends),
    // NOT at u1.pos() (which is ~34 units away from the actual intersection)
    assert_case_eq!(
        result,
        OneIntersect {
            point: Vector2::new(-171.0, -225.631646989571) // u2.pos()
        }
    );

    // reverse parameter order should yield the same result
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OneIntersect {
            point: Vector2::new(-171.0, -225.631646989571) // u2.pos()
        }
    );

    // changing direction of arc2 should yield the same result
    let u1 = PlineVertex::new(-171.0, -225.631646989571, 0.553407781718062);
    let u2 = PlineVertex::new(-153.0, -196.91384910249, -0.553407781718061);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OneIntersect {
            point: Vector2::new(-171.0, -225.631646989571)
        }
    );
}
