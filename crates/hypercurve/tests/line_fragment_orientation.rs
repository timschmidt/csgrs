mod support;

use hypercurve::{
    Contour2, ContourFragmentSet, ContourSplitMarkers, CurveCertainty, LineSeg2, LineSide, Point2,
    PredicatePolicy, Real, Segment2, SegmentSplitMarker,
};

#[test]
fn reversed_line_fragments_follow_the_integer_orientation_oracle() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (dx, dy) in [(6, 0), (0, 6), (6, 6), (-6, 6)] {
            for translated in [false, true] {
                for scale in ["1", "-3", "1e400", "1e-400"] {
                    let scale: Real = scale.parse().unwrap();
                    let origin = if translated {
                        Point2::new(Real::pi(), Real::e())
                    } else {
                        Point2::from_values(0, 0)
                    };
                    let point = |x: Real, y: Real| {
                        Point2::new((origin.x() + x) * &scale, (origin.y() + y) * &scale)
                    };
                    let integer_point = |x, y| point(Real::from(x), Real::from(y));
                    let corners = [
                        integer_point(0, 0),
                        integer_point(dx, dy),
                        integer_point(dx - dy, dy + dx),
                        integer_point(-dy, dx),
                    ];
                    let contour = Contour2::try_new(
                        (0..4)
                            .map(|i| {
                                Segment2::Line(
                                    LineSeg2::try_new(
                                        corners[i].clone(),
                                        corners[(i + 1) % 4].clone(),
                                    )
                                    .unwrap(),
                                )
                            })
                            .collect(),
                    )
                    .unwrap();
                    let mut markers = ContourSplitMarkers::with_contour_endpoints(&contour)
                        .segments()
                        .to_vec();
                    markers[0] = (0..=3)
                        .map(|i| SegmentSplitMarker {
                            segment_index: 0,
                            param: (Real::from(i) / Real::from(3)).unwrap(),
                            point: integer_point(i * dx / 3, i * dy / 3),
                        })
                        .collect();
                    let markers = ContourSplitMarkers::new(markers).unwrap();
                    let split = support::under(&policy, || {
                        ContourFragmentSet::from_split_markers(&contour, &markers)
                    })
                    .unwrap();
                    assert_eq!(split.certainty, CurveCertainty::Certified);
                    let gap: Real = "1e-40".parse().unwrap();
                    // cross((dx,dy),(-dy,dx)) = dx^2 + dy^2 > 0.
                    // Translation cancels, and a common nonzero scale squares.
                    let queries = [
                        (integer_point(-dy, dx), LineSide::Left),
                        (integer_point(dy, -dx), LineSide::Right),
                        (integer_point(0, 0), LineSide::On),
                        (integer_point(-dx, -dy), LineSide::On),
                        (integer_point(2 * dx - dy, 2 * dy + dx), LineSide::Left),
                        (
                            point(Real::from(-dy) * &gap, Real::from(dx) * &gap),
                            LineSide::Left,
                        ),
                        (
                            point(Real::from(dy) * &gap, Real::from(-dx) * &gap),
                            LineSide::Right,
                        ),
                    ];
                    for fragment in &split.value.fragments()[..3] {
                        assert_eq!(fragment.source_segment_index, 0);
                        let Segment2::Line(line) = &fragment.segment else {
                            panic!("expected line fragment");
                        };
                        let mut line = line.clone();
                        for reversals in 0..3 {
                            let rebuilt =
                                LineSeg2::try_new(line.start().clone(), line.end().clone())
                                    .unwrap();
                            // Repeat on warm shared support and scalar caches.
                            for _ in 0..2 {
                                for (query, source_side) in &queries {
                                    let expected = match (reversals % 2, source_side) {
                                        (1, LineSide::Left) => LineSide::Right,
                                        (1, LineSide::Right) => LineSide::Left,
                                        _ => *source_side,
                                    };
                                    for representation in [&line, &rebuilt] {
                                        let result = support::under(&policy, || {
                                            representation.classify_point(query)
                                        })
                                        .unwrap();
                                        assert_eq!(result.value, expected);
                                        assert_eq!(result.certainty, CurveCertainty::Certified);
                                    }
                                }
                            }
                            for endpoint in [line.start(), line.end()] {
                                let result =
                                    support::under(&policy, || line.contains_point(endpoint))
                                        .unwrap();
                                assert!(result.value);
                                assert_eq!(result.certainty, CurveCertainty::Certified);
                            }
                            line = line.reversed();
                        }
                    }
                }
            }
        }
    }
}
