//! Exact Boolean regressions derived from release-scale PCB process checks.
//!
//! A solder-paste image commonly contains hundreds of disjoint curved
//! apertures, each strictly contained by one member of a much larger disjoint
//! copper image. Treating the two images as one all-to-all Boolean exposed a
//! severe performance cliff in the unified region kernel. Keep both a routine
//! reduced case and the original Easyduino-scale topology in the corpus.

mod support;
use hypercurve::{BulgeVertex2, Contour2, CurveRegion2, Point2, PredicatePolicy, Real};
use support::IntoCertified;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn subdivided_square(
    center_x: i64,
    center_y: i64,
    half_extent: i64,
    segments_per_side: i64,
) -> Contour2 {
    assert_eq!((2 * half_extent) % segments_per_side, 0);
    let step = 2 * half_extent / segments_per_side;
    let mut vertices = Vec::new();
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x - half_extent + index * step,
                center_y - half_extent,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x + half_extent,
                center_y - half_extent + index * step,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x + half_extent - index * step,
                center_y + half_extent,
            ),
            Real::zero(),
        ));
    }
    for index in 0..segments_per_side {
        vertices.push(BulgeVertex2::new(
            point(
                center_x - half_extent,
                center_y + half_extent - index * step,
            ),
            Real::zero(),
        ));
    }
    Contour2::from_bulge_vertices(&vertices)
        .expect("subdivided exact square forms a closed contour")
}

fn pcb_containment_fixture(
    cover_count: usize,
    subject_count: usize,
) -> (CurveRegion2, CurveRegion2) {
    assert!(subject_count <= cover_count);
    let centers = (0..cover_count).map(|index| {
        let column = i64::try_from(index % 32).expect("fixture column fits i64");
        let row = i64::try_from(index / 32).expect("fixture row fits i64");
        (column * 100, row * 100)
    });
    let cover = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let subject = centers
        .take(subject_count)
        .map(|(x, y)| subdivided_square(x, y, 28, 14))
        .collect::<Vec<_>>();
    let policy = PredicatePolicy::STRICT;
    (
        crate::support::under(&policy, || {
            CurveRegion2::try_from_native_contours(cover, Vec::new())
        })
        .expect("disjoint cover components form an exact region")
        .into_value(),
        crate::support::under(&policy, || {
            CurveRegion2::try_from_native_contours(subject, Vec::new())
        })
        .expect("disjoint subject components form an exact region")
        .into_value(),
    )
}

fn assert_exact_containment_difference_is_empty(cover_count: usize, subject_count: usize) {
    let (cover, subject) = pcb_containment_fixture(cover_count, subject_count);
    let result = crate::support::under(&PredicatePolicy::STRICT, || {
        subject.boolean_region(&cover, hypercurve::BooleanOp::Difference)
    })
    .expect("PCB containment difference must decide exactly")
    .value;
    assert!(result.is_empty());

    for point in [point(0, 0), point(100, 0)] {
        assert_eq!(
            crate::support::under(&PredicatePolicy::STRICT, || cover
                .classify_point(&point.clone().into()))
            .expect("cover point classification must decide")
            .into_value(),
            hypercurve::RegionPointLocation::Inside,
        );
    }
}

#[test]
fn pcb_process_image_containment_regression_decides_exactly() {
    assert_exact_containment_difference_is_empty(64, 24);
}

#[test]
fn easyduino_scale_process_image_containment_corpus() {
    // Matches the order of magnitude observed in the Easyduino Nano release:
    // 138 paste components against 479 front-copper components.
    assert_exact_containment_difference_is_empty(479, 138);
}

#[test]
fn easyduino_uno_scale_process_image_with_holes_corpus() {
    // Matches the Easyduino Uno front-copper topology that originally took
    // about a minute to classify: 1,098 material components, 455 holes, and
    // 136 paste apertures. The apertures use material components after the
    // holed prefix so their exact difference is empty.
    let centers = (0..1_098).map(|index| {
        let column = i64::from(index % 32);
        let row = i64::from(index / 32);
        (column * 100, row * 100)
    });
    let materials = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let holes = centers
        .clone()
        .take(455)
        .map(|(x, y)| subdivided_square(x, y, 8, 8))
        .collect::<Vec<_>>();
    let subjects = centers
        .skip(455)
        .take(136)
        .map(|(x, y)| subdivided_square(x, y, 28, 14))
        .collect::<Vec<_>>();
    let policy = PredicatePolicy::STRICT;
    let cover = crate::support::under(&policy, || {
        CurveRegion2::try_from_native_contours(materials, holes)
    })
    .expect("holed front-copper corpus forms an exact region")
    .into_value();
    let subject = crate::support::under(&policy, || {
        CurveRegion2::try_from_native_contours(subjects, Vec::new())
    })
    .expect("paste corpus forms an exact region")
    .into_value();

    let result = crate::support::under(&policy, || {
        subject.boolean_region(&cover, hypercurve::BooleanOp::Difference)
    })
    .expect("holed PCB containment difference must decide exactly")
    .value;

    assert!(result.is_empty());
}

#[test]
fn pcb_process_image_hole_ownership_culls_sparse_materials() {
    let centers = (0..512).map(|index| {
        let column = i64::from(index % 32);
        let row = i64::from(index / 32);
        (column * 100, row * 100)
    });
    let materials = centers
        .clone()
        .map(|(x, y)| subdivided_square(x, y, 30, 15))
        .collect::<Vec<_>>();
    let holes = centers
        .take(128)
        .map(|(x, y)| subdivided_square(x, y, 10, 10))
        .collect::<Vec<_>>();
    let region = CurveRegion2::try_from_native_contours(materials, holes)
        .expect("sparse material and hole loops form an exact region");

    let profiles = region
        .boundary_profiles()
        .expect("PCB profile ownership must complete exactly");

    assert_eq!(profiles.len(), 512);
    assert_eq!(
        profiles
            .iter()
            .map(|profile| profile.holes().len())
            .sum::<usize>(),
        128
    );
    assert!(profiles.iter().all(|profile| profile.holes().len() <= 1));
}

fn ratio(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn exact_point(x: Real, y: Real) -> hypercurve::CurvePoint2 {
    Point2::new(x, y).into()
}

/// Rotates a region by `degrees` with the exact sine and cosine of the angle.
fn rotated(region: &CurveRegion2, degrees: i64) -> CurveRegion2 {
    let radians = (Real::from(degrees) * Real::pi() / Real::from(180)).unwrap();
    let cosine = radians.clone().cos();
    let sine = radians.sin();
    region
        .transform_affine(
            &cosine,
            &-sine.clone(),
            &sine,
            &cosine,
            &Real::zero(),
            &Real::zero(),
        )
        .unwrap()
}

fn translated(region: &CurveRegion2, x: Real, y: Real) -> CurveRegion2 {
    region
        .transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &x,
            &y,
        )
        .unwrap()
}

/// The rounded-rectangle land of Easyduino STM32F103 pad U2:8, placed through
/// the same two 135-degree rotations as the board, so that every corner
/// center carries the radicals of both rotations.
fn rotated_rounded_pad() -> CurveRegion2 {
    use hypercurve::{CircularArc2, Curve2, CurvePath2, FillRule, LineSeg2};
    let (width, height, radius) = (ratio(59, 40), ratio(3, 10), ratio(3, 40));
    let right = &width - &radius;
    let top = &height - &radius;
    let p = |x: &Real, y: &Real| Point2::new(x.clone(), y.clone());
    let zero = Real::zero();
    let line = |start: Point2, end: Point2| Curve2::from(LineSeg2::try_new(start, end).unwrap());
    let arc = |start: Point2, end: Point2, center: Point2| {
        Curve2::from(CircularArc2::try_from_center(start, end, center, false).unwrap())
    };
    let path = CurvePath2::try_new(vec![
        line(p(&radius, &zero), p(&right, &zero)),
        arc(p(&right, &zero), p(&width, &radius), p(&right, &radius)),
        line(p(&width, &radius), p(&width, &top)),
        arc(p(&width, &top), p(&right, &height), p(&right, &top)),
        line(p(&right, &height), p(&radius, &height)),
        arc(p(&radius, &height), p(&zero, &top), p(&radius, &top)),
        line(p(&zero, &top), p(&zero, &radius)),
        arc(p(&zero, &radius), p(&radius, &zero), p(&radius, &radius)),
    ])
    .unwrap();
    let local = CurveRegion2::try_from_boundary_paths(&[path], FillRule::EvenOdd).unwrap();
    let local = translated(&local, -ratio(59, 80), -ratio(3, 20));
    let local = translated(&rotated(&local, 135), ratio(-333, 80), ratio(3, 4));
    translated(
        &rotated(&local, 135),
        ratio(137_440_551, 1_000_000),
        ratio(20_069_443, 200_000),
    )
}

/// Builds one filled region from every loop of `regions`, keeping their
/// roles and using the nonzero rule, as an aggregate copper image does.
fn compound(regions: &[(&CurveRegion2, bool)]) -> hypercurve::ExactCurveResult<CurveRegion2> {
    use hypercurve::{CurveRegionLoopRole, FillRule};
    let mut paths = Vec::new();
    let mut roles = Vec::new();
    for (region, negative) in regions {
        paths.extend(region.boundary_paths()?);
        roles.extend(
            region
                .loop_roles()?
                .into_iter()
                .map(|role| match (role, negative) {
                    (CurveRegionLoopRole::Material, true) => CurveRegionLoopRole::Hole,
                    (CurveRegionLoopRole::Hole, true) => CurveRegionLoopRole::Material,
                    (role, false) => role,
                }),
        );
    }
    let rules = vec![FillRule::NonZero; paths.len()];
    CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths, &roles, &rules)
}

/// An exact planar affine map `(x, y) -> (a x + b y + e, c x + d y + f)`.
type AffineMap = [Real; 6];

fn identity_map() -> AffineMap {
    [
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::zero(),
    ]
}

fn mapped_region(region: &CurveRegion2, map: &AffineMap) -> CurveRegion2 {
    let [a, b, c, d, e, f] = map;
    region.transform_affine(a, b, c, d, e, f).unwrap()
}

fn mapped_point(map: &AffineMap, x: &Real, y: &Real) -> hypercurve::CurvePoint2 {
    let [a, b, c, d, e, f] = map;
    exact_point(a * x + b * y + e, c * x + d * y + f)
}

/// The route of Easyduino STM32F103 segment 262: a 45-degree 0.2 mm trace
/// with round caps, whose edges lie on `x + y = 236.02 ± sqrt(2)/10`.
fn board_route() -> CurveRegion2 {
    use hypercurve::{Curve2, CurvePath2, LineSeg2, OffsetCap, OffsetCornerStyle2};
    let centerline = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(
            Point2::new(ratio(279, 2), ratio(2413, 25)),
            Point2::new(ratio(1439, 10), ratio(2303, 25)),
        )
        .unwrap(),
    )])
    .unwrap();
    CurveRegion2::stroke_path(
        &centerline,
        ratio(1, 10),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
    )
    .unwrap()
}

/// A circular zone outline. Both ends of the zone's fragment of the route
/// edge are then line-circle crossings, so the fragment's contacts carry
/// radicals independent of those of the route edge itself.
fn circular_zone_outline() -> CurveRegion2 {
    CurveRegion2::try_from_native_contours(
        vec![
            Contour2::from_bulge_vertices(&[
                BulgeVertex2::new(Point2::new(ratio(701, 5), ratio(385, 4)), Real::one()),
                BulgeVertex2::new(Point2::new(ratio(699, 5), ratio(385, 4)), Real::one()),
            ])
            .unwrap(),
        ],
        Vec::new(),
    )
    .unwrap()
}

fn certified_area(region: &CurveRegion2) -> f64 {
    region
        .filled_area()
        .unwrap()
        .expect("bounded copper has a finite area")
        .to_f64_lossy()
        .unwrap()
}

/// Composes a pad, a route and a zero-clearance zone cut by that route, in
/// three orders, the first under strict, approximate and again strict policy.
///
/// The zone keeps a fragment of a route edge, so wherever the pad's circle
/// crosses that edge inside the zone the aggregate meets the crossing twice:
/// once on the route edge and once on the fragment, each with its own
/// square root. Returns the strict aggregate in authored order.
fn assert_concurrent_contacts_compose(
    pad: &CurveRegion2,
    route: &CurveRegion2,
    outline: &CurveRegion2,
    map: &AffineMap,
    samples: &[(Real, Real, hypercurve::RegionPointLocation)],
) -> CurveRegion2 {
    use hypercurve::CurveCertainty;
    let zone = compound(&[(outline, false), (route, true)])
        .expect("a zero-clearance zone cut by one route is exact");
    let orders = [
        [pad, route, &zone],
        [&zone, route, pad],
        [route, &zone, pad],
    ];
    let mut aggregates = Vec::new();
    for (index, order) in orders.iter().enumerate() {
        let policies: &[PredicatePolicy] = if index == 0 {
            &[
                PredicatePolicy::STRICT,
                PredicatePolicy::APPROXIMATE_512,
                PredicatePolicy::STRICT,
            ]
        } else {
            &[PredicatePolicy::STRICT]
        };
        for policy in policies {
            let outcome = crate::support::under(policy, || {
                compound(&[(order[0], false), (order[1], false), (order[2], false)])
            })
            .expect("concurrent contacts on one circle and one line must decide exactly");
            // Permission to approximate is not consumption: nothing needs it.
            assert_eq!(outcome.certainty, CurveCertainty::Certified);
            aggregates.push(outcome.value);
        }
    }
    // The zone and the route together cover the outline, so the aggregate is
    // the union of pad, route and outline. That union never forms the zone's
    // fragment of the route edge.
    let union = crate::support::under(&PredicatePolicy::STRICT, || {
        pad.boolean_region(route, hypercurve::BooleanOp::Union)?
            .boolean_region(outline, hypercurve::BooleanOp::Union)
    })
    .unwrap()
    .into_certified();
    let expected_area = certified_area(&union);
    for aggregate in &aggregates {
        assert!((certified_area(aggregate) - expected_area).abs() < 1e-12);
        for (x, y, expected) in samples {
            assert_eq!(
                crate::support::under(&PredicatePolicy::STRICT, || {
                    aggregate.classify_point(&mapped_point(map, x, y))
                })
                .unwrap()
                .into_certified(),
                *expected,
                "({x:?}, {y:?})"
            );
        }
    }
    aggregates.swap_remove(0)
}

fn board_samples() -> Vec<(Real, Real, hypercurve::RegionPointLocation)> {
    use hypercurve::RegionPointLocation::{Inside, Outside};
    // Pad interior outside the zone, route interior, zone interior above
    // the route edge, and empty board between them.
    vec![
        (ratio(13_985, 100), ratio(9_687, 100), Inside),
        (ratio(1_417, 10), ratio(9_432, 100), Inside),
        (ratio(14_015, 100), ratio(962, 10), Inside),
        (ratio(141, 1), ratio(96, 1), Outside),
    ]
}

#[test]
fn concurrent_circle_contacts_on_one_shared_line_support_decide_exactly() {
    // Easyduino STM32F103 copper layer 0. A 45-degree route edge crosses the
    // corner arc of pad U2:8 inside a zero-clearance zone cut by that route.
    // The two contacts cannot be proved equal from their coordinates. They
    // lie on one circle and one support line, which meet in at most two
    // points, and the side of the circle center's projection on the line
    // selects the same one.
    let map = identity_map();
    assert_concurrent_contacts_compose(
        &rotated_rounded_pad(),
        &board_route(),
        &circular_zone_outline(),
        &map,
        &board_samples(),
    );
}

#[test]
fn concurrent_circle_contacts_survive_rational_rotation_and_reflection() {
    // Rotation by the 3-4-5 angle and a reflection, applied to the inputs
    // before the zone is cut, so the zone still borrows the route's edge
    // support. Both preserve area.
    let reference = certified_area(
        &crate::support::under(&PredicatePolicy::STRICT, || {
            rotated_rounded_pad()
                .boolean_region(&board_route(), hypercurve::BooleanOp::Union)?
                .boolean_region(&circular_zone_outline(), hypercurve::BooleanOp::Union)
        })
        .unwrap()
        .into_certified(),
    );
    for map in [
        [
            ratio(3, 5),
            ratio(-4, 5),
            ratio(4, 5),
            ratio(3, 5),
            ratio(-7, 2),
            ratio(11, 3),
        ],
        [
            ratio(3, 5),
            ratio(4, 5),
            ratio(4, 5),
            ratio(-3, 5),
            Real::zero(),
            ratio(1, 7),
        ],
    ] {
        let aggregate = assert_concurrent_contacts_compose(
            &mapped_region(&rotated_rounded_pad(), &map),
            &mapped_region(&board_route(), &map),
            &mapped_region(&circular_zone_outline(), &map),
            &map,
            &board_samples(),
        );
        assert!((certified_area(&aggregate) - reference).abs() < 1e-12);
    }
}

#[test]
fn concurrent_circle_contacts_survive_independent_maps_after_cutting() {
    use hypercurve::{BooleanOp, CurveCertainty};

    let pad = rotated_rounded_pad();
    let route = board_route();
    let outline = circular_zone_outline();
    let zone = compound(&[(&outline, false), (&route, true)]).unwrap();
    for map in [
        [
            ratio(3, 5),
            ratio(-4, 5),
            ratio(4, 5),
            ratio(3, 5),
            ratio(-7, 2),
            ratio(11, 3),
        ],
        [
            ratio(3, 5),
            ratio(4, 5),
            ratio(4, 5),
            ratio(-3, 5),
            Real::zero(),
            ratio(1, 7),
        ],
    ] {
        // The cut and its source are transformed separately. Their supporting
        // lines have equal defining points but independent allocations.
        let pad = mapped_region(&pad, &map);
        let route = mapped_region(&route, &map);
        let zone = mapped_region(&zone, &map);
        let outline = mapped_region(&outline, &map);
        let expected = crate::support::under(&PredicatePolicy::STRICT, || {
            pad.boolean_region(&route, BooleanOp::Union)?
                .boolean_region(&outline, BooleanOp::Union)
        })
        .unwrap()
        .into_certified();
        for order in [
            [&pad, &route, &zone],
            [&zone, &route, &pad],
            [&route, &zone, &pad],
        ] {
            let result = crate::support::under(&PredicatePolicy::STRICT, || {
                compound(&[(order[0], false), (order[1], false), (order[2], false)])
            })
            .expect("independent maps must preserve the shared support proof");
            assert_eq!(result.certainty, CurveCertainty::Certified);
            // Set equality, including boundary ownership, and reuse as an
            // operand. Area and sampled membership alone are insufficient.
            let difference = crate::support::under(&PredicatePolicy::STRICT, || {
                result.value.boolean_region(&expected, BooleanOp::Xor)
            })
            .unwrap()
            .into_certified();
            assert!(difference.is_empty());
        }
    }
}

#[test]
fn distinct_crossings_of_one_shared_circle_and_line_stay_distinct() {
    // A 45-degree route whose edge `x - y = 43.66 + sqrt(2)/10` lies between
    // the chord and the arc of the pad's lower-right corner, so it crosses
    // that corner arc twice inside the zone. Each crossing has a contact on
    // the route edge and one on the zone fragment. Contacts at one crossing
    // merge; the two crossings, on opposite sides of the center's
    // projection, stay distinct.
    use hypercurve::RegionPointLocation::{Inside, Outside};
    use hypercurve::{Curve2, CurvePath2, LineSeg2, OffsetCap, OffsetCornerStyle2};
    let centerline = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(
            Point2::new(ratio(139, 1), ratio(4_767, 50)),
            Point2::new(ratio(141, 1), ratio(4_867, 50)),
        )
        .unwrap(),
    )])
    .unwrap();
    let route = CurveRegion2::stroke_path(
        &centerline,
        ratio(1, 10),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
    )
    .unwrap();
    assert_concurrent_contacts_compose(
        &rotated_rounded_pad(),
        &route,
        &circular_zone_outline(),
        &identity_map(),
        &[
            // Pad interior away from the zone and the route, route interior
            // beyond the pad and the zone, zone interior, and empty board.
            (ratio(13_985, 100), ratio(9_740, 100), Inside),
            (ratio(1_408, 10), ratio(9_714, 100), Inside),
            (ratio(14_010, 100), ratio(9_630, 100), Inside),
            (ratio(141, 1), ratio(96, 1), Outside),
        ],
    );
}

#[test]
fn nearly_tangent_crossings_of_one_shared_circle_and_line_stay_distinct() {
    // The route edge `x - y = c + sqrt(2)/10` misses tangency to the pad's
    // corner circle by about 1.3e-9 in `c`, so it crosses the corner arc
    // twice, about 2.4e-5 apart, inside the zone. The matching contacts on
    // the route edge and on the zone fragment merge at each crossing; the
    // two crossings stay distinct.
    use hypercurve::RegionPointLocation::{Inside, Outside};
    use hypercurve::{Curve2, CurvePath2, LineSeg2, OffsetCap, OffsetCornerStyle2};
    let offset = ratio(43_682_144_613, 1_000_000_000);
    let centerline = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(
            Point2::new(ratio(139, 1), ratio(139, 1) - &offset),
            Point2::new(ratio(141, 1), ratio(141, 1) - &offset),
        )
        .unwrap(),
    )])
    .unwrap();
    let route = CurveRegion2::stroke_path(
        &centerline,
        ratio(1, 10),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
    )
    .unwrap();
    assert_concurrent_contacts_compose(
        &rotated_rounded_pad(),
        &route,
        &circular_zone_outline(),
        &identity_map(),
        &[
            // Pad interior away from the zone and the route, route interior
            // beyond the pad and the zone, zone interior, and empty board.
            (ratio(13_985, 100), ratio(9_740, 100), Inside),
            (ratio(1_408, 10), ratio(9_712, 100), Inside),
            (ratio(14_015, 100), ratio(9_615, 100), Inside),
            (ratio(141, 1), ratio(96, 1), Outside),
        ],
    );
}

#[test]
fn tangent_contacts_of_one_shared_circle_and_line_decide_exactly() {
    // A route whose edge `x - y = c + (339/80) sqrt(2)` is exactly tangent to
    // the pad's corner circle at a point inside the zone. The route edge and
    // the zone fragment both touch the circle there. These contacts are
    // decided without the circle-and-line branch test; adding it must not
    // change that outcome.
    use hypercurve::RegionPointLocation::{Inside, Outside};
    use hypercurve::{Curve2, CurvePath2, LineSeg2, OffsetCap, OffsetCornerStyle2};
    let offset = ratio(9_457_709, 250_000);
    let centerline = CurvePath2::try_new(vec![Curve2::from(
        LineSeg2::try_new(
            Point2::new(ratio(136, 1), ratio(136, 1) - &offset),
            Point2::new(ratio(144, 1), ratio(144, 1) - &offset),
        )
        .unwrap(),
    )])
    .unwrap();
    let route = CurveRegion2::stroke_path(
        &centerline,
        ratio(339, 80),
        &OffsetCornerStyle2::Round,
        OffsetCap::Round,
    )
    .unwrap();
    assert_concurrent_contacts_compose(
        &rotated_rounded_pad(),
        &route,
        &circular_zone_outline(),
        &identity_map(),
        &[
            (ratio(14_015, 100), ratio(9_615, 100), Inside),
            (ratio(150, 1), ratio(96, 1), Outside),
        ],
    );
}
