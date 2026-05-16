//! Regression tests adapted from Geo and JTS-inspired Geo cases.
//!
//! The source suites exercise linear polygon topology. These tests keep the
//! same line-only geometry so the cases map directly to `hypercurve` contours
//! and regions without adding Geo concepts such as multipoints, linestring
//! clipping, or self-intersecting fill-rule behavior that `hypercurve` does not
//! currently expose.

use geo::{BooleanOps as _, Contains as _, Coord, LineString, MultiPolygon, Point, Polygon};
use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, CurvePolicy, DefaultBackend, FillRule,
    IntersectionKind, LineLineIntersection, LineSeg2, Point2, Region2, RegionPointLocation, Scalar,
    UncertaintyReason,
};

type Backend = DefaultBackend;
type HPoint = Point2<Backend>;
type HScalar = Scalar<Backend>;
type HContour = Contour2<Backend>;
type HRegion = Region2<Backend>;

fn s(value: f64) -> HScalar {
    HScalar::try_from(value).unwrap()
}

fn p(x: f64, y: f64) -> HPoint {
    HPoint::new(s(x), s(y))
}

fn vertex(x: f64, y: f64) -> BulgeVertex2<Backend> {
    BulgeVertex2::new(p(x, y), s(0.0))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn contour(coords: &[(f64, f64)]) -> HContour {
    let mut vertices: Vec<_> = coords.iter().map(|&(x, y)| vertex(x, y)).collect();
    if vertices.len() > 1 && coords.first() == coords.last() {
        vertices.pop();
    }
    Contour2::from_bulge_vertices_with_fill_rule(&vertices, FillRule::NonZero).unwrap()
}

fn region_from_rings(materials: &[&[(f64, f64)]], holes: &[&[(f64, f64)]]) -> HRegion {
    Region2::new(
        materials.iter().map(|ring| contour(ring)).collect(),
        holes.iter().map(|ring| contour(ring)).collect(),
    )
}

fn geo_polygon(ring: &[(f64, f64)]) -> Polygon<f64> {
    Polygon::new(
        LineString::from(
            ring.iter()
                .copied()
                .map(|(x, y)| Coord { x, y })
                .collect::<Vec<_>>(),
        ),
        Vec::new(),
    )
}

fn line(start: (f64, f64), end: (f64, f64)) -> LineSeg2<Backend> {
    LineSeg2::try_new(p(start.0, start.1), p(end.0, end.1)).unwrap()
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual}"
    );
}

fn assert_point_close(actual: &HPoint, expected: (f64, f64), tolerance: f64) {
    assert_close(actual.x().to_f64_approx().unwrap(), expected.0, tolerance);
    assert_close(actual.y().to_f64_approx().unwrap(), expected.1, tolerance);
}

fn assert_single_line_hit(
    first: ((f64, f64), (f64, f64)),
    second: ((f64, f64), (f64, f64)),
    expected: (f64, f64),
    tolerance: f64,
) {
    let result = line(first.0, first.1)
        .intersect_line(&line(second.0, second.1), &policy())
        .unwrap();
    let LineLineIntersection::Point { point, .. } = result else {
        panic!("expected a single line intersection point, got {result:?}");
    };
    assert_point_close(&point, expected, tolerance);
}

fn assert_no_line_hit(first: ((f64, f64), (f64, f64)), second: ((f64, f64), (f64, f64))) {
    assert_eq!(
        line(first.0, first.1)
            .intersect_line(&line(second.0, second.1), &policy())
            .unwrap(),
        LineLineIntersection::None
    );
}

fn assert_single_line_hit_exists(
    first: ((f64, f64), (f64, f64)),
    second: ((f64, f64), (f64, f64)),
) {
    let result = line(first.0, first.1)
        .intersect_line(&line(second.0, second.1), &policy())
        .unwrap();
    assert!(
        matches!(result, LineLineIntersection::Point { .. }),
        "expected a single line intersection point, got {result:?}"
    );
}

fn geo_boolean(first: &Polygon<f64>, second: &Polygon<f64>, op: BooleanOp) -> MultiPolygon<f64> {
    match op {
        BooleanOp::Union => first.union(second),
        BooleanOp::Intersection => first.intersection(second),
        BooleanOp::Difference => first.difference(second),
        BooleanOp::Xor => first.xor(second),
    }
}

fn assert_boolean_samples_match_geo(
    first: &[(f64, f64)],
    second: &[(f64, f64)],
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    let first_region = region_from_rings(&[first], &[]);
    let second_region = region_from_rings(&[second], &[]);
    let first_geo = geo_polygon(first);
    let second_geo = geo_polygon(second);
    let expected = geo_boolean(&first_geo, &second_geo, op);

    let result = first_region
        .boolean_region(&second_region, op, FillRule::NonZero, &policy())
        .unwrap();
    let Classification::Decided(result) = result else {
        panic!("expected Geo-derived boolean case to be decided, got {result:?}");
    };

    for &(x, y) in samples {
        let expected_inside = expected.contains(&Point::new(x, y));
        let actual = result.classify_point(&p(x, y), &policy());
        assert_eq!(
            actual,
            Classification::Decided(if expected_inside {
                RegionPointLocation::Inside
            } else {
                RegionPointLocation::Outside
            }),
            "sample ({x}, {y}) differed for {op:?}"
        );
    }
}

#[test]
fn geo_line_intersection_examples_match_hypercurve_topology() {
    let crossing = line((0.0, 0.0), (5.0, 5.0))
        .intersect_line(&line((0.0, 5.0), (5.0, 0.0)), &policy())
        .unwrap();
    let LineLineIntersection::Point { point, kind, .. } = crossing else {
        panic!("expected crossing point, got {crossing:?}");
    };
    assert_eq!(kind, IntersectionKind::Crossing);
    assert_point_close(&point, (2.5, 2.5), 1e-12);

    assert_no_line_hit(((0.0, 0.0), (5.0, 5.0)), ((0.0, 1.0), (5.0, 6.0)));

    let endpoint = line((0.0, 0.0), (5.0, 5.0))
        .intersect_line(&line((5.0, 5.0), (5.0, 0.0)), &policy())
        .unwrap();
    let LineLineIntersection::Point { point, kind, .. } = endpoint else {
        panic!("expected endpoint point, got {endpoint:?}");
    };
    assert_eq!(kind, IntersectionKind::Endpoint);
    assert_point_close(&point, (5.0, 5.0), 1e-12);

    let overlap = line((0.0, 0.0), (5.0, 5.0))
        .intersect_line(&line((3.0, 3.0), (6.0, 6.0)), &policy())
        .unwrap();
    let LineLineIntersection::Overlap { segment, .. } = overlap else {
        panic!("expected collinear overlap, got {overlap:?}");
    };
    assert_point_close(segment.start(), (3.0, 3.0), 1e-12);
    assert_point_close(segment.end(), (5.0, 5.0), 1e-12);
}

#[test]
fn geo_jts_line_intersection_regressions_are_classified() {
    // These cases come from Geo's JTS-inspired RobustLineIntersector regression
    // tests and stress near-endpoint and large-coordinate line intersections.
    assert_single_line_hit(
        ((163.81867067, -211.31840378), (165.9174252, -214.1665075)),
        ((2.84139601, -57.95412726), (469.59990601, -502.63851732)),
        (163.81867067, -211.31840378),
        1e-8,
    );
    assert_single_line_hit(
        (
            (-58.00593335955, -1.43739086465),
            (-513.86101637525, -457.29247388035),
        ),
        (
            (-215.22279674875, -158.65425425385),
            (-218.1208801283, -160.68343590235),
        ),
        (-215.22279674875, -158.65425425385),
        1e-8,
    );
    assert_no_line_hit(
        ((-42.0, 163.2), (21.2, 265.2)),
        ((-26.2, 188.7), (37.0, 290.7)),
    );
    assert_no_line_hit(
        ((-5.9, 163.1), (76.1, 250.7)),
        ((14.6, 185.0), (96.6, 272.6)),
    );
    assert_single_line_hit(
        (
            (305690.0434123494, 254176.46578338774),
            (305601.9999843455, 254243.19999846347),
        ),
        (
            (305689.6153764265, 254177.33102743194),
            (305692.4999844298, 254171.4999983967),
        ),
        (305690.0434123494, 254176.46578338774),
        1e-7,
    );
    assert_single_line_hit(
        (
            (588743.626135934, 4518924.610969561),
            (588732.2822865889, 4518925.4314047815),
        ),
        (
            (588739.1191384895, 4518927.235700594),
            (588731.7854614238, 4518924.578370095),
        ),
        (588733.8306132929, 4518925.319423238),
        1e-6,
    );
    assert_single_line_hit(
        (
            (588750.7429703881, 4518950.493668233),
            (588748.2060409798, 4518933.9452804085),
        ),
        (
            (588745.824857241, 4518940.742239175),
            (588748.2060437313, 4518933.9452791475),
        ),
        (588748.2060416829, 4518933.945284994),
        1e-6,
    );
    // Geo asserts a normalized robust surrogate coordinate for this
    // ill-conditioned case. `hypercurve` only needs the topological fact here:
    // the certified line relation is a single crossing, not none or overlap.
    assert_single_line_hit_exists(
        (
            (2089426.5233462777, 1180182.387733969),
            (2085646.6891757075, 1195618.7333999649),
        ),
        (
            (1889281.8148903656, 1997547.0560044837),
            (2259977.3672236, 483675.17050843034),
        ),
    );
    assert_single_line_hit(
        (
            (4348433.262114629, 5552595.478385733),
            (4348440.849387404, 5552599.272022122),
        ),
        (
            (4348433.26211463, 5552595.47838573),
            (4348440.8493874, 5552599.27202212),
        ),
        (4348440.8493874, 5552599.27202212),
        1e-6,
    );
}

#[test]
fn geo_contains_boundary_and_hole_point_cases_match_region_classification() {
    let square = region_from_rings(
        &[&[(-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]],
        &[],
    );
    assert_eq!(
        square.classify_point(&p(0.0, 0.0), &policy()),
        Classification::Decided(RegionPointLocation::Inside)
    );
    for point in [(-1.0, 1.0), (-1.0, 0.5), (-1.0, 0.0)] {
        assert_eq!(
            square.classify_point(&p(point.0, point.1), &policy()),
            Classification::Decided(RegionPointLocation::Boundary)
        );
    }
    assert_eq!(
        square.classify_point(&p(-2.0, 0.0), &policy()),
        Classification::Decided(RegionPointLocation::Outside)
    );

    let triangle = region_from_rings(&[&[(-1.0, 0.0), (0.0, 1.0), (1.0, 0.0)]], &[]);
    for point in [(-0.75, 1.0), (-0.5, 0.5), (0.0, 1.0), (0.75, 1.0)] {
        let expected = if point == (-0.5, 0.5) || point == (0.0, 1.0) {
            RegionPointLocation::Boundary
        } else {
            RegionPointLocation::Outside
        };
        assert_eq!(
            triangle.classify_point(&p(point.0, point.1), &policy()),
            Classification::Decided(expected)
        );
    }

    let hollow_ccw_outer = region_from_rings(
        &[&[(-2.0, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0)]],
        &[&[(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)]],
    );
    let hollow_cw_outer = region_from_rings(
        &[&[(-2.0, -2.0), (-2.0, 2.0), (2.0, 2.0), (2.0, -2.0)]],
        &[&[(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]],
    );
    for hollow in [hollow_ccw_outer, hollow_cw_outer] {
        assert_eq!(
            hollow.classify_point(&p(0.0, 0.0), &policy()),
            Classification::Decided(RegionPointLocation::Outside)
        );
        assert_eq!(
            hollow.classify_point(&p(1.5, 0.0), &policy()),
            Classification::Decided(RegionPointLocation::Inside)
        );
    }
}

#[test]
fn geo_boolean_issue_867_shared_vertex_triangles_are_explicitly_deferred() {
    let first = &[
        (17.724912058920285, -16.37118892052372),
        (18.06452454246989, -17.693907532504),
        (19.09389292605319, -17.924001641855178),
    ];
    let second = &[
        (17.576085274796423, -15.791540153598898),
        (17.19432983818328, -17.499393422066746),
        (18.06452454246989, -17.693907532504),
    ];
    let first_region = region_from_rings(&[first], &[]);
    let second_region = region_from_rings(&[second], &[]);

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        assert_eq!(
            first_region
                .boolean_region(&second_region, op, FillRule::NonZero, &policy())
                .unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported),
            "Geo issue 867 should be an explicit branch-vertex deferral for {op:?}"
        );
    }
}

#[test]
fn geo_boolean_triangle_pair_matches_geo_samples() {
    let first = &[
        (204.0, 287.0),
        (203.69670020700084, 288.2213844497616),
        (200.38308697914755, 288.338793163584),
    ];
    let second = &[
        (210.0, 290.0),
        (204.07584923592933, 288.2701221108328),
        (212.24082541367974, 285.47846008552216),
    ];
    let samples = &[
        (203.8, 288.1),
        (204.2, 288.25),
        (210.0, 289.0),
        (211.0, 286.5),
        (201.0, 287.2),
    ];

    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::Xor,
    ] {
        assert_boolean_samples_match_geo(first, second, op, samples);
    }
}

#[test]
fn geo_unary_union_third_triangle_branch_case_is_explicitly_deferred() {
    let first = &[
        (204.0, 287.0),
        (203.69670020700084, 288.2213844497616),
        (200.38308697914755, 288.338793163584),
    ];
    let second = &[
        (210.0, 290.0),
        (204.07584923592933, 288.2701221108328),
        (212.24082541367974, 285.47846008552216),
    ];
    let third = &[
        (211.0, 292.0),
        (202.07584923592933, 288.2701221108328),
        (212.24082541367974, 285.47846008552216),
        (210.0, 290.0),
    ];

    let first_region = region_from_rings(&[first], &[]);
    let second_region = region_from_rings(&[second], &[]);
    let third_region = region_from_rings(&[third], &[]);
    let Classification::Decided(partial) = first_region
        .boolean_region(
            &second_region,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap()
    else {
        panic!("expected first pairwise union to be decided");
    };

    let result = partial
        .boolean_region(
            &third_region,
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert_eq!(
        result,
        Classification::Uncertain(UncertaintyReason::Boundary)
    );

    let geo_expected = geo_polygon(first)
        .union(&geo_polygon(second))
        .union(&geo_polygon(third));
    assert!(geo_expected.contains(&Point::new(210.0, 289.0)));
}

#[test]
fn geo_issue_913_polygon_union_pair_is_explicitly_deferred() {
    let first = &[
        (204.0, 287.0),
        (206.69670020700084, 288.2213844497616),
        (200.38308697914755, 288.338793163584),
    ];
    let second = &[
        (210.0, 290.0),
        (204.07584923592933, 288.2701221108328),
        (212.24082541367974, 285.47846008552216),
    ];

    let result = region_from_rings(&[first], &[])
        .boolean_region(
            &region_from_rings(&[second], &[]),
            BooleanOp::Union,
            FillRule::NonZero,
            &policy(),
        )
        .unwrap();
    assert_eq!(
        result,
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
