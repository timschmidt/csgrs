use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BezierParallelFragment2, BezierParameter2, BezierParameterRange2, BezierSplitFragment2,
    BezierSubcurve2, BooleanOp, BulgeVertex2, CircularArc2, Classification, Contour2, CubicBezier2,
    Curve2, CurveBoundaryInteriorSide2, CurveContext, CurvePath2, CurveRegion2,
    CurveRegionBoundaryLoop2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2,
    RationalBezier2, Real,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn decided<T>(classification: Classification<T>) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("benchmark construction is uncertain: {reason:?}")
        }
    }
}

fn rectangle(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(min_x, min_y), Real::zero()),
        BulgeVertex2::new(point(max_x, min_y), Real::zero()),
        BulgeVertex2::new(point(max_x, max_y), Real::zero()),
        BulgeVertex2::new(point(min_x, max_y), Real::zero()),
    ])
    .unwrap()
}

fn circle(center_x: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(center_x - 2, 0), Real::one()),
        BulgeVertex2::new(point(center_x + 2, 0), Real::one()),
    ])
    .unwrap()
}

fn capsule(center_x: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(point(center_x - 3, -2), Real::zero()),
        BulgeVertex2::new(point(center_x + 3, -2), Real::one()),
        BulgeVertex2::new(point(center_x + 3, 2), Real::zero()),
        BulgeVertex2::new(point(center_x - 3, 2), Real::one()),
    ])
    .unwrap()
}

fn elevated_circle(center_x: i32, policy: &CurveContext) -> CurveRegion2 {
    let left = point(center_x - 2, 0);
    let right = point(center_x + 2, 0);
    let arcs = [
        CircularArc2::from_bulge(left.clone(), right.clone(), Real::one()).unwrap(),
        CircularArc2::from_bulge(right, left, Real::one()).unwrap(),
    ];
    let mut curves = Vec::with_capacity(4);
    for arc in &arcs {
        for span in arc
            .rational_bezier_decomposition(policy)
            .unwrap()
            .into_value()
            .spans()
        {
            curves.push(Curve2::from(
                RationalBezier2::from(span.curve().clone())
                    .elevated_to_degree(3)
                    .unwrap(),
            ));
        }
    }
    CurveRegion2::try_from_boundary_paths(
        &[CurvePath2::try_new_with_policy(curves, policy)
            .unwrap()
            .into_value()],
        policy,
    )
    .unwrap()
    .into_value()
}

fn rectangle_path(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> CurvePath2 {
    let points = [
        point(min_x, min_y),
        point(max_x, min_y),
        point(max_x, max_y),
        point(min_x, max_y),
    ];
    CurvePath2::try_new(
        (0..points.len())
            .map(|index| {
                Curve2::from(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect(),
    )
    .unwrap()
}

fn exact_parameter(value: i32, policy: &CurveContext) -> BezierParameter2 {
    decided(BezierParameter2::exact(Real::from(value), policy).unwrap())
}

fn analytic_parallel_fragment(
    start: Point2,
    midpoint: Point2,
    end: Point2,
    distance: i32,
    reversed: bool,
    policy: &CurveContext,
) -> BezierSplitFragment2 {
    let (start_parameter, end_parameter) = if reversed { (1, 0) } else { (0, 1) };
    let range = decided(
        BezierParameterRange2::try_new(
            exact_parameter(start_parameter, policy),
            exact_parameter(end_parameter, policy),
            policy,
        )
        .unwrap(),
    );
    let parallel = QuadraticBezier2::new(start, midpoint, end)
        .parallel_left(Real::from(distance))
        .unwrap();
    BezierSplitFragment2::AnalyticParallel(decided(
        BezierParallelFragment2::try_new(parallel, range, policy).unwrap(),
    ))
}

fn analytic_square(min_x: i32, max_x: i32, policy: &CurveContext) -> CurveRegion2 {
    let midpoint_x = (min_x + max_x) / 2;
    let edges = [
        (point(min_x, 0), point(midpoint_x, 0), point(max_x, 0)),
        (point(max_x, 0), point(max_x, 2), point(max_x, 4)),
        (point(max_x, 4), point(midpoint_x, 4), point(min_x, 4)),
        (point(min_x, 4), point(min_x, 2), point(min_x, 0)),
    ];
    let fragments = edges
        .into_iter()
        .map(|(start, midpoint, end)| {
            analytic_parallel_fragment(start, midpoint, end, 0, false, policy)
        })
        .collect();
    CurveRegion2::try_new_with_loop_topology(
        vec![CurveRegionBoundaryLoop2::new(fragments, policy).unwrap()],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

fn materialized_line(start: Point2, end: Point2, policy: &CurveContext) -> BezierSplitFragment2 {
    let midpoint = start.lerp(
        &end,
        (Real::one() / Real::from(2_u8)).expect("one half is represented"),
    );
    BezierSplitFragment2::Materialized {
        start: exact_parameter(0, policy),
        end: exact_parameter(1, policy),
        curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(start, midpoint, end)),
    }
}

fn curved_parallel_cap(policy: &CurveContext) -> CurveRegion2 {
    let parallel = QuadraticBezier2::new(point(0, 0), point(2, 2), point(4, 0))
        .parallel_left(Real::one())
        .unwrap();
    let right = decided(parallel.point_at(&Real::one(), policy).unwrap());
    let left = decided(parallel.point_at(&Real::zero(), policy).unwrap());
    let lower_left = Point2::new(left.x().clone(), Real::from(-2));
    let lower_right = Point2::new(right.x().clone(), Real::from(-2));
    let boundary = CurveRegionBoundaryLoop2::new(
        vec![
            analytic_parallel_fragment(point(0, 0), point(2, 2), point(4, 0), 1, true, policy),
            materialized_line(left, lower_left.clone(), policy),
            materialized_line(lower_left, lower_right.clone(), policy),
            materialized_line(lower_right, right, policy),
        ],
        policy,
    )
    .unwrap();
    CurveRegion2::try_new_with_loop_topology(
        vec![boundary],
        vec![CurveRegionLoopRole::Material],
        vec![FillRule::NonZero],
        vec![CurveBoundaryInteriorSide2::Left],
    )
    .unwrap()
}

fn clipped_region(
    path: &CurvePath2,
    clip: CurvePath2,
    interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    let promote = |path: &CurvePath2, interior_side| {
        CurveRegion2::try_from_boundary_paths_with_loop_topology(
            std::slice::from_ref(path),
            &[CurveRegionLoopRole::Material],
            &[FillRule::EvenOdd],
            &[interior_side],
            policy,
        )
        .unwrap()
        .into_value()
    };
    promote(path, interior_side)
        .boolean_region(
            &promote(&clip, CurveBoundaryInteriorSide2::Left),
            BooleanOp::Intersection,
            policy,
        )
        .unwrap()
        .into_value()
}

fn conic_overlap_regions(
    reparameterize: bool,
    policy: &CurveContext,
) -> (CurveRegion2, CurveRegion2) {
    let start = point(-2, 4);
    let control = point(0, -4);
    let end = point(2, 4);
    let quadratic = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            start.clone(),
            control.clone(),
            end.clone(),
        )),
        Curve2::from(LineSeg2::try_new(end.clone(), start.clone()).unwrap()),
    ])
    .unwrap();
    let reparameterized = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                vec![start.clone(), control, end.clone()],
                vec![Real::one(), Real::from(2_i8), Real::from(4_i8)],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(end, start).unwrap()),
    ])
    .unwrap();
    let wide = if reparameterize {
        &reparameterized
    } else {
        &quadratic
    };
    (
        clipped_region(
            &quadratic,
            rectangle_path(-3, -1, 3, 2),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
        clipped_region(
            wide,
            rectangle_path(-3, -1, 3, 3),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
    )
}

fn nonlinear_line_overlap_regions(policy: &CurveContext) -> (CurveRegion2, CurveRegion2) {
    let line_region = |control_x| {
        CurvePath2::try_new(vec![
            Curve2::from(
                RationalBezier2::try_new(
                    vec![point(0, 0), point(control_x, 0), point(4, 0)],
                    vec![Real::one(); 3],
                )
                .unwrap(),
            ),
            Curve2::from(LineSeg2::try_new(point(4, 0), point(4, 4)).unwrap()),
            Curve2::from(LineSeg2::try_new(point(4, 4), point(0, 4)).unwrap()),
            Curve2::from(LineSeg2::try_new(point(0, 4), point(0, 0)).unwrap()),
        ])
        .unwrap()
    };
    (
        clipped_region(
            &line_region(1),
            rectangle_path(-1, -1, 2, 5),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
        clipped_region(
            &line_region(3),
            rectangle_path(-1, -1, 3, 5),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
    )
}

fn cubic_mobius_overlap_regions(policy: &CurveContext) -> (CurveRegion2, CurveRegion2) {
    let controls = [point(0, 0), point(7, -5), point(8, -4), point(3, 3)];
    let polynomial = CurvePath2::try_new(vec![
        Curve2::from(CubicBezier2::new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            controls[3].clone(),
        )),
        Curve2::from(LineSeg2::try_new(controls[3].clone(), controls[0].clone()).unwrap()),
    ])
    .unwrap();
    let projective = CurvePath2::try_new(vec![
        Curve2::from(
            RationalBezier2::try_new(
                controls.to_vec(),
                vec![
                    Real::one(),
                    Real::from(2_i8),
                    Real::from(4_i8),
                    Real::from(8_i8),
                ],
            )
            .unwrap(),
        ),
        Curve2::from(LineSeg2::try_new(controls[3].clone(), controls[0].clone()).unwrap()),
    ])
    .unwrap();
    (
        clipped_region(
            &polynomial,
            rectangle_path(-20, -10, 20, -1),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
        clipped_region(
            &projective,
            rectangle_path(-20, -10, 20, 0),
            CurveBoundaryInteriorSide2::Left,
            policy,
        ),
    )
}

fn region_weight(region: &CurveRegion2) -> usize {
    region
        .boundary_loops()
        .iter()
        .map(|boundary| boundary.len())
        .sum()
}

fn measure(operation: &mut impl FnMut() -> usize, iterations: u32) -> (u128, usize) {
    let started = Instant::now();
    let mut checksum = 0;
    for _ in 0..iterations {
        checksum ^= black_box(operation());
    }
    (started.elapsed().as_nanos(), checksum)
}

fn main() {
    let policy = match std::env::var("HYPERCURVE_CURVE_REGION_BATCH_POLICY").as_deref() {
        Ok("approximate-512") => CurveContext::APPROXIMATE_512,
        Ok("strict") | Err(_) => CurveContext::STRICT,
        Ok(policy) => panic!("unknown batch benchmark policy {policy}"),
    };
    let native_regions = |contours: (Contour2, Contour2)| {
        [contours.0, contours.1].map(|contour| {
            CurveRegion2::try_from_native_material_contours(vec![contour], &policy)
                .unwrap()
                .into_value()
        })
    };
    let [first, second] = match std::env::var("HYPERCURVE_CURVE_REGION_BATCH_FIXTURE").as_deref() {
        Ok("circles") => native_regions((circle(0), circle(1))),
        Ok("capsules") => native_regions((capsule(0), capsule(2))),
        Ok("elevated-circles") => [
            CurveRegion2::try_from_native_material_contours(vec![circle(0)], &policy)
                .unwrap()
                .into_value(),
            elevated_circle(1, &policy),
        ],
        Ok("point-touch-rectangles") => {
            native_regions((rectangle(0, 0, 2, 2), rectangle(2, 2, 4, 4)))
        }
        Ok("shared-edge-rectangles") => {
            native_regions((rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)))
        }
        Ok("aligned-conic-overlap") => conic_overlap_regions(false, &policy).into(),
        Ok("mobius-overlap") => conic_overlap_regions(true, &policy).into(),
        Ok("mobius-cubic-overlap") => cubic_mobius_overlap_regions(&policy).into(),
        Ok("nonlinear-line-overlap") => nonlinear_line_overlap_regions(&policy).into(),
        Ok("analytic-squares") => [
            analytic_square(0, 4, &policy),
            analytic_square(2, 6, &policy),
        ],
        Ok("analytic-curved-cap") => [curved_parallel_cap(&policy), analytic_square(1, 5, &policy)],
        Ok(fixture) => panic!("unknown batch benchmark fixture {fixture}"),
        Err(_) => native_regions((rectangle(0, 0, 4, 4), rectangle(2, 0, 6, 4))),
    };
    let iterations = std::env::var("HYPERCURVE_CURVE_REGION_BATCH_ITERATIONS")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(1_000_u32);

    let mut independent = || {
        [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ]
        .into_iter()
        .map(|operation| {
            first
                .boolean_region(&second, operation, &policy)
                .unwrap()
                .into_value()
        })
        .map(|region| region_weight(&region))
        .sum()
    };
    let mut shared = || {
        let regions = first
            .boolean_regions(&second, &policy)
            .unwrap()
            .into_value();
        region_weight(regions.union())
            + region_weight(regions.intersection())
            + region_weight(regions.difference())
            + region_weight(regions.xor())
    };

    if std::env::var_os("HYPERCURVE_CURVE_REGION_BATCH_REPORT").is_some() {
        let independent_regions = [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ]
        .map(|operation| {
            first
                .boolean_region(&second, operation, &policy)
                .unwrap()
                .into_value()
        });
        let shared_regions = first
            .boolean_regions(&second, &policy)
            .unwrap()
            .into_value();
        println!(
            "independent_fragment_counts={:?}, shared_fragment_counts={:?}",
            independent_regions.map(|region| region_weight(&region)),
            [
                region_weight(shared_regions.union()),
                region_weight(shared_regions.intersection()),
                region_weight(shared_regions.difference()),
                region_weight(shared_regions.xor()),
            ]
        );
    }

    black_box(independent());
    black_box(shared());
    match std::env::var("HYPERCURVE_CURVE_REGION_BATCH_MODE").as_deref() {
        Ok("independent") => {
            let (ns, checksum) = measure(&mut independent, iterations);
            println!(
                "curve_region_boolean_four_independent: {iterations} iterations, total_ns={ns}, ns_per_iter={}, checksum={checksum}",
                ns / u128::from(iterations)
            );
            return;
        }
        Ok("shared") => {
            let (ns, checksum) = measure(&mut shared, iterations);
            println!(
                "curve_region_boolean_shared_all_four: {iterations} iterations, total_ns={ns}, ns_per_iter={}, checksum={checksum}",
                ns / u128::from(iterations)
            );
            return;
        }
        Ok(mode) => panic!("unknown batch benchmark mode {mode}"),
        Err(_) => {}
    }
    let shared_first = std::env::var_os("HYPERCURVE_CURVE_REGION_BATCH_SHARED_FIRST").is_some();
    let ((independent_ns, independent_checksum), (shared_ns, shared_checksum)) = if shared_first {
        let shared = measure(&mut shared, iterations);
        let independent = measure(&mut independent, iterations);
        (independent, shared)
    } else {
        let independent = measure(&mut independent, iterations);
        let shared = measure(&mut shared, iterations);
        (independent, shared)
    };
    println!(
        "curve_region_boolean_four_independent: {iterations} iterations, total_ns={independent_ns}, ns_per_iter={}, checksum={independent_checksum}",
        independent_ns / u128::from(iterations)
    );
    println!(
        "curve_region_boolean_shared_all_four: {iterations} iterations, total_ns={shared_ns}, ns_per_iter={}, checksum={shared_checksum}",
        shared_ns / u128::from(iterations)
    );
}
