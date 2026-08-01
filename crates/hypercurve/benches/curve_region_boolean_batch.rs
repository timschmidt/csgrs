use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, BulgeVertex2, Contour2, Curve2, CurveBoundaryInteriorSide2, CurveContext,
    CurvePath2, CurveRegion2, LineSeg2, Point2, QuadraticBezier2, RationalBezier2, Real,
};

fn point(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
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

fn clipped_region(
    path: &CurvePath2,
    clip: CurvePath2,
    interior_side: CurveBoundaryInteriorSide2,
    policy: &CurveContext,
) -> CurveRegion2 {
    path.boolean_region(
        &clip,
        BooleanOp::Intersection,
        interior_side,
        CurveBoundaryInteriorSide2::Left,
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
    let policy = CurveContext::STRICT;
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
        Ok("point-touch-rectangles") => {
            native_regions((rectangle(0, 0, 2, 2), rectangle(2, 2, 4, 4)))
        }
        Ok("shared-edge-rectangles") => {
            native_regions((rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)))
        }
        Ok("aligned-conic-overlap") => conic_overlap_regions(false, &policy).into(),
        Ok("mobius-overlap") => conic_overlap_regions(true, &policy).into(),
        Ok("nonlinear-line-overlap") => nonlinear_line_overlap_regions(&policy).into(),
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
