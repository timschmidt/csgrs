use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, CurvePolicy, FillRule, Real, Region2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator))
        .expect("positive integer benchmark denominator must define an exact rational")
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn pr(x: Real, y: Real) -> hypercurve::Point2 {
    hypercurve::Point2::new(x, y)
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn real_vertex(x: Real, y: Real) -> BulgeVertex2 {
    BulgeVertex2::new(pr(x, y), Real::zero())
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
    .unwrap()
}

fn real_rectangle(xmin: Real, ymin: Real, xmax: Real, ymax: Real) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        real_vertex(xmin.clone(), ymin.clone()),
        real_vertex(xmax.clone(), ymin),
        real_vertex(xmax, ymax.clone()),
        real_vertex(xmin, ymax),
    ])
    .unwrap()
}

fn rectangle_rotated_start(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
    ])
    .unwrap()
}

fn rectangle_reversed(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmin, ymax, 0),
        vertex(xmax, ymax, 0),
        vertex(xmax, ymin, 0),
    ])
    .unwrap()
}

fn region(contours: Vec<Contour2>) -> Region2 {
    Region2::from_material_contours(contours)
}

fn overlapping_rectangles() -> (Region2, Region2) {
    (
        region(vec![rectangle(0, 0, 4, 4)]),
        region(vec![rectangle(2, -1, 6, 3)]),
    )
}

fn touching_material_bins() -> Region2 {
    Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)])
}

fn touching_material_bins_reordered() -> Region2 {
    Region2::from_material_contours(vec![rectangle(2, 0, 4, 2), rectangle(0, 0, 2, 2)])
}

fn touching_material_bins_rotated_and_reversed() -> Region2 {
    Region2::from_material_contours(vec![
        rectangle_reversed(2, 0, 4, 2),
        rectangle_rotated_start(0, 0, 2, 2),
    ])
}

fn staggered_grid(side: i32, offset: i32) -> Region2 {
    let mut contours = Vec::new();
    for row in 0..side {
        for col in 0..side {
            let x = col * 10 + offset;
            let y = row * 10 + offset;
            contours.push(rectangle(x, y, x + 4, y + 4));
        }
    }
    region(contours)
}

fn bench_case(name: &str, first: &Region2, second: &Region2, op: BooleanOp, iterations: u32) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_loops = 0_usize;

    for _ in 0..iterations {
        let loops = match first.boolean_boundary_loops(second, op, &policy).unwrap() {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_loops += black_box(loops.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loops={total_loops}",
        elapsed / iterations
    );
}

fn bench_prepared_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_loops = 0_usize;

    for _ in 0..iterations {
        let loops = match first.boolean_boundary_loops(&second, op, &policy).unwrap() {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_loops += black_box(loops.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loops={total_loops}",
        elapsed / iterations
    );
}

/// Benchmark mixed left-prepared boundary-loop report dispatch.
///
/// Verifies loop-report parity when the left operand is prepared and the right
/// side is a borrowed plain view.
fn bench_left_prepared_boundary_loop_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_loop_report_against_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} left-prepared loop report became uncertain: {reason:?}");
            }
        };
        total_checks += black_box(
            report.audit.loop_count + report.audit.checked_loop_pair_count + report.loops.len(),
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loop audit checks={total_checks}",
        elapsed / iterations
    );
}

/// Benchmark mixed right-prepared boundary-loop report dispatch.
///
/// Verifies loop-report parity when only the right operand is prepared.
fn bench_right_prepared_boundary_loop_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_loop_report_against_prepared_region(
                &second,
                op,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} right-prepared loop report became uncertain: {reason:?}");
            }
        };
        total_checks += black_box(
            report.audit.loop_count + report.audit.checked_loop_pair_count + report.loops.len(),
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loop audit checks={total_checks}",
        elapsed / iterations
    );
}

/// Benchmark mixed left-prepared region report dispatch.
///
/// Mirrors `Region2::boolean_region_report` against a view-only right operand,
/// stressing mixed ownership handoff while preserving report structure.
fn bench_left_prepared_region_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_report_against_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} left-prepared region report became uncertain: {reason:?}");
            }
        };
        total_checks +=
            black_box(report.audit.checked_contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total region audit checks={total_checks}",
        elapsed / iterations
    );
}

/// Benchmark mixed right-prepared region report dispatch.
///
/// Mirrors `Region2::boolean_region_report_against_prepared_region` semantics
/// in the right-hand prepared path.
fn bench_right_prepared_region_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_report_against_prepared_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} right-prepared region report became uncertain: {reason:?}");
            }
        };
        total_checks +=
            black_box(report.audit.checked_contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total region audit checks={total_checks}",
        elapsed / iterations
    );
}

/// Benchmark mixed left-prepared region report dispatch.
///
/// Right side remains a borrowed view so this path stresses mixed prepared/view
/// report construction under Yap-aligned auditable operations.
fn bench_left_prepared_region_pipeline_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_pipeline_report_against_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} left-prepared pipeline report became uncertain: {reason:?}");
            }
        };
        total_checks += black_box(
            report.boundary_audit.contour_count
                + report.boundary_audit.checked_contour_pair_count
                + report.nesting_audit.checked_containment_pair_count
                + report.region_audit.checked_contour_count
                + report.region_audit.checked_contour_pair_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total pipeline audit checks={total_checks}",
        elapsed / iterations
    );
}

/// Benchmark mixed right-prepared region pipeline report dispatch.
///
/// Right side stays prepared while left is borrowed, verifying the same full
/// pipeline report shape as owned/prepared and plain paths.
fn bench_right_prepared_region_pipeline_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_checks = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_pipeline_report_against_prepared_region(
                &second,
                op,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} right-prepared pipeline report became uncertain: {reason:?}");
            }
        };
        total_checks += black_box(
            report.boundary_audit.contour_count
                + report.boundary_audit.checked_contour_pair_count
                + report.nesting_audit.checked_containment_pair_count
                + report.region_audit.checked_contour_count
                + report.region_audit.checked_contour_pair_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total pipeline audit checks={total_checks}",
        elapsed / iterations
    );
}

fn bench_traversal_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_graph_items = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_traversal_report(second, op, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_graph_items += black_box(
            report.classified_fragment_count
                + report.directed_fragment_count
                + report.assembled_chain_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total graph items={total_graph_items}",
        elapsed / iterations
    );
}

fn bench_prepared_traversal_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_graph_items = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_traversal_report(&second, op, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_graph_items += black_box(
            report.classified_fragment_count
                + report.directed_fragment_count
                + report.assembled_chain_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total graph items={total_graph_items}",
        elapsed / iterations
    );
}

fn bench_left_prepared_traversal_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_graph_items = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_traversal_report_against_region(&second, op, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_graph_items += black_box(
            report.classified_fragment_count
                + report.directed_fragment_count
                + report.assembled_chain_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total graph items={total_graph_items}",
        elapsed / iterations
    );
}

fn bench_right_prepared_traversal_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_graph_items = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_traversal_report_against_prepared_region(&second, op, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_graph_items += black_box(
            report.classified_fragment_count
                + report.directed_fragment_count
                + report.assembled_chain_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total graph items={total_graph_items}",
        elapsed / iterations
    );
}

fn bench_left_prepared_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_loops = 0_usize;

    for _ in 0..iterations {
        let loops = match first
            .boolean_boundary_loops_against_region(&second, op, &policy)
            .unwrap()
        {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_loops += black_box(loops.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loops={total_loops}",
        elapsed / iterations
    );
}

fn bench_right_prepared_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_loops = 0_usize;

    for _ in 0..iterations {
        let loops = match first
            .boolean_boundary_loops_against_prepared_region(&second, op, &policy)
            .unwrap()
        {
            Classification::Decided(loops) => loops,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_loops += black_box(loops.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loops={total_loops}",
        elapsed / iterations
    );
}

fn bench_region_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_contours = 0_usize;

    for _ in 0..iterations {
        let region = match first
            .boolean_region(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_contours +=
            black_box(region.material_contours().len() + region.hole_contours().len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contours={total_contours}",
        elapsed / iterations
    );
}

fn bench_prepared_region_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_contours = 0_usize;

    for _ in 0..iterations {
        let region = match first
            .boolean_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(region) => region,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_contours +=
            black_box(region.material_contours().len() + region.hole_contours().len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contours={total_contours}",
        elapsed / iterations
    );
}

fn bench_region_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_report(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_checked +=
            black_box(report.audit.checked_contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total audit checks={total_checked}",
        elapsed / iterations
    );
}

fn bench_prepared_region_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_report(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_checked +=
            black_box(report.audit.checked_contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total audit checks={total_checked}",
        elapsed / iterations
    );
}

fn bench_region_pipeline_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_region_pipeline_report(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_checked += black_box(
            report.boundary_audit.contour_count
                + report.boundary_audit.checked_contour_pair_count
                + report.nesting_audit.checked_containment_pair_count
                + report.region_audit.checked_contour_pair_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total pipeline audit checks={total_checked}",
        elapsed / iterations
    );
}

fn bench_boundary_contour_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_contours = 0_usize;

    for _ in 0..iterations {
        let contours = match first
            .boolean_boundary_contours(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(contours) => contours,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_contours += black_box(contours.len());
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contours={total_contours}",
        elapsed / iterations
    );
}

fn bench_boundary_contour_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_contour_report(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_checked +=
            black_box(report.audit.contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contour audit checks={total_checked}",
        elapsed / iterations
    );
}

/// Benchmark mixed left-prepared boundary-contour report dispatch.
///
/// Foster, Hormann, and Popa (2019) treat boundary-contour degeneracies as
/// report-ready products; this benchmark keeps the mixed-view path from regressing.
fn bench_left_prepared_boundary_contour_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.prepare_topology_queries(&policy);
    let second = second.as_view();
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_contour_report_against_region(&second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} left-prepared boundary-contour report became uncertain: {reason:?}");
            }
        };
        total_checked +=
            black_box(report.audit.contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contour audit checks={total_checked}",
        elapsed / iterations
    );
}

/// Benchmark mixed right-prepared boundary-contour report dispatch.
///
/// This keeps plain/owned right-hand preparation equivalent on report-only
/// boundary products that should already regularize antagonistic contacts.
fn bench_right_prepared_boundary_contour_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let first = first.as_view();
    let second = second.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_contour_report_against_prepared_region(
                &second,
                op,
                FillRule::NonZero,
                &policy,
            )
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!(
                    "{name} right-prepared boundary-contour report became uncertain: {reason:?}"
                );
            }
        };
        total_checked +=
            black_box(report.audit.contour_count + report.audit.checked_contour_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total contour audit checks={total_checked}",
        elapsed / iterations
    );
}

fn bench_boundary_loop_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report = match first
            .boolean_boundary_loop_report(second, op, FillRule::NonZero, &policy)
            .unwrap()
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_checked += black_box(report.audit.loop_count + report.audit.checked_loop_pair_count);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total loop audit checks={total_checked}",
        elapsed / iterations
    );
}

fn bench_boundary_nesting_report_case(
    name: &str,
    first: &Region2,
    second: &Region2,
    op: BooleanOp,
    iterations: u32,
) {
    let policy = CurvePolicy::certified();
    let contours = match first
        .boolean_boundary_contours(second, op, FillRule::NonZero, &policy)
        .unwrap()
    {
        Classification::Decided(contours) => contours,
        Classification::Uncertain(reason) => {
            panic!("{name} contour construction became uncertain before benchmark: {reason:?}");
        }
    };
    let started = Instant::now();
    let mut total_checked = 0_usize;

    for _ in 0..iterations {
        let report =
            match Region2::from_boundary_contours_report(contours.clone(), &policy).unwrap() {
                Classification::Decided(report) => report,
                Classification::Uncertain(reason) => {
                    panic!("{name} became uncertain during benchmark: {reason:?}");
                }
            };
        total_checked += black_box(
            report.audit.input_contour_count + report.audit.checked_containment_pair_count,
        );
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total nesting audit checks={total_checked}",
        elapsed / iterations
    );
}

fn main() {
    let (first, second) = overlapping_rectangles();
    bench_case(
        "overlap_rectangles_union",
        &first,
        &second,
        BooleanOp::Union,
        2_000,
    );
    bench_case(
        "overlap_rectangles_intersection",
        &first,
        &second,
        BooleanOp::Intersection,
        2_000,
    );
    bench_case(
        "overlap_rectangles_difference",
        &first,
        &second,
        BooleanOp::Difference,
        2_000,
    );

    let grid_a = staggered_grid(5, 0);
    let grid_b = staggered_grid(5, 2);
    bench_case(
        "staggered_grid_5x5_union",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_traversal_report_case(
        "staggered_grid_5x5_union_traversal_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_prepared_case(
        "prepared_staggered_grid_5x5_union",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_left_prepared_case(
        "left_prepared_staggered_grid_5x5_union",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_right_prepared_case(
        "right_prepared_staggered_grid_5x5_union",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_region_case(
        "staggered_grid_5x5_union_region",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_region_report_case(
        "staggered_grid_5x5_union_region_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_region_pipeline_report_case(
        "staggered_grid_5x5_union_region_pipeline_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_boundary_contour_report_case(
        "staggered_grid_5x5_union_boundary_contour_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_boundary_loop_report_case(
        "staggered_grid_5x5_union_boundary_loop_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_boundary_nesting_report_case(
        "staggered_grid_5x5_union_boundary_nesting_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_prepared_region_report_case(
        "prepared_staggered_grid_5x5_union_region_report",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );
    bench_prepared_region_case(
        "prepared_staggered_grid_5x5_union_region",
        &grid_a,
        &grid_b,
        BooleanOp::Union,
        100,
    );

    let outer = region(vec![rectangle(0, 0, 10, 10)]);
    let inner = region(vec![rectangle(3, 3, 7, 7)]);
    bench_region_case(
        "nested_rectangles_difference_region",
        &outer,
        &inner,
        BooleanOp::Difference,
        1_000,
    );

    let edge_touching_inner = region(vec![rectangle(3, 0, 7, 3)]);
    bench_region_case(
        "boundary_touching_containment_union_region",
        &outer,
        &edge_touching_inner,
        BooleanOp::Union,
        10_000,
    );
    bench_region_case(
        "boundary_touching_containment_intersection_region",
        &outer,
        &edge_touching_inner,
        BooleanOp::Intersection,
        10_000,
    );
    bench_region_case(
        "boundary_touching_subset_minus_container_region",
        &edge_touching_inner,
        &outer,
        BooleanOp::Difference,
        10_000,
    );
    bench_region_case(
        "boundary_touching_container_minus_subset_region",
        &outer,
        &edge_touching_inner,
        BooleanOp::Difference,
        10_000,
    );
    bench_boundary_contour_case(
        "boundary_touching_containment_union_boundary_contours",
        &outer,
        &edge_touching_inner,
        BooleanOp::Union,
        10_000,
    );
    bench_boundary_contour_case(
        "boundary_touching_container_minus_subset_boundary_contours",
        &outer,
        &edge_touching_inner,
        BooleanOp::Difference,
        10_000,
    );
    bench_case(
        "boundary_touching_containment_union_boundary_loops",
        &outer,
        &edge_touching_inner,
        BooleanOp::Union,
        10_000,
    );
    bench_prepared_case(
        "prepared_boundary_touching_containment_union_boundary_loops",
        &outer,
        &edge_touching_inner,
        BooleanOp::Union,
        10_000,
    );
    bench_boundary_loop_report_case(
        "boundary_touching_containment_union_boundary_loop_report",
        &outer,
        &edge_touching_inner,
        BooleanOp::Union,
        10_000,
    );

    let donut = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    bench_region_case(
        "identical_donut_union_region",
        &donut,
        &donut,
        BooleanOp::Union,
        10_000,
    );

    let empty = Region2::empty();
    bench_region_case(
        "empty_donut_union_region",
        &empty,
        &donut,
        BooleanOp::Union,
        10_000,
    );

    let island = region(vec![rectangle(4, 4, 6, 6)]);
    bench_region_case(
        "donut_union_island_in_hole_region",
        &donut,
        &island,
        BooleanOp::Union,
        1_000,
    );

    let hole_boundary_donut =
        Region2::new(vec![rectangle(0, 0, 12, 12)], vec![rectangle(4, 4, 8, 8)]);
    let hole_boundary_cutter = region(vec![rectangle(6, 2, 10, 10)]);
    bench_region_case(
        "donut_hole_boundary_cutter_union_region",
        &hole_boundary_donut,
        &hole_boundary_cutter,
        BooleanOp::Union,
        1_000,
    );
    bench_region_case(
        "donut_hole_boundary_cutter_difference_region",
        &hole_boundary_donut,
        &hole_boundary_cutter,
        BooleanOp::Difference,
        1_000,
    );
    bench_region_case(
        "donut_hole_boundary_cutter_xor_region",
        &hole_boundary_donut,
        &hole_boundary_cutter,
        BooleanOp::Xor,
        1_000,
    );
    bench_boundary_contour_case(
        "donut_hole_boundary_cutter_xor_boundary_contours",
        &hole_boundary_donut,
        &hole_boundary_cutter,
        BooleanOp::Xor,
        1_000,
    );

    let point_touch_a = region(vec![rectangle(0, 0, 2, 2)]);
    let point_touch_b = region(vec![rectangle(2, 2, 4, 4)]);
    bench_region_case(
        "point_touch_rectangles_union_region",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_region_case(
        "point_touch_rectangles_intersection_region",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Intersection,
        10_000,
    );
    bench_boundary_contour_case(
        "point_touch_rectangles_xor_boundary_contours",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Xor,
        10_000,
    );

    let shared_edge_a = region(vec![rectangle(0, 0, 4, 4)]);
    let shared_edge_b = region(vec![rectangle(2, -2, 6, 0)]);
    bench_region_case(
        "shared_edge_rectangles_union_region",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_region_case(
        "shared_edge_rectangles_xor_region",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Xor,
        10_000,
    );
    bench_boundary_contour_case(
        "shared_edge_rectangles_union_boundary_contours",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_boundary_contour_report_case(
        "left_prepared_shared_edge_rectangles_union_boundary_contour_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_boundary_contour_report_case(
        "right_prepared_shared_edge_rectangles_union_boundary_contour_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_case(
        "shared_edge_rectangles_union_boundary_loops",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_prepared_case(
        "prepared_shared_edge_rectangles_union_boundary_loops",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_case(
        "left_prepared_shared_edge_rectangles_union_boundary_loops",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_case(
        "right_prepared_shared_edge_rectangles_union_boundary_loops",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_boundary_loop_report_case(
        "shared_edge_rectangles_union_boundary_loop_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_boundary_loop_report_case(
        "left_prepared_shared_edge_rectangles_union_boundary_loop_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_boundary_loop_report_case(
        "right_prepared_shared_edge_rectangles_union_boundary_loop_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_region_report_case(
        "left_prepared_shared_edge_rectangles_union_region_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_region_report_case(
        "right_prepared_shared_edge_rectangles_union_region_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_region_pipeline_report_case(
        "left_prepared_shared_edge_rectangles_union_region_pipeline_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_region_pipeline_report_case(
        "right_prepared_shared_edge_rectangles_union_region_pipeline_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_traversal_report_case(
        "shared_edge_rectangles_union_boundary_traversal_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_prepared_traversal_report_case(
        "prepared_shared_edge_rectangles_union_boundary_traversal_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_traversal_report_case(
        "left_prepared_shared_edge_rectangles_union_boundary_traversal_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_traversal_report_case(
        "right_prepared_shared_edge_rectangles_union_boundary_traversal_report",
        &shared_edge_a,
        &shared_edge_b,
        BooleanOp::Union,
        10_000,
    );
    bench_case(
        "point_touch_rectangles_union_boundary_loops",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_traversal_report_case(
        "point_touch_rectangles_union_boundary_traversal_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_prepared_traversal_report_case(
        "prepared_point_touch_rectangles_union_boundary_traversal_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_traversal_report_case(
        "left_prepared_point_touch_rectangles_union_boundary_traversal_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_traversal_report_case(
        "right_prepared_point_touch_rectangles_union_boundary_traversal_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_prepared_case(
        "prepared_point_touch_rectangles_union_boundary_loops",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_case(
        "left_prepared_point_touch_rectangles_union_boundary_loops",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_case(
        "right_prepared_point_touch_rectangles_union_boundary_loops",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_boundary_loop_report_case(
        "point_touch_rectangles_union_boundary_loop_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_boundary_contour_report_case(
        "point_touch_rectangles_union_boundary_contour_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_boundary_contour_report_case(
        "left_prepared_point_touch_rectangles_union_boundary_contour_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_boundary_contour_report_case(
        "right_prepared_point_touch_rectangles_union_boundary_contour_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_boundary_loop_report_case(
        "left_prepared_point_touch_rectangles_union_boundary_loop_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_boundary_loop_report_case(
        "right_prepared_point_touch_rectangles_union_boundary_loop_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_region_report_case(
        "left_prepared_point_touch_rectangles_union_region_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_region_report_case(
        "right_prepared_point_touch_rectangles_union_region_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_left_prepared_region_pipeline_report_case(
        "left_prepared_point_touch_rectangles_union_region_pipeline_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );
    bench_right_prepared_region_pipeline_report_case(
        "right_prepared_point_touch_rectangles_union_region_pipeline_report",
        &point_touch_a,
        &point_touch_b,
        BooleanOp::Union,
        10_000,
    );

    let epsilon = q(1, 1024);
    let near_collinear_base = region(vec![rectangle(0, 0, 8, 4)]);
    let near_collinear_gap = region(vec![real_rectangle(s(2), s(-2), s(6), -epsilon.clone())]);
    let near_collinear_overlap = region(vec![real_rectangle(s(2), s(-2), s(6), epsilon)]);
    bench_traversal_report_case(
        "near_collinear_gap_rectangles_union_boundary_traversal_report",
        &near_collinear_base,
        &near_collinear_gap,
        BooleanOp::Union,
        10_000,
    );
    bench_region_pipeline_report_case(
        "near_collinear_gap_rectangles_union_region_pipeline_report",
        &near_collinear_base,
        &near_collinear_gap,
        BooleanOp::Union,
        10_000,
    );
    bench_traversal_report_case(
        "near_collinear_overlap_rectangles_union_boundary_traversal_report",
        &near_collinear_base,
        &near_collinear_overlap,
        BooleanOp::Union,
        10_000,
    );
    bench_region_pipeline_report_case(
        "near_collinear_overlap_rectangles_union_region_pipeline_report",
        &near_collinear_base,
        &near_collinear_overlap,
        BooleanOp::Union,
        10_000,
    );

    let touching = touching_material_bins();
    bench_region_case(
        "touching_bins_self_union_region",
        &touching,
        &touching,
        BooleanOp::Union,
        10_000,
    );
    bench_region_case(
        "empty_touching_bins_union_region",
        &empty,
        &touching,
        BooleanOp::Union,
        10_000,
    );

    let touching_reordered = touching_material_bins_reordered();
    bench_region_case(
        "reordered_touching_bins_union_region",
        &touching,
        &touching_reordered,
        BooleanOp::Union,
        10_000,
    );

    let touching_rotated_reversed = touching_material_bins_rotated_and_reversed();
    bench_region_case(
        "rotated_reversed_touching_bins_union_region",
        &touching,
        &touching_rotated_reversed,
        BooleanOp::Union,
        10_000,
    );
}
