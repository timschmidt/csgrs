use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BooleanOp, BulgeVertex2, Classification, Contour2, CurvePolicy, DefaultBackend, FillRule,
    Region2, Scalar,
};

fn s(value: i32) -> Scalar<DefaultBackend> {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2<DefaultBackend> {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2<DefaultBackend> {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
    .unwrap()
}

fn rectangle_rotated_start(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(&[
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
    ])
    .unwrap()
}

fn rectangle_reversed(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin, 0),
        vertex(xmin, ymax, 0),
        vertex(xmax, ymax, 0),
        vertex(xmax, ymin, 0),
    ])
    .unwrap()
}

fn region(contours: Vec<Contour2<DefaultBackend>>) -> Region2<DefaultBackend> {
    Region2::from_material_contours(contours)
}

fn overlapping_rectangles() -> (Region2<DefaultBackend>, Region2<DefaultBackend>) {
    (
        region(vec![rectangle(0, 0, 4, 4)]),
        region(vec![rectangle(2, -1, 6, 3)]),
    )
}

fn touching_material_bins() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![rectangle(0, 0, 2, 2), rectangle(2, 0, 4, 2)])
}

fn touching_material_bins_reordered() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![rectangle(2, 0, 4, 2), rectangle(0, 0, 2, 2)])
}

fn touching_material_bins_rotated_and_reversed() -> Region2<DefaultBackend> {
    Region2::from_material_contours(vec![
        rectangle_reversed(2, 0, 4, 2),
        rectangle_rotated_start(0, 0, 2, 2),
    ])
}

fn staggered_grid(side: i32, offset: i32) -> Region2<DefaultBackend> {
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

fn bench_case(
    name: &str,
    first: &Region2<DefaultBackend>,
    second: &Region2<DefaultBackend>,
    op: BooleanOp,
    iterations: u32,
) {
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

fn bench_region_case(
    name: &str,
    first: &Region2<DefaultBackend>,
    second: &Region2<DefaultBackend>,
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

fn bench_boundary_contour_case(
    name: &str,
    first: &Region2<DefaultBackend>,
    second: &Region2<DefaultBackend>,
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
    bench_region_case(
        "staggered_grid_5x5_union_region",
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

    let donut = Region2::new(vec![rectangle(0, 0, 10, 10)], vec![rectangle(3, 3, 7, 7)]);
    bench_region_case(
        "identical_donut_union_region",
        &donut,
        &donut,
        BooleanOp::Union,
        10_000,
    );

    let empty = Region2::<DefaultBackend>::empty();
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
