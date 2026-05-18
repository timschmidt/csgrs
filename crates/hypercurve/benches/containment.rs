use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, ContourPointLocation, CurvePolicy,
    CurveResult, CurveString2, FiniteProjectionOptions, LineSeg2, LineSide, Point2, Real, Region2,
    RegionPointLocation,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(0))
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(xmin, ymin),
        vertex(xmax, ymin),
        vertex(xmax, ymax),
        vertex(xmin, ymax),
    ])
    .unwrap()
}

fn sparse_region(contour_count: i32) -> Region2 {
    let mut contours = Vec::with_capacity(contour_count as usize);
    for index in 0..contour_count {
        let x = index * 10;
        contours.push(rectangle(x, 0, x + 4, 4));
    }
    Region2::from_material_contours(contours)
}

fn bench_contour_bbox_miss(iterations: u32) -> CurveResult<()> {
    let contour = rectangle(0, 0, 10, 10);
    let point = p(100, 100);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut outside_count = 0_usize;

    for _ in 0..iterations {
        match contour.classify_point(&point, &policy) {
            Classification::Decided(ContourPointLocation::Outside) => {
                outside_count += black_box(1);
            }
            other => panic!("contour bbox miss benchmark expected outside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "contour_bbox_miss_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), outside={outside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_contour_bbox_miss(iterations: u32) -> CurveResult<()> {
    let contour = rectangle(0, 0, 10, 10);
    let point = p(100, 100);
    let policy = CurvePolicy::certified();
    let prepared = contour.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut outside_count = 0_usize;

    for _ in 0..iterations {
        match prepared.classify_point(&point, &policy) {
            Classification::Decided(ContourPointLocation::Outside) => {
                outside_count += black_box(1);
            }
            other => panic!("prepared contour bbox miss benchmark expected outside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_contour_bbox_miss_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), outside={outside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_line_segment_classify(iterations: u32) -> CurveResult<()> {
    let line = LineSeg2::try_new(p(0, 0), p(997, 311))?;
    let prepared = line.prepare_topology_queries();
    let point = p(401, 971);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut left_count = 0_usize;

    for _ in 0..iterations {
        match prepared.classify_point(&point, &policy) {
            Classification::Decided(LineSide::Left) => {
                left_count += black_box(1);
            }
            other => panic!("prepared line classify benchmark expected left, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_line_segment_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), left={left_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_circular_arc_contains(iterations: u32) -> CurveResult<()> {
    let arc = CircularArc2::from_bulge(p(-100, 0), p(100, 0), s(1))?;
    let prepared = arc.prepare_topology_queries();
    let point = p(0, -100);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut hit_count = 0_usize;

    for _ in 0..iterations {
        match prepared.contains_point(&point, &policy) {
            Classification::Decided(true) => {
                hit_count += black_box(1);
            }
            other => panic!("prepared circular arc benchmark expected hit, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_circular_arc_contains: {iterations} iterations in {elapsed:?} ({:?}/iter), hits={hit_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_region_outside(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let point = p(5_000, 5_000);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut outside_count = 0_usize;

    for _ in 0..iterations {
        match region.classify_point(&point, &policy) {
            Classification::Decided(RegionPointLocation::Outside) => {
                outside_count += black_box(1);
            }
            other => panic!("sparse outside benchmark expected outside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_region_outside_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), outside={outside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_sparse_region_outside(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let point = p(5_000, 5_000);
    let policy = CurvePolicy::certified();
    let prepared = region.prepare_point_classifier(&policy);
    let started = Instant::now();
    let mut outside_count = 0_usize;

    for _ in 0..iterations {
        match prepared.classify_point(&point, &policy) {
            Classification::Decided(RegionPointLocation::Outside) => {
                outside_count += black_box(1);
            }
            other => panic!("prepared sparse outside benchmark expected outside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_sparse_region_outside_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), outside={outside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_region_single_hit(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let point = p(612, 2);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut inside_count = 0_usize;

    for _ in 0..iterations {
        match region.classify_point(&point, &policy) {
            Classification::Decided(RegionPointLocation::Inside) => {
                inside_count += black_box(1);
            }
            other => panic!("sparse single-hit benchmark expected inside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_region_single_hit_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), inside={inside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_region_filled_area(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        match region.filled_area(&policy)? {
            Classification::Decided(Some(area)) => {
                checksum ^= format!("{area:?}").len();
            }
            other => panic!("sparse area benchmark expected exact filled area, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_region_filled_area: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_region_filled_area_report(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        match region.filled_area_report(&policy)? {
            Classification::Decided(report) if report.is_complete() => {
                checksum ^= report.material_contour_count;
                checksum ^= format!("{:?}", report.filled_area).len();
            }
            other => panic!("sparse area-report benchmark expected exact report, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_region_filled_area_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_contour_finite_projection_certificate(iterations: u32) -> CurveResult<()> {
    let contour = rectangle(0, 0, 100, 100);
    let options = FiniteProjectionOptions::try_new(0.01)?;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let projection = black_box(&contour).project_to_finite_ring(&options)?;
        checksum ^= projection.certificate().source_segment_count();
        checksum ^= projection.certificate().emitted_point_count();
        checksum ^= projection.points().len();
    }

    let elapsed = started.elapsed();
    println!(
        "contour_finite_projection_certificate: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_region_finite_projection_certificate(iterations: u32) -> CurveResult<()> {
    let region = Region2::new(
        vec![rectangle(0, 0, 100, 100)],
        vec![rectangle(25, 25, 75, 75)],
    );
    let options = FiniteProjectionOptions::try_new(0.01)?;
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let projection = black_box(&region).project_to_finite_region(&options)?;
        checksum ^= projection.certificate().material_ring_count();
        checksum ^= projection.certificate().hole_ring_count();
        checksum ^= projection.certificate().emitted_point_count();
    }

    let elapsed = started.elapsed();
    println!(
        "region_finite_projection_certificate: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_finite_import_certificate(iterations: u32) -> CurveResult<()> {
    let line_points = &[[0.0, 0.0], [25.0, 0.0], [25.0, 0.0], [50.0, 25.0]];
    let ring_points = &[
        [0.0, 0.0],
        [100.0, 0.0],
        [100.0, 100.0],
        [0.0, 100.0],
        [0.0, 0.0],
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;

    for _ in 0..iterations {
        let line_import = CurveString2::import_finite_line_string(black_box(line_points))?;
        checksum ^= line_import.certificate().input_point_count();
        checksum ^= line_import.certificate().skipped_duplicate_edge_count();
        checksum ^= line_import.certificate().output_segment_count();

        let ring_import = Contour2::import_finite_ring(black_box(ring_points))?;
        checksum ^= ring_import.certificate().retained_point_count();
        checksum ^= usize::from(ring_import.certificate().repeated_closing_point());
        checksum ^= ring_import.certificate().output_segment_count();
    }

    let elapsed = started.elapsed();
    println!(
        "finite_import_certificate: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_sparse_region_single_hit(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let point = p(612, 2);
    let policy = CurvePolicy::certified();
    let prepared = region.prepare_point_classifier(&policy);
    let started = Instant::now();
    let mut inside_count = 0_usize;

    for _ in 0..iterations {
        match prepared.classify_point(&point, &policy) {
            Classification::Decided(RegionPointLocation::Inside) => {
                inside_count += black_box(1);
            }
            other => panic!("prepared sparse single-hit benchmark expected inside, got {other:?}"),
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_sparse_region_single_hit_classify: {iterations} iterations in {elapsed:?} ({:?}/iter), inside={inside_count}",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> CurveResult<()> {
    bench_prepared_line_segment_classify(100_000)?;
    bench_prepared_circular_arc_contains(100_000)?;
    bench_contour_bbox_miss(100_000)?;
    bench_prepared_contour_bbox_miss(100_000)?;
    bench_sparse_region_outside(10_000)?;
    bench_prepared_sparse_region_outside(10_000)?;
    bench_sparse_region_single_hit(10_000)?;
    bench_prepared_sparse_region_single_hit(10_000)?;
    bench_sparse_region_filled_area(10_000)?;
    bench_sparse_region_filled_area_report(10_000)?;
    bench_contour_finite_projection_certificate(10_000)?;
    bench_region_finite_projection_certificate(10_000)?;
    bench_finite_import_certificate(10_000)?;
    Ok(())
}
