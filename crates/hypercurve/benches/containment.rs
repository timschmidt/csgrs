use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, ContourPointLocation, CurvePolicy,
    CurveResult, LineSeg2, LineSide, PlanarPcurveImageRelation2, Point2, Real, Region2,
    RegionPointLocation, RetainedPlanarFace2, RetainedPlanarFacePointLocation2,
    RetainedPlanarSurfaceIdentity2, RetainedPlanarTrimLoop2,
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

fn bench_planar_trim_loop_image_equality(iterations: u32) -> CurveResult<()> {
    let surface = RetainedPlanarSurfaceIdentity2::new(3);
    let first = RetainedPlanarTrimLoop2::new(surface, rectangle(0, 0, 10, 10));
    let rotated = RetainedPlanarTrimLoop2::new(
        surface,
        Contour2::from_bulge_vertices(&[
            vertex(10, 10),
            vertex(0, 10),
            vertex(0, 0),
            vertex(10, 0),
        ])?,
    );
    let started = Instant::now();
    let mut same_count = 0_usize;

    for _ in 0..iterations {
        let report = first.image_equality_report(&rotated);
        if report.relation() == PlanarPcurveImageRelation2::SameDirected {
            same_count += black_box(report.segment_count());
        }
    }

    let elapsed = started.elapsed();
    println!(
        "planar_trim_loop_image_equality: {iterations} iterations in {elapsed:?} ({:?}/iter), same={same_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_retained_planar_face_point_query(iterations: u32) -> CurveResult<()> {
    let surface = RetainedPlanarSurfaceIdentity2::new(5);
    let face = RetainedPlanarFace2::try_new(
        surface,
        vec![RetainedPlanarTrimLoop2::new(
            surface,
            rectangle(0, 0, 100, 100),
        )],
        vec![RetainedPlanarTrimLoop2::new(
            surface,
            rectangle(40, 40, 60, 60),
        )],
    )?;
    let point = p(10, 10);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut inside_count = 0_usize;

    for _ in 0..iterations {
        let report = match face.classify_uv_point(surface, &point, &policy)? {
            Classification::Decided(report) => report,
            other => {
                panic!("retained planar face benchmark expected decided report, got {other:?}")
            }
        };
        if report.location() == RetainedPlanarFacePointLocation2::Inside {
            inside_count += black_box(1);
        }
    }

    let elapsed = started.elapsed();
    println!(
        "retained_planar_face_point_query: {iterations} iterations in {elapsed:?} ({:?}/iter), inside={inside_count}",
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
    bench_planar_trim_loop_image_equality(100_000)?;
    bench_retained_planar_face_point_query(100_000)?;
    Ok(())
}
