use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourPointLocation, CurvePolicy, CurveResult,
    LineArcRegion2, Point2, Real, RegionPointLocation,
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

fn sparse_region(contour_count: i32) -> LineArcRegion2 {
    let mut contours = Vec::with_capacity(contour_count as usize);
    for index in 0..contour_count {
        let x = index * 10;
        contours.push(rectangle(x, 0, x + 4, 4));
    }
    LineArcRegion2::from_material_contours(contours)
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

fn bench_batched_contour_bbox_miss(iterations: u32) -> CurveResult<()> {
    let contour = rectangle(0, 0, 10, 10);
    let points = vec![p(100, 100); 64];
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut outside_count = 0_usize;

    for _ in 0..iterations {
        for result in Contour2::classify_points(&contour, black_box(&points), &policy) {
            match result {
                Classification::Decided(ContourPointLocation::Outside) => {
                    outside_count += black_box(1);
                }
                other => panic!("batched contour bbox miss expected outside, got {other:?}"),
            }
        }
    }

    let elapsed = started.elapsed();
    println!(
        "batched_contour_bbox_miss_classify_64: {iterations} batches in {elapsed:?} ({:?}/batch), outside={outside_count}",
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

fn bench_batched_sparse_region(iterations: u32) -> CurveResult<()> {
    let region = sparse_region(120);
    let points = (0..64)
        .map(|index| {
            if index % 2 == 0 {
                p(5_000, 5_000)
            } else {
                p(612, 2)
            }
        })
        .collect::<Vec<_>>();
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut decided_count = 0_usize;

    for _ in 0..iterations {
        for result in LineArcRegion2::classify_points(&region, black_box(&points), &policy) {
            match result {
                Classification::Decided(
                    RegionPointLocation::Inside | RegionPointLocation::Outside,
                ) => decided_count += black_box(1),
                other => panic!("batched sparse region benchmark became non-decided: {other:?}"),
            }
        }
    }

    let elapsed = started.elapsed();
    println!(
        "batched_sparse_region_classify_64: {iterations} batches in {elapsed:?} ({:?}/batch), decided={decided_count}",
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

fn main() -> CurveResult<()> {
    bench_contour_bbox_miss(100_000)?;
    bench_batched_contour_bbox_miss(10_000)?;
    bench_sparse_region_outside(10_000)?;
    bench_sparse_region_single_hit(10_000)?;
    bench_batched_sparse_region(1_000)?;
    bench_sparse_region_filled_area(10_000)?;
    Ok(())
}
