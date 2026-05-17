use std::hint::black_box;
use std::time::Instant;

use hypercurve::{
    ArcArcIntersection, BulgeVertex2, CircleCircleRelation, CircularArc2, Classification, Contour2,
    CurvePolicy, CurveResult, CurveString2, LineCircleRelation, LineSeg2, Point2, Real, Region2,
    Segment2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
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

fn arc(start: Point2, end: Point2, center: Point2, clockwise: bool) -> CircularArc2 {
    CircularArc2::try_from_center(start, end, center, clockwise).unwrap()
}

fn line_segment(start: Point2, end: Point2) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(start, end).unwrap())
}

fn bench_arc_arc_case(
    name: &str,
    first: &CircularArc2,
    second: &CircularArc2,
    iterations: u32,
) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_weight = 0_usize;

    for _ in 0..iterations {
        let weight = match first.intersect_arc(second, &policy)? {
            ArcArcIntersection::None => 0,
            ArcArcIntersection::Point(_) => 1,
            ArcArcIntersection::TwoPoints { .. } => 2,
            ArcArcIntersection::Overlap { .. } => 3,
            ArcArcIntersection::Uncertain { reason } => {
                panic!("{name} became uncertain during benchmark: {reason:?}");
            }
        };
        total_weight += black_box(weight);
    }

    let elapsed = started.elapsed();
    println!(
        "{name}: {iterations} iterations in {elapsed:?} ({:?}/iter), total weight={total_weight}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_line_circle_relation(iterations: u32) -> CurveResult<()> {
    let line = LineSeg2::try_new(p(-100, 0), p(100, 0))?;
    let circle = arc(p(25, 0), p(-25, 0), p(0, 0), false);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_weight = 0_usize;

    for _ in 0..iterations {
        let weight = match line.supporting_line_circle_relation(&circle, &policy)? {
            LineCircleRelation::Disjoint => 0,
            LineCircleRelation::Tangent { .. } => 1,
            LineCircleRelation::Secant { .. } => 2,
            LineCircleRelation::Uncertain { reason } => {
                panic!("line_circle_relation became uncertain during benchmark: {reason:?}");
            }
        };
        total_weight += black_box(weight);
    }

    let elapsed = started.elapsed();
    println!(
        "line_circle_relation_secant: {iterations} iterations in {elapsed:?} ({:?}/iter), total weight={total_weight}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_circle_circle_relation(iterations: u32) -> CurveResult<()> {
    let first = arc(p(4, 3), p(4, -3), p(0, 0), true);
    let second = arc(p(4, -3), p(4, 3), p(8, 0), true);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_weight = 0_usize;

    for _ in 0..iterations {
        let weight = match first.circle_relation(&second, &policy)? {
            CircleCircleRelation::Coincident => 3,
            CircleCircleRelation::Disjoint => 0,
            CircleCircleRelation::Tangent { .. } => 1,
            CircleCircleRelation::Secant { .. } => 2,
            CircleCircleRelation::Uncertain { reason } => {
                panic!("circle_circle_relation became uncertain during benchmark: {reason:?}");
            }
        };
        total_weight += black_box(weight);
    }

    let elapsed = started.elapsed();
    println!(
        "circle_circle_relation_secant: {iterations} iterations in {elapsed:?} ({:?}/iter), total weight={total_weight}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_curve_self_contacts(segment_count: i32, iterations: u32) -> CurveResult<()> {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(line_segment(previous, next.clone()));
        previous = next;
    }

    let curve = CurveString2::try_new(segments)?;
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut decided_false_count = 0_usize;

    for _ in 0..iterations {
        match curve.has_self_contacts(&policy)? {
            Classification::Decided(false) => {
                decided_false_count += black_box(1);
            }
            Classification::Decided(true) => {
                panic!("sparse benchmark curve unexpectedly self-contacted");
            }
            Classification::Uncertain(reason) => {
                panic!("sparse benchmark curve became uncertain: {reason:?}");
            }
        }
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_curve_self_contacts_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), decided_false={decided_false_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_sparse_curve_self_contacts(
    segment_count: i32,
    iterations: u32,
) -> CurveResult<()> {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(line_segment(previous, next.clone()));
        previous = next;
    }

    let curve = CurveString2::try_new(segments)?;
    let policy = CurvePolicy::certified();
    let prepared = curve.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut decided_false_count = 0_usize;

    for _ in 0..iterations {
        match prepared.has_self_contacts(&policy)? {
            Classification::Decided(false) => {
                decided_false_count += black_box(1);
            }
            Classification::Decided(true) => {
                panic!("prepared sparse benchmark curve unexpectedly self-contacted");
            }
            Classification::Uncertain(reason) => {
                panic!("prepared sparse benchmark curve became uncertain: {reason:?}");
            }
        }
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_sparse_curve_self_contacts_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), decided_false={decided_false_count}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_curve_string_intersections(segment_count: i32, iterations: u32) -> CurveResult<()> {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(line_segment(previous, next.clone()));
        previous = next;
    }

    let curve = CurveString2::try_new(segments)?;
    let cutter = CurveString2::try_new(vec![line_segment(p(241, -2), p(241, 3))])?;
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_events = 0_usize;

    for _ in 0..iterations {
        let events = curve.intersect_curve_string(&cutter, &policy)?;
        if events.len() != 1 {
            panic!("sparse curve-string benchmark expected one segment-pair event");
        }
        total_events += black_box(events.len());
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_curve_string_intersections_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), total events={total_events}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_sparse_curve_string_intersections(
    segment_count: i32,
    iterations: u32,
) -> CurveResult<()> {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(line_segment(previous, next.clone()));
        previous = next;
    }

    let curve = CurveString2::try_new(segments)?;
    let cutter = CurveString2::try_new(vec![line_segment(p(241, -2), p(241, 3))])?;
    let policy = CurvePolicy::certified();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_events = 0_usize;

    for _ in 0..iterations {
        let events = prepared_curve.intersect_prepared_curve_string(&prepared_cutter, &policy)?;
        if events.len() != 1 {
            panic!("prepared sparse curve-string benchmark expected one segment-pair event");
        }
        total_events += black_box(events.len());
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_sparse_curve_string_intersections_{segment_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), total events={total_events}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_sparse_region_events(contour_count: i32, iterations: u32) -> CurveResult<()> {
    let mut contours = Vec::with_capacity(contour_count as usize);
    for index in 0..contour_count {
        let x = index * 10;
        contours.push(rectangle(x, 0, x + 4, 4));
    }
    let region = Region2::from_material_contours(contours);
    let cutter = Region2::from_material_contours(vec![rectangle(12, -1, 18, 5)]);
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut total_pairs = 0_usize;

    for _ in 0..iterations {
        let events = region.intersect_region(&cutter, &policy)?;
        if events.len() != 1 {
            panic!("sparse region benchmark expected one contour-pair event set");
        }
        total_pairs += black_box(events.len());
    }

    let elapsed = started.elapsed();
    println!(
        "sparse_region_events_{contour_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), total pairs={total_pairs}",
        elapsed / iterations
    );
    Ok(())
}

fn bench_prepared_sparse_region_events(contour_count: i32, iterations: u32) -> CurveResult<()> {
    let mut contours = Vec::with_capacity(contour_count as usize);
    for index in 0..contour_count {
        let x = index * 10;
        contours.push(rectangle(x, 0, x + 4, 4));
    }
    let region = Region2::from_material_contours(contours);
    let cutter = Region2::from_material_contours(vec![rectangle(12, -1, 18, 5)]);
    let policy = CurvePolicy::certified();
    let prepared_region = region.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);
    let started = Instant::now();
    let mut total_pairs = 0_usize;

    for _ in 0..iterations {
        let events = prepared_region.intersect_prepared_region(&prepared_cutter, &policy)?;
        if events.len() != 1 {
            panic!("prepared sparse region benchmark expected one contour-pair event set");
        }
        total_pairs += black_box(events.len());
    }

    let elapsed = started.elapsed();
    println!(
        "prepared_sparse_region_events_{contour_count}: {iterations} iterations in {elapsed:?} ({:?}/iter), total pairs={total_pairs}",
        elapsed / iterations
    );
    Ok(())
}

fn main() -> CurveResult<()> {
    bench_line_circle_relation(100_000)?;
    bench_circle_circle_relation(100_000)?;

    let same_circle_a = arc(p(5, 0), p(-5, 0), p(0, 0), false);
    let same_circle_overlap_b = arc(p(0, 5), p(0, -5), p(0, 0), false);
    bench_arc_arc_case(
        "arc_arc_same_circle_overlap",
        &same_circle_a,
        &same_circle_overlap_b,
        100_000,
    )?;

    let same_circle_endpoint_b = arc(p(5, 0), p(-5, 0), p(0, 0), true);
    bench_arc_arc_case(
        "arc_arc_same_circle_endpoint_pair",
        &same_circle_a,
        &same_circle_endpoint_b,
        100_000,
    )?;

    let crossing_a = arc(p(4, 3), p(4, -3), p(0, 0), true);
    let crossing_b = arc(p(4, -3), p(4, 3), p(8, 0), true);
    bench_arc_arc_case(
        "arc_arc_two_point_crossing",
        &crossing_a,
        &crossing_b,
        100_000,
    )?;
    bench_sparse_curve_self_contacts(160, 1_000)?;
    bench_prepared_sparse_curve_self_contacts(160, 1_000)?;
    bench_sparse_curve_string_intersections(160, 10_000)?;
    bench_prepared_sparse_curve_string_intersections(160, 10_000)?;
    bench_sparse_region_events(120, 1_000)?;
    bench_prepared_sparse_region_events(120, 1_000)?;

    Ok(())
}
