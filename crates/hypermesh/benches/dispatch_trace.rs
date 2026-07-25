mod common;

use hypermesh::clip::clip_polygon;
use hypermesh::{
    BooleanOp, EmberConfig, ExactBvh, HypermeshResult, LocalBsp, Plane, Point3, Real,
    boolean_operation, boolean_operation_with_certified_convex_inputs,
    boolean_triangle_soup_with_certified_convex_inputs, build_polygon_soup,
    classify_polygon_output, convex_hull, convex_hull_with_coplanar_groups,
    convex_hull_with_retained_facts, extract_output, intersect_polygons, make_indicator,
    make_triangle, propagate_wnv, trace_axis_segment, trace_segment,
};

fn trace_workload<T>(name: &str, workload: impl FnOnce() -> HypermeshResult<T>) -> T {
    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(workload)
        .unwrap_or_else(|error| panic!("{name} trace workload must remain certified: {error}"));
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "{name} did not emit an exact-computation path trace"
    );
    println!("{name}: correlation={correlation:?}");
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }
    result
}

fn main() {
    for (name, meshes) in [
        ("cubes", common::cube_pair()),
        ("nested_cubes", common::nested_cube_pair()),
        ("octahedra", common::octahedron_pair()),
    ] {
        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            hyperreal::dispatch_trace::reset();
            let result = hyperreal::dispatch_trace::with_recording(|| {
                boolean_operation(
                    &[meshes[0].as_ref(), meshes[1].as_ref()],
                    op,
                    EmberConfig::default(),
                )
            });
            let output = result.expect("trace workload must remain certified");
            let trace = hyperreal::dispatch_trace::take_trace();
            let correlation = trace.correlation_summary();
            assert!(
                correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
                "{name}/{op:?} did not emit an exact-computation path trace"
            );
            println!(
                "{name}/{op:?}: polygons={}, correlation={:?}",
                output.classifications().len(),
                correlation
            );
            for summary in &trace.dispatch {
                println!(
                    "  {}/{}/{}/{}",
                    summary.layer, summary.operation, summary.path, summary.count
                );
            }
        }
    }

    let nested_tools = common::nested_tool_cubes();
    let nested_tool_refs = nested_tools
        .iter()
        .map(|mesh| mesh.as_ref())
        .collect::<Vec<_>>();
    hyperreal::dispatch_trace::reset();
    let nested_tool_result = hyperreal::dispatch_trace::with_recording(|| {
        boolean_operation(
            &nested_tool_refs,
            BooleanOp::Difference,
            EmberConfig::default(),
        )
    })
    .expect("trace variadic difference must remain certified");
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "nested_tools_5/Difference did not emit an exact-computation path trace"
    );
    println!(
        "nested_tools_5/Difference: polygons={}, correlation={:?}",
        nested_tool_result.classifications().len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let subdivided_cubes = common::subdivided_cube_pair(2);
    hyperreal::dispatch_trace::reset();
    let subdivided_result = hyperreal::dispatch_trace::with_recording(|| {
        boolean_operation(
            &[subdivided_cubes[0].as_ref(), subdivided_cubes[1].as_ref()],
            BooleanOp::Union,
            EmberConfig::default(),
        )
    })
    .expect("subdivided cube union must remain certified");
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "subdivided_cubes_192/Union did not emit an exact-computation path trace"
    );
    println!(
        "subdivided_cubes_192/Union: polygons={}, correlation={:?}",
        subdivided_result.classifications().len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let hull_points = (-8..=8)
        .flat_map(|x| {
            (-8..=8).flat_map(move |y| {
                (-8..=8).map(move |z| Point3::new(Real::from(x), Real::from(y), Real::from(z)))
            })
        })
        .collect::<Vec<_>>();
    hyperreal::dispatch_trace::reset();
    let hull = hyperreal::dispatch_trace::with_recording(|| convex_hull(&hull_points))
        .expect("trace point set must span 3D");
    let trace = hyperreal::dispatch_trace::take_trace();
    let correlation = trace.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "convex_hull/grid_4913 did not emit an exact-computation path trace"
    );
    println!(
        "convex_hull/grid_4913: vertices={}, triangles={}, correlation={:?}",
        hull.positions.len(),
        hull.triangles.len(),
        correlation
    );
    for summary in &trace.dispatch {
        println!(
            "  {}/{}/{}/{}",
            summary.layer, summary.operation, summary.path, summary.count
        );
    }

    let cube_pair = common::cube_pair();
    let cube_refs = [cube_pair[0].as_ref(), cube_pair[1].as_ref()];
    let soup = trace_workload("mesh_build_polygon_soup", || build_polygon_soup(&cube_refs));
    assert_eq!(soup.num_meshes, 2);
    assert!(!soup.polygons.is_empty());

    trace_workload("immediate_certified_convex_polygon", || {
        let result = boolean_operation_with_certified_convex_inputs(
            &cube_refs,
            BooleanOp::Union,
            &[true, true],
            EmberConfig::default(),
        )?;
        let owned = extract_output(&result)?;
        let borrowed = hypermesh::output::extract_output_polygons(&result.output().polygons)?;
        assert_eq!(owned.len(), borrowed.len());
        Ok(owned.len())
    });
    trace_workload("immediate_certified_convex_triangle_soup", || {
        let triangle_soup = boolean_triangle_soup_with_certified_convex_inputs(
            &cube_refs,
            BooleanOp::Union,
            &[true, true],
            EmberConfig::default(),
        )?;
        Ok(triangle_soup.triangles.len())
    });

    let p = |x, y, z| Point3::new(Real::from(x), Real::from(y), Real::from(z));
    let host = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
    let cutter = make_triangle(&p(2, -1, -1), &p(2, 5, -1), &p(2, 2, 1), 1, 0);
    trace_workload("polygon_clip_intersection_bvh_bsp", || {
        let clipped = clip_polygon(&host, &Plane::axis_aligned(0, Real::from(1)))?;
        assert!(clipped.left.is_valid() || clipped.right.is_valid());

        let intersection = intersect_polygons(&host, &cutter, 1)?;
        let mut bsp = LocalBsp::new(&host);
        if let Some(segment) = &intersection.segment {
            bsp.add_segment(segment)?;
        }

        let left = ExactBvh::build(std::slice::from_ref(&host))?;
        let right = ExactBvh::build(std::slice::from_ref(&cutter))?;
        let mut pair_count = 0;
        left.intersect_pairs(&right, |_, _| pair_count += 1)?;
        assert_eq!(pair_count, 1);
        Ok((bsp.node_count(), pair_count))
    });

    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    trace_workload("segment_and_winding", || {
        let axis = trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall.clone()])?;
        let winding = trace_segment(&p(0, 0, 0), &p(2, 0, 0), &[0], &[wall.clone()])?;
        assert!(axis.valid);
        assert_eq!(axis.winding, winding);

        let propagated = propagate_wnv(&[0, 1], -1, &[1, -1])?;
        let indicator = make_indicator(BooleanOp::Difference, 2);
        let classification = classify_polygon_output(&[0, 1], &propagated, &indicator);
        Ok((winding, classification))
    });

    let retained_points = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)];
    let coordinate_ids = vec![
        [0, 1, 2, 10, 20],
        [3, 4, 5, 10, 21],
        [6, 7, 8, 10, 22],
        [9, 10, 11, 10, 23],
    ];
    trace_workload("convex_hull_public_variants", || {
        let grouped = convex_hull_with_coplanar_groups(&retained_points, &[])?;
        let retained = convex_hull_with_retained_facts(&retained_points, &[], &coordinate_ids)?;
        Ok((grouped.triangles.len(), retained.triangles.len()))
    });
}
