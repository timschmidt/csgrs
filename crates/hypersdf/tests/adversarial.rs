use hyperlattice::{Matrix4, Vector3};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{
    SdfBatchDispatch, SdfCellLocation, SdfContourProjectionFilterStatus, SdfCoordinate,
    SdfDomainStatus, SdfDualCellTopologyStatus, SdfDualContouringBlocker, SdfDualContouringReport,
    SdfDualContouringSource, SdfDualEdgeRootEvidence, SdfDualVertexPlacementStatus, SdfExpr,
    SdfFiniteDifferenceStencil, SdfFreshness, SdfGradientContourBlocker, SdfGradientContourReport,
    SdfGradientContourSource, SdfGradientStatus, SdfGridSamplingReport, SdfHandoffBlocker,
    SdfHandoffDomain, SdfHandoffReadiness, SdfLipschitzStatus, SdfMetricStatus, SdfPointLocation,
    SdfPreviewGrid, SdfPreviewSample, SdfProjectionProposal, SdfProjectionProposalKind,
    SdfProjectionReplayStatus, SdfSampleTopologyStatus, SdfSamplingPrecision, SdfSamplingReport,
    SdfVoxelCellGrid, SdfVoxelCoordinateSystem, SdfVoxelGridSource, SdfVoxelOccupancy,
    SdfVoxelRowOrder, prepare, prepare_versioned,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn generated_axis_plane_matches_z_order(x in -1000_i32..=1000, y in -1000_i32..=1000, z in -1000_i32..=1000, plane_z in -1000_i32..=1000) {
        let plane = Plane3::new(p(0, 0, 1), r(-plane_z));
        let sdf = prepare(SdfExpr::plane(plane));
        let expected = if z < plane_z {
            SdfPointLocation::Inside
        } else if z == plane_z {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(sdf.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_aabb_point_is_translation_invariant(dx in -100_i32..=100, dy in -100_i32..=100, dz in -100_i32..=100) {
        let base = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)));
        let shifted = prepare(SdfExpr::aabb(p(-5 + dx, -5 + dy, -5 + dz), p(5 + dx, 5 + dy, 5 + dz)));

        prop_assert_eq!(
            base.classify_point(&p(1, 2, 3)).location,
            shifted.classify_point(&p(1 + dx, 2 + dy, 3 + dz)).location
        );
    }

    #[test]
    fn generated_inner_aabb_cell_is_inside_outer(extent in 2_i32..=100, inset in 1_i32..=50) {
        prop_assume!(inset < extent);
        let sdf = prepare(SdfExpr::aabb(p(-extent, -extent, -extent), p(extent, extent, extent)));

        prop_assert_eq!(
            sdf.classify_cell(&p(-inset, -inset, -inset), &p(inset, inset, inset)).location,
            SdfCellLocation::ConservativeInside
        );
    }

    #[test]
    fn generated_arithmetic_preview_matches_exact_scalar(x in -20_i32..=20, y in -20_i32..=20, z in -20_i32..=20) {
        let sdf = prepare(
            SdfExpr::x()
                .sub_expr(SdfExpr::constant(r(1)))
                .mul_expr(SdfExpr::y().add_expr(SdfExpr::constant(r(2))))
                .abs()
                .sub_expr(SdfExpr::z()),
        );
        let point = p(x, y, z);
        let report = sdf.classify_point(&point);
        let samples = sdf.sample_points_preview([point.clone()].iter(), SdfSamplingPrecision::F64);

        prop_assert_eq!(samples.sample_count, 1);
        prop_assert_eq!(
            report.scalar_value.as_ref().and_then(Real::to_f64_lossy),
            samples.samples[0].value
        );
        prop_assert_eq!(
            samples.negative_count + samples.zero_count + samples.positive_count,
            if report.location == SdfPointLocation::Unknown { 0 } else { 1 }
        );
        prop_assert_eq!(
            samples.unknown_sign_count,
            if report.location == SdfPointLocation::Unknown { 1 } else { 0 }
        );
    }

    #[test]
    fn generated_sqrt_of_square_matches_abs(x in -50_i32..=50) {
        let sdf = prepare(
            SdfExpr::x()
                .mul_expr(SdfExpr::x())
                .sqrt()
                .sub_expr(SdfExpr::x().abs()),
        );

        prop_assert_eq!(
            sdf.classify_point(&p(x, 0, 0)).location,
            SdfPointLocation::Boundary
        );
    }

    #[test]
    fn generated_linear_vector3_matches_equivalent_plane(x in -100_i32..=100, y in -100_i32..=100, z in -100_i32..=100) {
        let linear = prepare(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)));
        let plane = prepare(SdfExpr::plane(Plane3::new(p(2, -3, 5), r(-7))));
        let point = p(x, y, z);

        prop_assert_eq!(
            linear.classify_point(&point).location,
            plane.classify_point(&point).location
        );
    }

    #[test]
    fn generated_affine_swap_matches_y_coordinate(x in -100_i32..=100, y in -100_i32..=100, z in -100_i32..=100) {
        let swap_xy = Matrix4([
            [r(0), r(1), r(0), r(0)],
            [r(1), r(0), r(0), r(0)],
            [r(0), r(0), r(1), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let transformed = prepare(
            SdfExpr::x()
                .affine_transform(swap_xy)
                .expect("invertible affine transform"),
        );
        let y_coordinate = prepare(SdfExpr::y());
        let point = p(x, y, z);

        prop_assert_eq!(
            transformed.classify_point(&point).location,
            y_coordinate.classify_point(&point).location
        );
    }

    #[test]
    fn generated_axis_slab_matches_absolute_z_threshold(x in -100_i32..=100, y in -100_i32..=100, z in -100_i32..=100, half_width in 0_i32..=50) {
        let slab = prepare(SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(half_width)));
        let expected = if z.abs() < half_width {
            SdfPointLocation::Inside
        } else if z.abs() == half_width {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(slab.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_z_cylinder_matches_radial_and_height_thresholds(x in -40_i32..=40, y in -40_i32..=40, z in -40_i32..=40, radius in 0_i32..=20, half_height in 0_i32..=20) {
        let cylinder = prepare(SdfExpr::cylinder(
            SdfCoordinate::Z,
            p(0, 0, 0),
            r(radius * radius),
            r(half_height),
        ));
        let radial_squared = x * x + y * y;
        let radius_squared = radius * radius;
        let expected = if radial_squared < radius_squared && z.abs() < half_height {
            SdfPointLocation::Inside
        } else if radial_squared <= radius_squared && z.abs() <= half_height {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(cylinder.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_z_capsule_matches_segment_distance_thresholds(x in -40_i32..=40, y in -40_i32..=40, z in -40_i32..=40, radius in 0_i32..=20, half_length in 0_i32..=20) {
        let capsule = prepare(SdfExpr::capsule(
            SdfCoordinate::Z,
            p(0, 0, 0),
            r(radius * radius),
            r(half_length),
        ));
        let axial_outside = (z.abs() - half_length).max(0);
        let distance_squared = x * x + y * y + axial_outside * axial_outside;
        let radius_squared = radius * radius;
        let expected = if distance_squared < radius_squared {
            SdfPointLocation::Inside
        } else if distance_squared == radius_squared {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(capsule.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_z_torus_matches_polynomial_equation(x in -20_i32..=20, y in -20_i32..=20, z in -20_i32..=20, major in 1_i32..=10, minor in 0_i32..=10) {
        let major_squared = major * major;
        let minor_squared = minor * minor;
        let torus = prepare(SdfExpr::torus(
            SdfCoordinate::Z,
            p(0, 0, 0),
            r(major_squared),
            r(minor_squared),
        ));
        let radial_squared = x * x + y * y;
        let q = radial_squared + z * z + major_squared - minor_squared;
        let value = q * q - 4 * major_squared * radial_squared;
        let expected = if value < 0 {
            SdfPointLocation::Inside
        } else if value == 0 {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(torus.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_rounded_aabb_matches_squared_distance_threshold(x in -20_i32..=20, y in -20_i32..=20, z in -20_i32..=20, radius in 0_i32..=8) {
        let rounded = prepare(SdfExpr::rounded_aabb(p(-5, -5, -5), p(5, 5, 5), r(radius * radius)));
        let core_value = [
            -5 - x,
            x - 5,
            -5 - y,
            y - 5,
            -5 - z,
            z - 5,
        ]
        .into_iter()
        .max()
        .expect("nonempty core value candidates");
        let value = if core_value <= 0 {
            core_value - radius * radius
        } else {
            let dx = if x < -5 { -5 - x } else if x > 5 { x - 5 } else { 0 };
            let dy = if y < -5 { -5 - y } else if y > 5 { y - 5 } else { 0 };
            let dz = if z < -5 { -5 - z } else if z > 5 { z - 5 } else { 0 };
            dx * dx + dy * dy + dz * dz - radius * radius
        };
        let expected = if value < 0 {
            SdfPointLocation::Inside
        } else if value == 0 {
            SdfPointLocation::Boundary
        } else {
            SdfPointLocation::Outside
        };

        prop_assert_eq!(rounded.classify_point(&p(x, y, z)).location, expected);
    }

    #[test]
    fn generated_sine_integer_pi_multiples_are_exact_boundaries(k in -20_i32..=20) {
        let sdf = prepare(SdfExpr::constant(r(k) * Real::pi()).sin());

        prop_assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );
    }

    #[test]
    fn generated_gradient_batch_matches_scalar_reports(x in -20_i32..=20, y in -20_i32..=20, z in -20_i32..=20) {
        let sdf = prepare(
            SdfExpr::x()
                .mul_expr(SdfExpr::y())
                .add_expr(SdfExpr::z().mul_expr(SdfExpr::z())),
        );
        let points = [p(x, y, z), p(x + 1, y - 1, z + 2)];

        prop_assert_eq!(
            sdf.gradient_points(points.iter()),
            points
                .iter()
                .map(|point| sdf.gradient_point(point))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn generated_normal_batch_matches_scalar_reports(x in -20_i32..=20, y in -20_i32..=20, z in -20_i32..=20) {
        let sdf = prepare(SdfExpr::linear(
            Vector3([r(2), r(-3), r(5)]),
            r(11),
        ));
        let points = [p(x, y, z), p(x - 2, y + 3, z - 4)];

        prop_assert_eq!(
            sdf.normal_points(points.iter()),
            points
                .iter()
                .map(|point| sdf.normal_point(point))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn generated_prepared_point_batch_report_matches_scalar_replay(xs in proptest::collection::vec(-20_i32..=20, 0..16)) {
        let sdf = prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(3))).abs());
        let points = xs.iter().map(|x| p(*x, *x - 1, -*x)).collect::<Vec<_>>();
        let report = sdf.classify_points_report(points.iter());

        prop_assert_eq!(report.dispatch, SdfBatchDispatch::ScalarReplay);
        prop_assert_eq!(report.cache_payoff.query_count, points.len());
        prop_assert_eq!(
            report.cache_payoff.avoided_fact_rebuild_count,
            points.len().saturating_sub(1)
        );
        prop_assert!(report.is_self_consistent());
        prop_assert_eq!(report.reports, sdf.classify_points(points.iter()));
    }

    #[test]
    fn generated_prepared_cell_batch_report_matches_scalar_replay(offsets in proptest::collection::vec(-20_i32..=20, 0..16)) {
        let sdf = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)));
        let cells = offsets
            .iter()
            .map(|offset| {
                (
                    p(*offset, *offset, *offset),
                    p(*offset + 1, *offset + 1, *offset + 1),
                )
            })
            .collect::<Vec<_>>();
        let report = sdf.classify_cells_report(cells.iter().map(|(min, max)| (min, max)));

        prop_assert_eq!(report.dispatch, SdfBatchDispatch::ScalarReplay);
        prop_assert_eq!(report.cache_payoff.query_count, cells.len());
        prop_assert_eq!(
            report.cache_payoff.avoided_fact_rebuild_count,
            cells.len().saturating_sub(1)
        );
        prop_assert!(report.is_self_consistent());
        prop_assert_eq!(
            report.reports,
            sdf.classify_cells(cells.iter().map(|(min, max)| (min, max)))
        );
    }

    #[test]
    fn generated_handoff_package_grid_requirement_follows_sample_finiteness(x0 in -10_i32..=10, step in 1_i32..=4) {
        let sdf = prepare(SdfExpr::x());
        let grid = SdfPreviewGrid::new(p(x0, 0, 0), p(step, 1, 1), [3, 1, 1]);
        let samples = sdf
            .sample_grid_preview(grid, SdfSamplingPrecision::F32)
            .expect("valid generated grid");
        let package = sdf.handoff_package().with_grid_samples(samples);
        let requirement = package.require_domain(SdfHandoffDomain::SampledGridPreview);

        prop_assert_eq!(requirement.readiness, SdfHandoffReadiness::Ready);
        prop_assert!(requirement.blockers.is_empty());
        prop_assert!(package.is_self_consistent());
    }

    #[test]
    fn generated_handoff_package_voxel_requirement_reports_unknown_cells(offset in -10_i32..=10) {
        let sdf = prepare(SdfExpr::x().tan());
        let cells = [(p(offset, 0, 0), p(offset + 1, 0, 0))];
        let package = sdf
            .handoff_package()
            .with_voxel_cells(sdf.classify_cells_for_handoff(cells.iter().map(|(min, max)| (min, max))));
        let requirement = package.require_domain(SdfHandoffDomain::VoxelCells);

        prop_assert_eq!(requirement.readiness, SdfHandoffReadiness::Blocked);
        prop_assert!(requirement.blockers.contains(&SdfHandoffBlocker::UnknownDomain));
        prop_assert!(requirement.blockers.contains(&SdfHandoffBlocker::UnknownVoxelCells));
    }

    #[test]
    fn generated_hypervoxel_grid_handoff_matches_cell_classification(extent in 1_i32..=4, ox in -4_i32..=4, oy in -4_i32..=4, oz in -4_i32..=4) {
        let sdf = prepare(SdfExpr::aabb(p(ox, oy, oz), p(ox + extent, oy + extent, oz + extent)));
        let grid = SdfVoxelCellGrid::new(p(ox, oy, oz), p(1, 1, 1), [4, 4, 4])
            .with_source(SdfVoxelGridSource::new("generated:aabb", extent as u64));
        let report = sdf
            .classify_voxel_grid_for_handoff(grid)
            .expect("positive generated grid");

        prop_assert!(report.frame.hypervoxel_frame_ready);
        prop_assert_eq!(report.frame.depth, Some(2));
        prop_assert!(report.is_self_consistent());
        for cell in &report.cells {
            let expected = SdfVoxelOccupancy::from_cell_location(cell.classification.location);
            prop_assert_eq!(cell.occupancy, expected);
        }
        prop_assert_eq!(
            report.as_voxel_handoff_report().cells,
            report
                .cells
                .iter()
                .map(|cell| cell.classification.clone())
                .collect::<Vec<_>>()
        );
        let manifest = report.interchange_manifest();
        let interchange = report.interchange_report(&manifest);
        prop_assert_eq!(manifest.coordinate_system, SdfVoxelCoordinateSystem::HyperGrid);
        prop_assert_eq!(manifest.row_order, SdfVoxelRowOrder::ZMajorYThenXFast);
        prop_assert_eq!(manifest.declared_depth, Some(2));
        prop_assert_eq!(manifest.declared_dimensions, [4, 4, 4]);
        prop_assert_eq!(manifest.declared_cell_count, report.cell_count);
        prop_assert!(interchange.exact_interchange_ready);
    }

    #[test]
    fn generated_affine_dual_contouring_cells_are_exact_handoffs(offset in -12_i32..=12, span in 1_i32..=6) {
        let sdf = prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(offset))));
        let grid = SdfPreviewGrid::new(p(offset - span, -1, -1), p(span * 2, 2, 2), [2, 2, 2]);
        let report = sdf
            .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
            .expect("valid generated dual-contouring grid");

        prop_assert!(report.is_self_consistent());
        prop_assert!(report.validation_handoff_ready);
        prop_assert_eq!(report.crossing_edge_count, 4);
        prop_assert_eq!(report.exact_edge_root_count, 4);
        prop_assert_eq!(report.active_cell_count, 1);
        prop_assert_eq!(
            report.cells[0].placement_status,
            SdfDualVertexPlacementStatus::ExactAffineQefCandidate
        );
    }

    #[test]
    fn generated_dual_contouring_zero_endpoint_is_not_handoff(offset in -12_i32..=12, span in 1_i32..=6) {
        let sdf = prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(offset))));
        let grid = SdfPreviewGrid::new(p(offset, -1, -1), p(span, 2, 2), [2, 2, 2]);
        let report = sdf
            .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
            .expect("valid generated zero-touch grid");

        prop_assert!(report.is_self_consistent());
        prop_assert!(!report.validation_handoff_ready);
        prop_assert!(report.zero_touch_edge_count > 0);
        prop_assert!(report.blockers.contains(&SdfDualContouringBlocker::DegenerateZeroTouch));
    }

    #[test]
    fn generated_gradient_contouring_affine_plane_projects_inside_cell(offset in -12_i32..=12, span in 1_i32..=6) {
        let sdf = prepare(SdfExpr::x().sub_expr(SdfExpr::constant(r(offset))));
        let grid = SdfPreviewGrid::new(p(offset - span, -1, -1), p(span * 2, 2, 2), [2, 2, 2]);
        let report = sdf
            .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
            .expect("valid generated gradient-contouring grid");

        prop_assert!(report.is_self_consistent());
        prop_assert!(!report.validation_handoff_ready);
        prop_assert_eq!(report.active_cell_count, 1);
        prop_assert_eq!(report.kept_projection_count, 1);
        prop_assert!(report.blockers.contains(&SdfGradientContourBlocker::LossyGradientApproximation));
        let projected = report.projections[0].projected_point.expect("kept projection");
        prop_assert_eq!(projected[0], f64::from(offset));
        prop_assert_eq!(
            report.projections[0].filter_status,
            SdfContourProjectionFilterStatus::KeptProposal
        );
    }

    #[test]
    fn generated_coordinate_lipschitz_bound_is_one(a in -50_i32..=50, b in -50_i32..=50) {
        let min_x = a.min(b);
        let max_x = a.max(b);
        let sdf = prepare(SdfExpr::x());
        let report = sdf.lipschitz_cell(&p(min_x, -1, -1), &p(max_x, 1, 1));

        prop_assert!(report.is_certified());
        prop_assert_eq!(report.bound, Some(r(1)));
    }
}

#[test]
fn zero_radius_sphere_keeps_boundary_point_exact() {
    let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(0)));

    assert_eq!(
        sdf.classify_point(&p(0, 0, 0)).location,
        SdfPointLocation::Boundary
    );
    assert_eq!(
        sdf.classify_point(&p(1, 0, 0)).location,
        SdfPointLocation::Outside
    );
}

#[test]
fn degenerate_box_point_cell_is_boundary_not_interior() {
    let sdf = prepare(SdfExpr::aabb(p(0, 0, 0), p(0, 0, 0)));

    assert_eq!(
        sdf.classify_point(&p(0, 0, 0)).location,
        SdfPointLocation::Boundary
    );
    assert_eq!(
        sdf.classify_cell(&p(0, 0, 0), &p(0, 0, 0)).location,
        SdfCellLocation::Boundary
    );
}

#[test]
fn batch_classification_is_not_a_distinct_semantics_path() {
    let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
    let points = [p(0, 0, 0), p(3, 4, 0), p(6, 0, 0)];

    assert_eq!(
        sdf.classify_points(points.iter()),
        points
            .iter()
            .map(|point| sdf.classify_point(point))
            .collect::<Vec<_>>()
    );
}

#[test]
fn translated_plane_interval_matches_shifted_child_interval() {
    let base = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(0))));
    let translated = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(0))).translate(p(0, 0, 5)));

    assert_eq!(
        base.interval_cell(&p(-1, -1, -2), &p(1, 1, 2)).interval,
        translated
            .interval_cell(&p(-1, -1, 3), &p(1, 1, 7))
            .interval
    );
}

#[test]
fn preview_sampling_never_claims_exact_topology() {
    let sdf =
        prepare(SdfExpr::sphere(p(0, 0, 0), r(25)).union(SdfExpr::sphere(p(10, 0, 0), r(25))));
    let points = [p(0, 0, 0), p(5, 0, 0), p(20, 0, 0)];
    let report = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F64);

    assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
    assert_eq!(report.sample_count, points.len());
    assert!(report.samples.iter().all(|sample| sample.value.is_some()));
}

#[test]
fn handoff_counts_match_individual_cell_reports() {
    let sdf = prepare(SdfExpr::aabb(p(-10, -10, -10), p(10, 10, 10)));
    let mins = [p(-1, -1, -1), p(10, 0, 0), p(20, 20, 20)];
    let maxs = [p(1, 1, 1), p(11, 1, 1), p(21, 21, 21)];

    let handoff = sdf.classify_cells_for_handoff(mins.iter().zip(maxs.iter()));
    let scalar = sdf.classify_cells(mins.iter().zip(maxs.iter()));

    assert_eq!(handoff.cells, scalar);
    assert_eq!(handoff.cell_count, scalar.len());
    assert_eq!(handoff.unknown_count, 0);
}

#[test]
fn invalid_sphere_domain_does_not_generate_certified_preview_values() {
    let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(-25)));
    let points = [p(0, 0, 0), p(1, 0, 0)];
    let report = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F64);

    assert_eq!(sdf.facts().domain_status, SdfDomainStatus::Invalid);
    assert_eq!(report.sample_count, 2);
    assert_eq!(report.non_finite_count, 2);
    assert_eq!(report.negative_count, 0);
    assert_eq!(report.zero_count, 0);
    assert_eq!(report.positive_count, 0);
    assert_eq!(report.unknown_sign_count, 2);
    assert!(report.is_self_consistent());
    assert!(report.samples.iter().all(|sample| sample.value.is_none()));
}

#[test]
fn box_facts_expose_piecewise_gradient_and_global_lipschitz_status() {
    let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));

    assert_eq!(
        sdf.facts().gradient_status,
        SdfGradientStatus::PiecewiseExact
    );
    assert_eq!(
        sdf.facts().lipschitz_status,
        SdfLipschitzStatus::GlobalExact
    );
}

#[test]
fn offset_sampling_matches_reported_scalar_values() {
    let sdf = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)).offset(r(2)));
    let points = [p(5, 0, 0), p(7, 0, 0), p(9, 0, 0)];
    let reports = sdf.classify_points(points.iter());
    let samples = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F64);

    for (report, sample) in reports.iter().zip(samples.samples.iter()) {
        assert_eq!(
            report.scalar_value.as_ref().and_then(Real::to_f64_lossy),
            sample.value
        );
    }
}

#[test]
fn grid_preview_matches_explicit_point_preview_order() {
    let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));
    let grid = SdfPreviewGrid::new(p(0, 0, 0), p(1, 1, 1), [2, 2, 1]);
    let points = grid.points().expect("valid grid");
    let point_report = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F64);
    let grid_report = sdf
        .sample_grid_preview(grid, SdfSamplingPrecision::F64)
        .expect("valid grid");

    assert_eq!(grid_report.samples, point_report);
}

#[test]
fn mesh_preview_report_reuses_grid_sample_report() {
    let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(1)));
    let grid = SdfPreviewGrid::new(p(-1, 0, 0), p(1, 1, 1), [3, 1, 1]);
    let grid_report = sdf
        .sample_grid_preview(grid.clone(), SdfSamplingPrecision::F64)
        .expect("valid grid");
    let mesh_report = sdf
        .mesh_preview_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid grid");

    assert_eq!(mesh_report.grid_samples, grid_report);
    assert_eq!(
        mesh_report.topology_status,
        SdfSampleTopologyStatus::PreviewOnly
    );
}

#[test]
fn dual_contouring_affine_plane_builds_exact_qef_handoff() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let report = sdf
        .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid dual-contouring grid");

    assert!(report.is_self_consistent());
    assert_eq!(report.source, SdfDualContouringSource::ExactSdfReplay);
    assert!(report.validation_handoff_ready);
    assert!(report.blockers.is_empty());
    assert_eq!(report.crossing_edge_count, 4);
    assert_eq!(report.exact_edge_root_count, 4);
    assert_eq!(report.sampled_edge_root_count, 0);
    assert_eq!(report.active_cell_count, 1);
    assert!(report.crossings.iter().all(|crossing| {
        crossing.root_evidence == SdfDualEdgeRootEvidence::ExactAffineEdgeRoot
            && crossing.exact_hermite_ready()
    }));

    let cell = &report.cells[0];
    assert_eq!(cell.topology_status, SdfDualCellTopologyStatus::Active);
    assert_eq!(
        cell.placement_status,
        SdfDualVertexPlacementStatus::ExactAffineQefCandidate
    );
    assert_eq!(cell.qef_terms.len(), 4);
    assert_eq!(
        cell.proposal_vertex.as_ref().map(|point| &point.x),
        Some(&r(0))
    );
}

#[test]
fn dual_contouring_signed_samples_stay_lossy_proposals() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let samples = sdf
        .sample_grid_preview(grid, SdfSamplingPrecision::F64)
        .expect("valid sampled grid");
    let report = SdfDualContouringReport::from_signed_grid_samples(samples);

    assert!(report.is_self_consistent());
    assert_eq!(report.source, SdfDualContouringSource::SignedGridSamples);
    assert!(!report.validation_handoff_ready);
    assert!(
        report
            .blockers
            .contains(&SdfDualContouringBlocker::LossyPrimitiveSamples)
    );
    assert_eq!(report.crossing_edge_count, 4);
    assert_eq!(report.exact_edge_root_count, 0);
    assert_eq!(report.sampled_edge_root_count, 4);
    assert_eq!(
        report.cells[0].placement_status,
        SdfDualVertexPlacementStatus::ProposalOnly
    );
}

#[test]
fn dual_contouring_zero_endpoint_reports_degenerate_topology() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(0, -1, -1), p(1, 2, 2), [2, 2, 2]);
    let report = sdf
        .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid zero-touch grid");

    assert!(report.is_self_consistent());
    assert!(!report.validation_handoff_ready);
    assert_eq!(report.zero_touch_edge_count, 8);
    assert!(
        report
            .blockers
            .contains(&SdfDualContouringBlocker::DegenerateZeroTouch)
    );
    assert_eq!(
        report.cells[0].topology_status,
        SdfDualCellTopologyStatus::DegenerateZeroTouch
    );
}

#[test]
fn dual_contouring_nonlinear_edge_crossing_requires_root_replay() {
    let sdf = prepare(
        SdfExpr::x()
            .mul_expr(SdfExpr::x())
            .sub_expr(SdfExpr::constant(r(1))),
    );
    let grid = SdfPreviewGrid::new(p(0, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let report = sdf
        .dual_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid nonlinear grid");

    assert!(report.is_self_consistent());
    assert!(!report.validation_handoff_ready);
    assert_eq!(report.crossing_edge_count, 4);
    assert_eq!(report.exact_edge_root_count, 0);
    assert!(
        report
            .blockers
            .contains(&SdfDualContouringBlocker::UnsupportedEdgeRoot)
    );
    assert_eq!(
        report.cells[0].placement_status,
        SdfDualVertexPlacementStatus::Blocked
    );
}

#[test]
fn dual_contouring_malformed_signed_grid_does_not_panic_or_claim_handoff() {
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let samples = SdfSamplingReport {
        precision: SdfSamplingPrecision::F64,
        metric_status: SdfMetricStatus::SampledApproximation,
        topology_status: SdfSampleTopologyStatus::PreviewOnly,
        freshness: SdfFreshness::Unversioned,
        sample_count: 1,
        non_finite_count: 0,
        negative_count: 1,
        zero_count: 0,
        positive_count: 0,
        unknown_sign_count: 0,
        samples: vec![SdfPreviewSample {
            point: p(-1, -1, -1),
            value: Some(-1.0),
        }],
    };
    let report =
        SdfDualContouringReport::from_signed_grid_samples(SdfGridSamplingReport { grid, samples });

    assert!(!report.validation_handoff_ready);
    assert!(
        report
            .blockers
            .contains(&SdfDualContouringBlocker::InvalidSampleCount)
    );
    assert!(!report.is_self_consistent());
}

#[test]
fn gradient_contouring_plane_reports_projected_sampled_candidates() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let report = sdf
        .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid gradient-contouring grid");

    assert!(report.is_self_consistent());
    assert_eq!(report.source, SdfGradientContourSource::PreparedSdfGrid);
    assert!(!report.validation_handoff_ready);
    assert!(
        report
            .blockers
            .contains(&SdfGradientContourBlocker::LossyGradientApproximation)
    );
    assert_eq!(report.sample_count, 8);
    assert_eq!(report.finite_gradient_count, 8);
    assert_eq!(report.active_cell_count, 1);
    assert_eq!(report.kept_projection_count, 1);
    assert_eq!(report.rejected_projection_count, 0);
    assert_eq!(
        report.projections[0].filter_status,
        SdfContourProjectionFilterStatus::KeptProposal
    );
    assert_eq!(
        report.projections[0].averaged_gradient,
        Some([1.0, 0.0, 0.0])
    );
    assert_eq!(report.projections[0].projected_point, Some([0.0, 0.0, 0.0]));
    assert_eq!(
        report.gradients[0].stencils,
        [
            SdfFiniteDifferenceStencil::Forward,
            SdfFiniteDifferenceStencil::Forward,
            SdfFiniteDifferenceStencil::Forward,
        ]
    );
}

#[test]
fn gradient_contouring_connectivity_is_sampled_proposal_only() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(-2, 0, 0), p(3, 1, 1), [3, 3, 2]);
    let report = sdf
        .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid connected gradient-contouring grid");

    assert!(report.is_self_consistent());
    assert_eq!(report.active_cell_count, 2);
    assert_eq!(report.kept_projection_count, 2);
    assert_eq!(report.connectivity.len(), 1);
    assert_eq!(report.connectivity[0].axis, 1);
    assert_eq!(report.connectivity[0].face_crossing_count, 2);
    assert_eq!(report.connectivity_component_count, 1);
    assert!(!report.validation_handoff_ready);
    assert!(
        report
            .blockers
            .contains(&SdfGradientContourBlocker::ConnectivityProposalOnly)
    );
}

#[test]
fn gradient_contouring_zero_touch_is_filtered_before_projection() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(0, -1, -1), p(1, 2, 2), [2, 2, 2]);
    let report = sdf
        .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid zero-touch gradient-contouring grid");

    assert!(report.is_self_consistent());
    assert_eq!(report.active_cell_count, 1);
    assert_eq!(report.kept_projection_count, 0);
    assert_eq!(report.rejected_projection_count, 1);
    assert_eq!(
        report.projections[0].filter_status,
        SdfContourProjectionFilterStatus::DegenerateZeroTouch
    );
    assert!(
        report
            .blockers
            .contains(&SdfGradientContourBlocker::DegenerateZeroTouch)
    );
}

#[test]
fn gradient_contouring_external_signed_grid_remains_lossy() {
    let sdf = prepare(SdfExpr::x());
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let samples = sdf
        .sample_grid_preview(grid, SdfSamplingPrecision::F64)
        .expect("valid sampled grid");
    let report = SdfGradientContourReport::from_signed_grid_samples(samples);

    assert!(report.is_self_consistent());
    assert_eq!(report.source, SdfGradientContourSource::ExternalSignedGrid);
    assert_eq!(report.kept_projection_count, 1);
    assert!(!report.validation_handoff_ready);
    assert!(
        report
            .blockers
            .contains(&SdfGradientContourBlocker::LossyGradientApproximation)
    );
}

#[test]
fn gradient_contouring_reports_nonfinite_samples_and_bad_steps() {
    let invalid = prepare(SdfExpr::sphere(p(0, 0, 0), r(-1)));
    let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    let nonfinite = invalid
        .gradient_contouring_report_from_grid(grid, SdfSamplingPrecision::F64)
        .expect("valid grid with invalid SDF values");

    assert!(nonfinite.is_self_consistent());
    assert_eq!(nonfinite.non_finite_sample_count, 8);
    assert!(
        nonfinite
            .blockers
            .contains(&SdfGradientContourBlocker::NonFinitePrimitiveSample)
    );
    assert!(
        nonfinite
            .blockers
            .contains(&SdfGradientContourBlocker::UnknownSampleSign)
    );

    let samples = SdfSamplingReport {
        precision: SdfSamplingPrecision::F64,
        metric_status: SdfMetricStatus::SampledApproximation,
        topology_status: SdfSampleTopologyStatus::PreviewOnly,
        freshness: SdfFreshness::Unversioned,
        sample_count: 8,
        non_finite_count: 0,
        negative_count: 4,
        zero_count: 0,
        positive_count: 4,
        unknown_sign_count: 0,
        samples: vec![
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(-1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(-1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(-1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(-1.0),
            },
            SdfPreviewSample {
                point: p(0, 0, 0),
                value: Some(1.0),
            },
        ],
    };
    let bad_step = SdfGradientContourReport::from_signed_grid_samples(SdfGridSamplingReport {
        grid: SdfPreviewGrid::new(p(0, 0, 0), p(0, 1, 1), [2, 2, 2]),
        samples,
    });

    assert!(!bad_step.validation_handoff_ready);
    assert!(
        bad_step
            .blockers
            .contains(&SdfGradientContourBlocker::NonFiniteOrZeroStep)
    );
}

#[test]
fn shader_export_is_not_topology_evidence() {
    let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)).offset(r(1)));
    let report = sdf.export_glsl_preview("sdf_value", SdfSamplingPrecision::F32);

    assert!(report.is_complete());
    assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
    assert_eq!(report.non_finite_constant_count, 0);
}

#[test]
fn projection_replay_preserves_rejected_candidates_for_audit() {
    let sdf = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(0))));
    let proposal = SdfProjectionProposal::new(
        "external-newton-fixture",
        SdfProjectionProposalKind::LevelSetIntersection,
        p(0, 0, 10),
        p(0, 0, 1),
    );
    let report = sdf.replay_projection_proposal(proposal.clone());

    assert_eq!(report.proposal, proposal);
    assert_eq!(
        report.status,
        SdfProjectionReplayStatus::RejectedByClassification
    );
    assert_eq!(report.candidate_report.location, SdfPointLocation::Outside);
}

#[test]
fn stale_prepared_freshness_does_not_change_exact_classification() {
    let current = prepare_versioned(SdfExpr::sphere(p(0, 0, 0), r(25)), 1);
    let stale = current.clone().with_current_source_version(2);
    let point = p(3, 4, 0);

    assert_eq!(
        current.classify_point(&point).location,
        stale.classify_point(&point).location
    );
    assert_eq!(stale.classify_point(&point).freshness, SdfFreshness::Stale);
}

#[test]
fn coordinate_preview_samples_match_input_coordinates() {
    let sdf = prepare(SdfExpr::x().offset(r(1)));
    let points = [p(-2, 0, 0), p(-1, 0, 0), p(3, 0, 0)];
    let samples = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F64);

    assert_eq!(samples.samples[0].value, Some(-3.0));
    assert_eq!(samples.samples[1].value, Some(-2.0));
    assert_eq!(samples.samples[2].value, Some(2.0));
    assert_eq!(
        samples.topology_status,
        SdfSampleTopologyStatus::PreviewOnly
    );
}
