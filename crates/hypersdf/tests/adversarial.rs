use hyperlattice::{Matrix4, Vector3};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{
    SdfCellLocation, SdfCoordinate, SdfDomainStatus, SdfExpr, SdfGradientStatus,
    SdfLipschitzStatus, SdfPointLocation, SdfPreviewGrid, SdfProjectionProposal,
    SdfProjectionProposalKind, SdfProjectionReplayStatus, SdfSampleTopologyStatus,
    SdfSamplingPrecision, prepare,
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
