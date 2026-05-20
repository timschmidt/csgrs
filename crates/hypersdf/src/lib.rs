//! Exact-aware signed-distance and implicit-field carriers.
//!
//! `hypersdf` owns continuous implicit geometry over the Hyper stack. It does
//! not turn primitive-float samples, ray marching, or preview meshes into
//! topology truth. Instead, retained field structure is classified through
//! `hyperlimit` predicates and reported with metric status, exact evidence, and
//! explicit unknowns. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): exactness is a property of the
//! geometric system and its decisions, not only of individual scalar values.

mod expr;
mod facts;
mod handoff;
mod interval;
mod mesh;
mod prepared;
mod primitive;
mod sampling;
mod shader;
mod solver;
mod status;
mod transform;

pub use expr::{SdfCoordinate, SdfExpr};
pub use facts::SdfFacts;
pub use handoff::SdfVoxelHandoffReport;
pub use interval::{SdfInterval, SdfIntervalReport};
pub use mesh::{SdfMeshPreviewBackend, SdfMeshPreviewReport, SdfPreviewNormalStatus};
pub use prepared::PreparedSdf;
pub use primitive::SdfPrimitive;
pub use sampling::{
    SdfGridSamplingError, SdfGridSamplingReport, SdfPreviewGrid, SdfPreviewSample,
    SdfSampleTopologyStatus, SdfSamplingPrecision, SdfSamplingReport,
};
pub use shader::{SdfShaderExportReport, SdfShaderLanguage};
pub use solver::{
    SdfProjectionProposal, SdfProjectionProposalKind, SdfProjectionReplayReport,
    SdfProjectionReplayStatus,
};
pub use status::{
    SdfCellClassificationReport, SdfCellLocation, SdfDomainStatus, SdfEvidenceStatus, SdfFreshness,
    SdfGradientStatus, SdfLipschitzStatus, SdfMetricStatus, SdfPointClassificationReport,
    SdfPointLocation,
};
pub use transform::{SdfTransform, SdfTransformError};

/// Prepare an SDF expression for repeated exact-aware classification.
pub fn prepare(expr: SdfExpr) -> PreparedSdf {
    PreparedSdf::new(expr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlattice::{Matrix4, Vector3};
    use hyperlimit::{Plane3, Point3};
    use hyperreal::Real;

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(r(x), r(y), r(z))
    }

    #[test]
    fn plane_point_classification_uses_oriented_halfspace() {
        let plane = Plane3::new(p(0, 0, 1), r(0));
        let sdf = prepare(SdfExpr::plane(plane));

        assert_eq!(
            sdf.classify_point(&p(0, 0, -1)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(0, 0, 1)).location,
            SdfPointLocation::Outside
        );
    }

    #[test]
    fn sphere_point_classification_is_square_root_free() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(4)));

        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(2, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(3, 0, 0)).location,
            SdfPointLocation::Outside
        );
    }

    #[test]
    fn box_point_classification_preserves_boundary() {
        let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));

        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(1, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(2, 0, 0)).location,
            SdfPointLocation::Outside
        );
    }

    #[test]
    fn slab_point_and_cell_classification_is_exact() {
        let slab = prepare(SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(2)));

        assert_eq!(
            slab.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            slab.classify_point(&p(0, 0, 2)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            slab.classify_point(&p(0, 0, -3)).location,
            SdfPointLocation::Outside
        );

        assert_eq!(
            slab.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            slab.classify_cell(&p(-1, -1, 1), &p(1, 1, 3)).location,
            SdfCellLocation::Boundary
        );
        assert_eq!(
            slab.classify_cell(&p(-1, -1, 3), &p(1, 1, 4)).location,
            SdfCellLocation::ConservativeOutside
        );

        let interval = slab
            .interval_cell(&p(0, 0, -3), &p(0, 0, 1))
            .interval
            .expect("slab interval");
        assert_eq!(interval.lower, r(-2));
        assert_eq!(interval.upper, r(1));
    }

    #[test]
    fn cylinder_point_and_cell_classification_is_exact() {
        let cylinder = prepare(SdfExpr::cylinder(SdfCoordinate::Z, p(0, 0, 0), r(25), r(3)));

        assert_eq!(
            cylinder.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            cylinder.classify_point(&p(3, 4, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            cylinder.classify_point(&p(0, 0, 3)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            cylinder.classify_point(&p(6, 0, 0)).location,
            SdfPointLocation::Outside
        );
        assert_eq!(
            cylinder.classify_point(&p(0, 0, 4)).location,
            SdfPointLocation::Outside
        );

        assert_eq!(
            cylinder.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            cylinder.classify_cell(&p(4, -1, -1), &p(6, 1, 1)).location,
            SdfCellLocation::Boundary
        );
        assert_eq!(
            cylinder.classify_cell(&p(6, 0, 0), &p(7, 1, 1)).location,
            SdfCellLocation::ConservativeOutside
        );

        let interval = cylinder
            .interval_cell(&p(-1, -1, -1), &p(1, 1, 1))
            .interval
            .expect("cylinder interval");
        assert_eq!(interval.lower, r(-3));
        assert_eq!(interval.upper, r(-2));
    }

    #[test]
    fn capsule_point_and_cell_classification_is_exact() {
        let capsule = prepare(SdfExpr::capsule(SdfCoordinate::Z, p(0, 0, 0), r(25), r(3)));

        assert_eq!(
            capsule.classify_point(&p(0, 0, 4)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            capsule.classify_point(&p(0, 0, 8)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            capsule.classify_point(&p(6, 0, 0)).location,
            SdfPointLocation::Outside
        );

        assert_eq!(
            capsule.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            capsule.classify_cell(&p(0, 0, 7), &p(0, 0, 9)).location,
            SdfCellLocation::Boundary
        );

        let interval = capsule
            .interval_cell(&p(0, 0, 7), &p(0, 0, 8))
            .interval
            .expect("capsule interval");
        assert_eq!(interval.lower, r(-9));
        assert_eq!(interval.upper, r(0));
    }

    #[test]
    fn torus_point_and_cell_classification_is_exact() {
        let torus = prepare(SdfExpr::torus(SdfCoordinate::Z, p(0, 0, 0), r(9), r(1)));

        assert_eq!(
            torus.classify_point(&p(3, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            torus.classify_point(&p(4, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            torus.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Outside
        );

        let interval = torus
            .interval_cell(&p(4, 0, 0), &p(4, 0, 0))
            .interval
            .expect("torus interval");
        assert_eq!(interval.lower, r(0));
        assert_eq!(interval.upper, r(0));
    }

    #[test]
    fn invalid_torus_domain_is_unknown_not_outside() {
        let torus = prepare(SdfExpr::torus(SdfCoordinate::Z, p(0, 0, 0), r(0), r(1)));

        assert_eq!(torus.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            torus.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert!(
            torus
                .interval_cell(&p(0, 0, 0), &p(1, 1, 1))
                .interval
                .is_none()
        );
    }

    #[test]
    fn invalid_capsule_domain_is_unknown_not_outside() {
        let capsule = prepare(SdfExpr::capsule(SdfCoordinate::Z, p(0, 0, 0), r(25), r(-3)));

        assert_eq!(capsule.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            capsule.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert!(
            capsule
                .interval_cell(&p(0, 0, 0), &p(1, 1, 1))
                .interval
                .is_none()
        );
    }

    #[test]
    fn invalid_cylinder_domain_is_unknown_not_outside() {
        let cylinder = prepare(SdfExpr::cylinder(
            SdfCoordinate::Z,
            p(0, 0, 0),
            r(-25),
            r(3),
        ));

        assert_eq!(cylinder.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            cylinder.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert!(
            cylinder
                .interval_cell(&p(0, 0, 0), &p(1, 1, 1))
                .interval
                .is_none()
        );
    }

    #[test]
    fn invalid_slab_half_width_is_unknown_not_outside() {
        let slab = prepare(SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(-1)));

        assert_eq!(slab.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            slab.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert!(
            slab.interval_cell(&p(0, 0, 0), &p(1, 1, 1))
                .interval
                .is_none()
        );
    }

    #[test]
    fn primitive_cell_classification_is_conservative() {
        let sphere = prepare(SdfExpr::sphere(p(0, 0, 0), r(100)));
        assert_eq!(
            sphere.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            sphere
                .classify_cell(&p(20, 20, 20), &p(21, 21, 21))
                .location,
            SdfCellLocation::ConservativeOutside
        );
        assert_eq!(
            sphere.classify_cell(&p(9, -1, -1), &p(11, 1, 1)).location,
            SdfCellLocation::Boundary
        );
    }

    #[test]
    fn csg_union_intersection_and_complement_classify_points() {
        let left = SdfExpr::sphere(p(-2, 0, 0), r(4));
        let right = SdfExpr::sphere(p(2, 0, 0), r(4));
        let union = prepare(left.clone().union(right.clone()));
        let intersection = prepare(left.clone().intersection(right.clone()));
        let complement = prepare(left.complement());

        assert_eq!(
            union.classify_point(&p(-2, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            intersection.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            complement.classify_point(&p(-2, 0, 0)).location,
            SdfPointLocation::Outside
        );
    }

    #[test]
    fn batch_point_and_cell_classification_matches_scalar() {
        let sdf = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)));
        let points = [p(0, 0, 0), p(5, 0, 0), p(10, 0, 0)];
        let point_batch = sdf.classify_points(points.iter());
        let point_scalar = points
            .iter()
            .map(|point| sdf.classify_point(point))
            .collect::<Vec<_>>();
        assert_eq!(point_batch, point_scalar);

        let cell_mins = [p(-1, -1, -1), p(4, 4, 4)];
        let cell_maxs = [p(1, 1, 1), p(6, 6, 6)];
        let cell_batch = sdf.classify_cells(cell_mins.iter().zip(cell_maxs.iter()));
        let cell_scalar = cell_mins
            .iter()
            .zip(cell_maxs.iter())
            .map(|(min, max)| sdf.classify_cell(min, max))
            .collect::<Vec<_>>();
        assert_eq!(cell_batch, cell_scalar);
    }

    #[test]
    fn translated_expression_replays_child_queries_exactly() {
        let moved = prepare(SdfExpr::sphere(p(0, 0, 0), r(4)).translate(p(10, 0, 0)));

        assert_eq!(
            moved.classify_point(&p(10, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            moved.classify_point(&p(12, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            moved.classify_cell(&p(9, -1, -1), &p(11, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
    }

    #[test]
    fn affine_transform_replays_points_and_cells_exactly() {
        let swap_xy = Matrix4([
            [r(0), r(1), r(0), r(0)],
            [r(1), r(0), r(0), r(0)],
            [r(0), r(0), r(1), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let sdf = prepare(
            SdfExpr::x()
                .offset(r(3))
                .affine_transform(swap_xy)
                .expect("invertible affine transform"),
        );

        assert_eq!(
            sdf.classify_point(&p(100, 2, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(100, 3, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(100, 4, 0)).location,
            SdfPointLocation::Outside
        );

        let interval = sdf
            .interval_cell(&p(-10, 2, 0), &p(10, 5, 0))
            .interval
            .expect("affine interval");
        assert_eq!(interval.lower, r(-1));
        assert_eq!(interval.upper, r(2));

        assert_eq!(
            sdf.facts().gradient_status,
            SdfGradientStatus::PiecewiseExact
        );
        assert_eq!(sdf.facts().lipschitz_status, SdfLipschitzStatus::LocalOnly);
    }

    #[test]
    fn affine_transform_rejects_non_affine_or_singular_matrices() {
        let non_affine = Matrix4([
            [r(1), r(0), r(0), r(0)],
            [r(0), r(1), r(0), r(0)],
            [r(0), r(0), r(1), r(0)],
            [r(0), r(0), r(1), r(1)],
        ]);
        assert!(SdfExpr::x().affine_transform(non_affine).is_err());

        let singular = Matrix4([
            [r(1), r(0), r(0), r(0)],
            [r(1), r(0), r(0), r(0)],
            [r(0), r(0), r(1), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        assert!(SdfExpr::x().affine_transform(singular).is_err());
    }

    #[test]
    fn interval_reports_cover_plane_and_sphere_cells() {
        let plane = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(0))));
        let plane_interval = plane.interval_cell(&p(-1, -1, -2), &p(1, 1, 3));
        let plane_bounds = plane_interval.interval.expect("plane interval");
        assert_eq!(plane_bounds.lower, r(-2));
        assert_eq!(plane_bounds.upper, r(3));

        let sphere = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
        let sphere_interval = sphere.interval_cell(&p(3, 0, 0), &p(4, 0, 0));
        let sphere_bounds = sphere_interval.interval.expect("sphere interval");
        assert_eq!(sphere_bounds.lower, r(-16));
        assert_eq!(sphere_bounds.upper, r(-9));
    }

    #[test]
    fn aabb_scalar_and_interval_use_exact_max_halfspace_field() {
        let sdf = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)));
        let inside = sdf.classify_point(&p(0, 0, 0));
        let boundary = sdf.classify_point(&p(5, 0, 0));
        let outside = sdf.classify_point(&p(8, 0, 0));

        assert_eq!(inside.scalar_value, Some(r(-5)));
        assert_eq!(boundary.scalar_value, Some(r(0)));
        assert_eq!(outside.scalar_value, Some(r(3)));

        let interval = sdf
            .interval_cell(&p(-1, -2, -3), &p(2, 3, 4))
            .interval
            .expect("aabb interval");
        assert_eq!(interval.lower, r(-6));
        assert_eq!(interval.upper, r(-1));
    }

    #[test]
    fn offset_expression_classifies_through_exact_scalar_signs() {
        let sdf = prepare(SdfExpr::aabb(p(-5, -5, -5), p(5, 5, 5)).offset(r(2)));

        assert_eq!(
            sdf.classify_point(&p(6, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(7, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(8, 0, 0)).location,
            SdfPointLocation::Outside
        );
        assert_eq!(sdf.classify_point(&p(7, 0, 0)).scalar_value, Some(r(0)));

        assert_eq!(
            sdf.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            sdf.interval_cell(&p(7, 0, 0), &p(8, 0, 0))
                .interval
                .expect("offset interval")
                .lower,
            r(0)
        );
    }

    #[test]
    fn constants_and_coordinate_variables_classify_exactly() {
        let constant = prepare(SdfExpr::constant(r(-3)));
        let x = prepare(SdfExpr::x());

        assert_eq!(
            constant.classify_point(&p(10, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            x.classify_point(&p(0, 4, 5)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            x.classify_point(&p(2, 4, 5)).location,
            SdfPointLocation::Outside
        );
        assert_eq!(
            x.classify_point(&p(-2, 4, 5)).location,
            SdfPointLocation::Inside
        );
    }

    #[test]
    fn coordinate_interval_normalizes_reversed_cell_bounds() {
        let y = prepare(SdfExpr::y());
        let interval = y
            .interval_cell(&p(0, 5, 0), &p(0, -2, 0))
            .interval
            .expect("coordinate interval");

        assert_eq!(interval.lower, r(-2));
        assert_eq!(interval.upper, r(5));
        assert_eq!(
            y.classify_cell(&p(0, -3, 0), &p(0, -1, 0)).location,
            SdfCellLocation::ConservativeInside
        );
    }

    #[test]
    fn linear_vector3_field_retains_exact_vector_coefficients() {
        let sdf = prepare(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)));

        assert_eq!(
            sdf.classify_point(&p(1, 0, 1)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(1, 1, 1)).location,
            SdfPointLocation::Inside
        );

        let interval = sdf
            .interval_cell(&p(0, 0, 0), &p(1, 1, 1))
            .interval
            .expect("linear interval");
        assert_eq!(interval.lower, r(-10));
        assert_eq!(interval.upper, r(0));

        assert_eq!(sdf.facts().parameter_exact.len, 4);
        assert_eq!(
            sdf.facts().gradient_status,
            SdfGradientStatus::ExactSymbolic
        );

        let shader = sdf.export_glsl_preview("linear_field", SdfSamplingPrecision::F32);
        assert!(shader.is_complete());
        assert!(shader.source.expect("shader").contains("dot(vec3"));
    }

    #[test]
    fn variable_facts_and_shader_export_are_available() {
        let expr = SdfExpr::z().offset(r(1));
        let sdf = prepare(expr);

        assert_eq!(sdf.facts().node_count, 2);
        assert_eq!(sdf.facts().parameter_exact.len, 1);
        assert_eq!(
            sdf.facts().gradient_status,
            SdfGradientStatus::ExactSymbolic
        );
        assert_eq!(
            sdf.facts().lipschitz_status,
            SdfLipschitzStatus::GlobalExact
        );

        let shader = sdf.export_glsl_preview("z_field", SdfSamplingPrecision::F32);
        assert!(shader.is_complete());
        assert!(shader.source.expect("shader").contains("p.z"));
    }

    #[test]
    fn arithmetic_nodes_classify_and_interval_exactly() {
        let expr = SdfExpr::x()
            .add_expr(SdfExpr::constant(r(2)))
            .mul_expr(SdfExpr::y().sub_expr(SdfExpr::constant(r(1))));
        let sdf = prepare(expr);

        assert_eq!(
            sdf.classify_point(&p(-2, 5, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(sdf.classify_point(&p(0, 3, 0)).scalar_value, Some(r(4)));

        let interval = sdf
            .interval_cell(&p(-1, 2, 0), &p(1, 4, 0))
            .interval
            .expect("arithmetic interval");
        assert_eq!(interval.lower, r(1));
        assert_eq!(interval.upper, r(9));
    }

    #[test]
    fn abs_node_uses_exact_interval_sign_splits() {
        let sdf = prepare(SdfExpr::x().abs().offset(r(2)));

        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(2, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        let interval = sdf
            .interval_cell(&p(-3, 0, 0), &p(1, 0, 0))
            .interval
            .expect("abs interval");
        assert_eq!(interval.lower, r(-2));
        assert_eq!(interval.upper, r(1));
    }

    #[test]
    fn arithmetic_nodes_export_to_shader_preview() {
        let sdf = prepare(
            SdfExpr::x()
                .add_expr(SdfExpr::y())
                .mul_expr(SdfExpr::z().abs()),
        );
        let report = sdf.export_glsl_preview("arith", SdfSamplingPrecision::F32);

        assert!(report.is_complete());
        let source = report.source.expect("shader");
        assert!(source.contains("+"));
        assert!(source.contains("*"));
        assert!(source.contains("abs(p.z)"));
    }

    #[test]
    fn sqrt_node_preserves_domain_and_exact_signs() {
        let sdf = prepare(
            SdfExpr::x()
                .add_expr(SdfExpr::constant(r(4)))
                .sqrt()
                .sub_expr(SdfExpr::constant(r(2))),
        );

        assert_eq!(
            sdf.classify_point(&p(-3, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            sdf.classify_point(&p(5, 0, 0)).location,
            SdfPointLocation::Outside
        );
        assert_eq!(
            sdf.classify_point(&p(-5, 0, 0)).location,
            SdfPointLocation::Unknown
        );

        let interval = sdf
            .interval_cell(&p(0, 0, 0), &p(5, 0, 0))
            .interval
            .expect("sqrt interval");
        assert_eq!(interval.lower, r(0));
        assert_eq!(interval.upper, r(1));

        assert!(
            sdf.interval_cell(&p(-5, 0, 0), &p(0, 0, 0))
                .interval
                .is_none()
        );
    }

    #[test]
    fn sqrt_node_exports_to_shader_preview() {
        let sdf = prepare(SdfExpr::x().abs().sqrt());
        let report = sdf.export_glsl_preview("field", SdfSamplingPrecision::F32);

        assert!(report.is_complete());
        assert!(report.source.expect("shader").contains("sqrt(abs(p.x))"));
    }

    #[test]
    fn offset_amount_is_counted_as_retained_parameter() {
        let base = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));
        let offset = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)).offset(r(3)));

        assert_eq!(
            offset.facts().parameter_exact.len,
            base.facts().parameter_exact.len + 1
        );
        assert_eq!(
            offset.facts().lipschitz_status,
            SdfLipschitzStatus::GlobalExact
        );
    }

    #[test]
    fn structural_facts_count_nodes_and_transform_parameters() {
        let sdf = prepare(
            SdfExpr::sphere(p(0, 0, 0), r(4))
                .translate(p(1, 2, 3))
                .complement(),
        );

        assert_eq!(sdf.facts().node_count, 3);
        assert_eq!(sdf.facts().primitive_count, 1);
        assert_eq!(sdf.facts().transform_count, 1);
        assert!(sdf.facts().all_parameters_exact);
        assert_eq!(sdf.facts().domain_status, SdfDomainStatus::Valid);
        assert_eq!(
            sdf.facts().gradient_status,
            SdfGradientStatus::ExactSymbolic
        );
        assert_eq!(sdf.facts().lipschitz_status, SdfLipschitzStatus::LocalOnly);
    }

    #[test]
    fn preview_sampling_reports_lossy_non_topological_boundary() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
        let points = [p(0, 0, 0), p(3, 4, 0), p(6, 0, 0)];
        let report = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F32);

        assert_eq!(report.sample_count, 3);
        assert_eq!(report.non_finite_count, 0);
        assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
        assert_eq!(report.samples[0].value, Some(-25.0));
        assert_eq!(report.samples[1].value, Some(0.0));
        assert_eq!(report.samples[2].value, Some(11.0));
    }

    #[test]
    fn grid_preview_sampling_constructs_exact_points_then_lowers() {
        let sdf = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(0))));
        let grid = SdfPreviewGrid::new(p(0, 0, -1), p(1, 1, 1), [2, 1, 3]);
        let report = sdf
            .sample_grid_preview(grid.clone(), SdfSamplingPrecision::F64)
            .expect("valid grid");

        assert_eq!(report.grid, grid);
        assert_eq!(report.samples.sample_count, 6);
        assert_eq!(report.samples.non_finite_count, 0);
        assert_eq!(
            report.samples.topology_status,
            SdfSampleTopologyStatus::PreviewOnly
        );
        assert_eq!(report.samples.samples[0].value, Some(-1.0));
        assert_eq!(report.samples.samples[2].value, Some(0.0));
        assert_eq!(report.samples.samples[4].value, Some(1.0));
    }

    #[test]
    fn grid_preview_sampling_rejects_empty_dimensions() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(1)));
        let grid = SdfPreviewGrid::new(p(0, 0, 0), p(1, 1, 1), [1, 0, 1]);

        assert_eq!(
            sdf.sample_grid_preview(grid, SdfSamplingPrecision::F32)
                .expect_err("empty grid"),
            SdfGridSamplingError::EmptyDimension
        );
    }

    #[test]
    fn mesh_preview_report_counts_crossings_but_generates_no_topology() {
        let sdf = prepare(SdfExpr::plane(Plane3::new(p(1, 0, 0), r(0))));
        let grid = SdfPreviewGrid::new(p(-1, 0, 0), p(1, 1, 1), [3, 1, 1]);
        let report = sdf
            .mesh_preview_from_grid(grid, SdfSamplingPrecision::F32)
            .expect("valid preview grid");

        assert_eq!(report.backend, SdfMeshPreviewBackend::SurfaceNets);
        assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
        assert_eq!(report.normal_status, SdfPreviewNormalStatus::NotGenerated);
        assert_eq!(report.crossing_edge_count, 2);
        assert_eq!(report.vertex_count, 0);
        assert_eq!(report.triangle_count, 0);
    }

    #[test]
    fn shader_export_reports_preview_only_glsl_source() {
        let sdf = prepare(
            SdfExpr::sphere(p(0, 0, 0), r(25))
                .translate(p(1, 0, 0))
                .offset(r(2)),
        );
        let report = sdf.export_glsl_preview("field", SdfSamplingPrecision::F32);

        assert!(report.is_complete());
        assert_eq!(report.language, SdfShaderLanguage::Glsl);
        assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
        let source = report.source.expect("shader source");
        assert!(source.contains("float field(vec3 p)"));
        assert!(source.contains("dot("));
        assert!(source.contains("- 2.0"));
    }

    #[test]
    fn shader_export_rejects_invalid_function_names() {
        let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));
        let report = sdf.export_glsl_preview("123-field", SdfSamplingPrecision::F64);

        assert!(report.source.is_none());
        assert_eq!(report.unsupported_nodes.len(), 1);
        assert!(report.unsupported_nodes[0].contains("invalid GLSL"));
    }

    #[test]
    fn projection_replay_accepts_only_boundary_candidates() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(25)));
        let accepted = sdf.replay_projection_proposal(SdfProjectionProposal::new(
            "fixture",
            SdfProjectionProposalKind::ClosestPoint,
            p(10, 0, 0),
            p(5, 0, 0),
        ));
        let rejected = sdf.replay_projection_proposal(SdfProjectionProposal::new(
            "fixture",
            SdfProjectionProposalKind::ClosestPoint,
            p(10, 0, 0),
            p(4, 0, 0),
        ));

        assert_eq!(
            accepted.status,
            SdfProjectionReplayStatus::BoundaryCertified
        );
        assert_eq!(accepted.displacement_squared, r(25));
        assert_eq!(
            rejected.status,
            SdfProjectionReplayStatus::RejectedByClassification
        );
    }

    #[test]
    fn voxel_handoff_summarizes_conservative_cells_without_sampling() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(100)));
        let mins = [p(-1, -1, -1), p(9, -1, -1), p(20, 20, 20)];
        let maxs = [p(1, 1, 1), p(11, 1, 1), p(21, 21, 21)];

        let report = sdf.classify_cells_for_handoff(mins.iter().zip(maxs.iter()));

        assert_eq!(report.cell_count, 3);
        assert_eq!(report.certified_inside_count, 1);
        assert_eq!(report.boundary_count, 1);
        assert_eq!(report.certified_outside_count, 1);
        assert_eq!(report.unknown_count, 0);
        assert!(report.all_cells_classified());
    }

    #[test]
    fn negative_squared_radius_is_invalid_domain_not_outside() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(-1)));

        assert_eq!(sdf.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            sdf.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert_eq!(
            sdf.classify_cell(&p(0, 0, 0), &p(1, 1, 1)).location,
            SdfCellLocation::Unknown
        );
        assert!(
            sdf.interval_cell(&p(0, 0, 0), &p(1, 1, 1))
                .interval
                .is_none()
        );
        assert_eq!(
            sdf.sample_points_preview([p(0, 0, 0)].iter(), SdfSamplingPrecision::F64)
                .non_finite_count,
            1
        );
    }
}
