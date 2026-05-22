//! Exact-aware signed-distance and implicit-field carriers.
//!
//! `hypersdf` owns continuous implicit geometry over the Hyper stack. It does
//! not turn primitive-float samples, ray marching, or preview meshes into
//! topology truth. Instead, retained field structure is classified through
//! `hyperlimit` predicates and reported with metric status, exact evidence, and
//! explicit unknowns. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): exactness is a property of the
//! geometric system and its decisions, not only of individual scalar values.

mod batch;
mod dual_contour;
mod expr;
mod facts;
mod gradient;
mod gradient_contour;
mod handoff;
#[cfg(feature = "hypervoxel-adapter")]
mod hypervoxel_adapter;
mod interval;
mod lipschitz;
mod mesh;
mod package;
mod prepared;
mod primitive;
mod sampling;
mod shader;
mod solver;
mod status;
mod transform;
mod voxel;

pub use batch::{
    SdfBatchDispatch, SdfCachePayoffReport, SdfCellBatchClassificationReport,
    SdfPointBatchClassificationReport,
};
pub use dual_contour::{
    SdfDualCellReport, SdfDualCellTopologyStatus, SdfDualContouringBlocker,
    SdfDualContouringReport, SdfDualContouringSource, SdfDualEdgeCrossingKind,
    SdfDualEdgeCrossingRecord, SdfDualEdgeRootEvidence, SdfDualGridEdgeAxis,
    SdfDualGridSampleRecord, SdfDualNormalEvidence, SdfDualQefTerm, SdfDualVertexPlacementStatus,
    SdfGridSampleSign,
};
pub use expr::{SdfCoordinate, SdfExpr};
pub use facts::SdfFacts;
pub use gradient::{SdfGradientReport, SdfNormalReport};
pub use gradient_contour::{
    SdfApproxGradientRecord, SdfContourConnectivityEdge, SdfContourConnectivityStatus,
    SdfContourProjectionFilterStatus, SdfFiniteDifferenceStencil, SdfGradientContourBlocker,
    SdfGradientContourReport, SdfGradientContourSampleRecord, SdfGradientContourSampleSign,
    SdfGradientContourSource, SdfProjectedSurfacePointRecord,
};
pub use handoff::SdfVoxelHandoffReport;
#[cfg(feature = "hypervoxel-adapter")]
pub use hypervoxel_adapter::{SdfHypervoxelAdapterError, continuous_field_manifest_from_sdf};
pub use interval::{SdfInterval, SdfIntervalReport};
pub use lipschitz::SdfLipschitzReport;
pub use mesh::{
    SdfMeshPreviewBackend, SdfMeshPreviewReport, SdfPreviewNormalStatus, SdfPreviewTriangle,
    SdfPreviewVertex,
};
pub use package::{
    SdfHandoffBlocker, SdfHandoffDomain, SdfHandoffPackage, SdfHandoffReadiness,
    SdfHandoffRequirementReport,
};
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
    SdfGradientStatus, SdfLipschitzStatus, SdfMetricStatus, SdfNormalStatus,
    SdfPointClassificationReport, SdfPointLocation,
};
pub use transform::{SdfTransform, SdfTransformError};
pub use voxel::{
    SdfHypervoxelFrameReport, SdfHypervoxelHandoffReport, SdfHypervoxelInterchangeManifest,
    SdfHypervoxelInterchangeReport, SdfVoxelCellGrid, SdfVoxelCellHandoff,
    SdfVoxelCoordinateSystem, SdfVoxelGridError, SdfVoxelGridSource, SdfVoxelLengthUnit,
    SdfVoxelOccupancy, SdfVoxelRowOrder,
};

/// Prepare an SDF expression for repeated exact-aware classification.
pub fn prepare(expr: SdfExpr) -> PreparedSdf {
    PreparedSdf::new(expr)
}

/// Prepare an SDF expression with caller-owned construction-version metadata.
///
/// Versioning is report freshness evidence only. It does not change exact
/// predicate replay, but it lets downstream crates reject stale prepared facts
/// before consuming classification, sampling, solver, or handoff reports.
pub fn prepare_versioned(expr: SdfExpr, source_version: u64) -> PreparedSdf {
    PreparedSdf::new_versioned(expr, source_version)
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
    fn rounded_box_point_and_cell_classification_is_square_root_free() {
        let rounded = prepare(SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), r(4)));

        assert_eq!(
            rounded.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            rounded.classify_point(&p(1, 0, 0)).location,
            SdfPointLocation::Inside
        );
        assert_eq!(
            rounded.classify_point(&p(3, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            rounded.classify_point(&p(4, 0, 0)).location,
            SdfPointLocation::Outside
        );

        assert_eq!(
            rounded.classify_cell(&p(0, 0, 0), &p(0, 0, 0)).location,
            SdfCellLocation::ConservativeInside
        );
        assert_eq!(
            rounded.classify_cell(&p(3, 0, 0), &p(3, 0, 0)).location,
            SdfCellLocation::Boundary
        );

        let interval = rounded
            .interval_cell(&p(3, 0, 0), &p(3, 0, 0))
            .interval
            .expect("rounded box interval");
        assert_eq!(interval.upper, r(0));
    }

    #[test]
    fn zero_radius_rounded_box_preserves_core_boundary() {
        let rounded = prepare(SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), r(0)));

        assert_eq!(
            rounded.classify_point(&p(1, 0, 0)).location,
            SdfPointLocation::Boundary
        );
        assert_eq!(
            rounded.classify_point(&p(2, 0, 0)).location,
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
    fn invalid_rounded_box_domain_is_unknown_not_outside() {
        let rounded = prepare(SdfExpr::rounded_aabb(p(-1, -1, -1), p(1, 1, 1), r(-4)));

        assert_eq!(rounded.facts().domain_status, SdfDomainStatus::Invalid);
        assert_eq!(
            rounded.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );
        assert!(
            rounded
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
    fn prepared_batch_reports_dispatch_and_cache_payoff_without_new_semantics() {
        let sdf = prepare_versioned(
            SdfExpr::sphere(p(0, 0, 0), r(9)).union(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1))),
            3,
        )
        .with_current_source_version(4);
        let points = [p(0, 0, 0), p(3, 0, 0), p(5, 0, 0)];
        let point_report = sdf.classify_points_report(points.iter());

        assert_eq!(point_report.dispatch, SdfBatchDispatch::ScalarReplay);
        assert_eq!(point_report.freshness, SdfFreshness::Stale);
        assert_eq!(point_report.cache_payoff.query_count, points.len());
        assert_eq!(
            point_report.cache_payoff.avoided_fact_rebuild_count,
            points.len() - 1
        );
        assert_eq!(
            point_report.cache_payoff.retained_node_count,
            sdf.facts().node_count
        );
        assert!(point_report.is_self_consistent());
        assert_eq!(point_report.reports, sdf.classify_points(points.iter()));

        let cells = [(p(-1, -1, -1), p(1, 1, 1)), (p(5, 5, 5), p(6, 6, 6))];
        let cell_report = sdf.classify_cells_report(cells.iter().map(|(min, max)| (min, max)));

        assert_eq!(cell_report.dispatch, SdfBatchDispatch::ScalarReplay);
        assert_eq!(cell_report.freshness, SdfFreshness::Stale);
        assert_eq!(cell_report.cache_payoff.query_count, cells.len());
        assert_eq!(
            cell_report.cache_payoff.avoided_fact_rebuild_count,
            cells.len() - 1
        );
        assert!(cell_report.is_self_consistent());
        assert_eq!(
            cell_report.reports,
            sdf.classify_cells(cells.iter().map(|(min, max)| (min, max)))
        );
    }

    #[test]
    fn prepared_batch_reports_empty_queries_without_fake_payoff() {
        let sdf = prepare(SdfExpr::x());
        let points: [Point3; 0] = [];
        let point_report = sdf.classify_points_report(points.iter());
        assert_eq!(point_report.cache_payoff.query_count, 0);
        assert_eq!(point_report.cache_payoff.avoided_fact_rebuild_count, 0);
        assert!(point_report.reports.is_empty());
        assert!(point_report.is_self_consistent());

        let cells: [(Point3, Point3); 0] = [];
        let cell_report = sdf.classify_cells_report(cells.iter().map(|(min, max)| (min, max)));
        assert_eq!(cell_report.cache_payoff.query_count, 0);
        assert_eq!(cell_report.cache_payoff.avoided_fact_rebuild_count, 0);
        assert!(cell_report.reports.is_empty());
        assert!(cell_report.is_self_consistent());
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
    fn gradient_reports_exact_symbolic_vectors_where_certified() {
        let plane = prepare(SdfExpr::plane(Plane3::new(p(2, -3, 5), r(7))));
        assert_eq!(
            plane.gradient_point(&p(10, 20, 30)).gradient,
            Some(Vector3([r(2), r(-3), r(5)]))
        );

        let sphere = prepare(SdfExpr::sphere(p(1, 2, 3), r(25)));
        assert_eq!(
            sphere.gradient_point(&p(4, 6, 8)).gradient,
            Some(Vector3([r(6), r(8), r(10)]))
        );

        let arithmetic = prepare(
            SdfExpr::x()
                .mul_expr(SdfExpr::y())
                .add_expr(SdfExpr::z().mul_expr(SdfExpr::z())),
        );
        assert_eq!(
            arithmetic.gradient_point(&p(2, 3, 4)).gradient,
            Some(Vector3([r(3), r(2), r(8)]))
        );
    }

    #[test]
    fn gradient_reports_unknown_at_csg_ties_and_piecewise_branches() {
        let tied = prepare(SdfExpr::x().union(SdfExpr::y()));
        let report = tied.gradient_point(&p(0, 0, 0));
        assert!(!report.is_certified());
        assert!(report.gradient.is_none());

        let box_field = prepare(SdfExpr::aabb(p(-1, -1, -1), p(1, 1, 1)));
        assert!(box_field.gradient_point(&p(0, 0, 0)).gradient.is_none());
    }

    #[test]
    fn normal_reports_require_certified_nonzero_gradients() {
        let plane = prepare(SdfExpr::plane(Plane3::new(p(2, 0, 0), r(0))));
        let normal = plane.normal_point(&p(3, 0, 0));
        assert!(normal.is_certified_direction());
        assert_eq!(normal.normal_status, SdfNormalStatus::ExactDirection);
        assert_eq!(normal.normal, Some(Vector3([r(2), r(0), r(0)])));

        let constant = prepare(SdfExpr::constant(r(7)));
        let zero = constant.normal_point(&p(0, 0, 0));
        assert!(!zero.is_certified_direction());
        assert_eq!(zero.normal_status, SdfNormalStatus::ZeroGradient);
        assert!(zero.normal.is_none());

        let tied = prepare(SdfExpr::x().union(SdfExpr::y()));
        let unknown = tied.normal_point(&p(0, 0, 0));
        assert_eq!(unknown.normal_status, SdfNormalStatus::Unknown);
        assert!(unknown.normal.is_none());
    }

    #[test]
    fn local_lipschitz_reports_certified_exact_bounds_where_supported() {
        let linear = prepare(SdfExpr::linear(Vector3([r(3), r(4), r(0)]), r(0)));
        let linear_bound = linear
            .lipschitz_cell(&p(-10, -10, -10), &p(10, 10, 10))
            .bound;
        assert_eq!(linear_bound, Some(r(5)));

        let sphere = prepare(SdfExpr::sphere(p(0, 0, 0), r(1)));
        let sphere_bound = sphere.lipschitz_cell(&p(0, 0, 0), &p(3, 4, 0)).bound;
        assert_eq!(sphere_bound, Some(r(10)));

        let product = prepare(SdfExpr::x().mul_expr(SdfExpr::y()));
        let product_report = product.lipschitz_cell(&p(0, 0, 0), &p(2, 3, 0));
        assert!(product_report.is_certified());
        assert_eq!(product_report.bound, Some(r(5)));

        let unsupported = prepare(SdfExpr::x().tan()).lipschitz_cell(&p(0, 0, 0), &p(1, 0, 0));
        assert!(!unsupported.is_certified());
        assert!(unsupported.bound.is_none());
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
    fn trig_nodes_use_hyperreal_exact_point_signs_and_unknown_cell_ranges() {
        let pi = Real::pi();
        let half_pi = (pi.clone() / r(2)).expect("nonzero denominator");
        let quarter_pi = (pi.clone() / r(4)).expect("nonzero denominator");

        let sin_pi = prepare(SdfExpr::constant(pi.clone()).sin());
        assert_eq!(
            sin_pi.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );

        let cos_pi_plus_one = prepare(
            SdfExpr::constant(pi.clone())
                .cos()
                .add_expr(SdfExpr::constant(r(1))),
        );
        assert_eq!(
            cos_pi_plus_one.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Boundary
        );

        let tan_quarter_pi_minus_one = prepare(
            SdfExpr::constant(quarter_pi)
                .tan()
                .sub_expr(SdfExpr::constant(r(1))),
        );
        assert_eq!(
            tan_quarter_pi_minus_one
                .classify_point(&p(0, 0, 0))
                .location,
            SdfPointLocation::Boundary
        );

        let tan_pole = prepare(SdfExpr::constant(half_pi).tan());
        assert_eq!(
            tan_pole.classify_point(&p(0, 0, 0)).location,
            SdfPointLocation::Unknown
        );

        let varying_sine = prepare(SdfExpr::x().sin());
        assert_eq!(
            varying_sine
                .classify_cell(&p(-1, 0, 0), &p(1, 0, 0))
                .location,
            SdfCellLocation::Unknown
        );
    }

    #[test]
    fn versioned_prepared_handles_report_current_and_stale_freshness() {
        let current = prepare_versioned(SdfExpr::sphere(p(0, 0, 0), r(4)), 7);
        assert_eq!(current.prepared_source_version(), Some(7));
        assert_eq!(current.current_source_version(), Some(7));
        assert_eq!(current.freshness(), SdfFreshness::Current);
        assert_eq!(
            current.classify_point(&p(2, 0, 0)).freshness,
            SdfFreshness::Current
        );
        assert!(current.classify_point(&p(2, 0, 0)).is_self_consistent());

        let stale = current.clone().with_current_source_version(8);
        assert_eq!(stale.freshness(), SdfFreshness::Stale);
        assert_eq!(
            stale.classify_cell(&p(-1, -1, -1), &p(1, 1, 1)).freshness,
            SdfFreshness::Stale
        );
        assert!(
            stale
                .classify_cell(&p(-1, -1, -1), &p(1, 1, 1))
                .is_self_consistent()
        );
        assert_eq!(
            stale
                .sample_points_preview([p(0, 0, 0)].iter(), SdfSamplingPrecision::F32)
                .freshness,
            SdfFreshness::Stale
        );

        assert_eq!(
            prepare(SdfExpr::x()).classify_point(&p(1, 0, 0)).freshness,
            SdfFreshness::Unversioned
        );
    }

    #[test]
    fn reports_self_validate_summary_fields() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(4)));
        let points = [p(0, 0, 0), p(2, 0, 0), p(3, 0, 0)];
        let samples = sdf.sample_points_preview(points.iter(), SdfSamplingPrecision::F32);
        assert!(samples.is_self_consistent());

        let grid = SdfPreviewGrid::new(p(-2, -2, -2), p(2, 2, 2), [2, 2, 2]);
        let grid_report = sdf
            .sample_grid_preview(grid.clone(), SdfSamplingPrecision::F32)
            .expect("grid report");
        assert!(grid_report.is_self_consistent());
        assert!(
            sdf.mesh_preview_from_grid(grid, SdfSamplingPrecision::F32)
                .expect("mesh preview")
                .is_self_consistent()
        );

        let cells = [(p(-1, -1, -1), p(1, 1, 1)), (p(3, 3, 3), p(4, 4, 4))];
        let handoff = sdf.classify_cells_for_handoff(cells.iter().map(|(min, max)| (min, max)));
        assert!(handoff.is_self_consistent());

        let proposal = SdfProjectionProposal::new(
            "self-validation-fixture",
            SdfProjectionProposalKind::ClosestPoint,
            p(3, 0, 0),
            p(2, 0, 0),
        );
        assert!(
            sdf.replay_projection_proposal(proposal)
                .is_self_consistent()
        );

        assert!(sdf.gradient_point(&p(2, 0, 0)).is_self_consistent());
        assert!(sdf.normal_point(&p(2, 0, 0)).is_self_consistent());
        assert!(
            sdf.lipschitz_cell(&p(-1, -1, -1), &p(1, 1, 1))
                .is_self_consistent()
        );
    }

    #[test]
    fn handoff_package_requires_named_domains_instead_of_optional_guessing() {
        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(4)));
        let grid = SdfPreviewGrid::new(p(-2, -2, -2), p(2, 2, 2), [2, 2, 2]);
        let grid_report = sdf
            .sample_grid_preview(grid.clone(), SdfSamplingPrecision::F32)
            .expect("grid samples");
        let mesh_report = sdf
            .mesh_preview_from_grid(grid, SdfSamplingPrecision::F32)
            .expect("mesh preview");
        let shader_report = sdf.export_glsl_preview("field", SdfSamplingPrecision::F32);
        let cells = [(p(-1, -1, -1), p(1, 1, 1)), (p(3, 3, 3), p(4, 4, 4))];
        let voxel_report =
            sdf.classify_cells_for_handoff(cells.iter().map(|(min, max)| (min, max)));
        let projection = sdf.replay_projection_proposal(SdfProjectionProposal::new(
            "package-fixture",
            SdfProjectionProposalKind::ClosestPoint,
            p(3, 0, 0),
            p(2, 0, 0),
        ));

        let package = sdf
            .handoff_package()
            .with_grid_samples(grid_report)
            .with_mesh_preview(mesh_report)
            .with_shader_preview(shader_report)
            .with_voxel_cells(voxel_report)
            .with_projection_replay(projection);

        assert!(package.is_self_consistent());
        assert!(
            package
                .require_domain(SdfHandoffDomain::ContinuousField)
                .is_ready()
        );
        assert!(
            package
                .require_domain(SdfHandoffDomain::SampledGridPreview)
                .is_ready()
        );
        assert!(
            package
                .require_domain(SdfHandoffDomain::MeshPreview)
                .is_ready()
        );
        assert!(
            package
                .require_domain(SdfHandoffDomain::ShaderPreview)
                .is_ready()
        );
        assert!(
            package
                .require_domain(SdfHandoffDomain::ProjectionReplay)
                .is_ready()
        );
    }

    #[test]
    fn handoff_package_exposes_frame_aware_hypervoxel_grid_domain() {
        let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(3, 3, 3)));
        let ready_grid = SdfVoxelCellGrid::new(p(0, 0, 0), p(1, 1, 1), [2, 2, 2]);
        let ready = sdf
            .handoff_package()
            .with_hypervoxel_grid(
                sdf.classify_voxel_grid_for_handoff(ready_grid)
                    .expect("ready grid"),
            )
            .require_domain(SdfHandoffDomain::HypervoxelGrid);
        assert!(ready.is_ready());

        let non_octree_grid = SdfVoxelCellGrid::new(p(0, 0, 0), p(1, 1, 1), [3, 2, 2]);
        let blocked = sdf
            .handoff_package()
            .with_hypervoxel_grid(
                sdf.classify_voxel_grid_for_handoff(non_octree_grid)
                    .expect("positive non-octree grid"),
            )
            .require_domain(SdfHandoffDomain::HypervoxelGrid);
        assert_eq!(blocked.readiness, SdfHandoffReadiness::Blocked);
        assert!(
            blocked
                .blockers
                .contains(&SdfHandoffBlocker::HypervoxelFrameNotReady)
        );
    }

    #[test]
    fn handoff_package_reports_missing_stale_invalid_and_rejected_domains() {
        let stale = prepare_versioned(SdfExpr::sphere(p(0, 0, 0), r(4)), 1)
            .with_current_source_version(2)
            .handoff_package();
        let stale_requirement = stale.require_domain(SdfHandoffDomain::ContinuousField);
        assert_eq!(stale_requirement.readiness, SdfHandoffReadiness::Blocked);
        assert!(
            stale_requirement
                .blockers
                .contains(&SdfHandoffBlocker::StaleSource)
        );

        let missing = prepare(SdfExpr::x())
            .handoff_package()
            .require_domain(SdfHandoffDomain::MeshPreview);
        assert_eq!(missing.readiness, SdfHandoffReadiness::Missing);
        assert_eq!(missing.blockers, vec![SdfHandoffBlocker::MissingReport]);

        let invalid = prepare(SdfExpr::sphere(p(0, 0, 0), r(-1))).handoff_package();
        let invalid_requirement = invalid.require_domain(SdfHandoffDomain::ContinuousField);
        assert_eq!(invalid_requirement.readiness, SdfHandoffReadiness::Blocked);
        assert!(
            invalid_requirement
                .blockers
                .contains(&SdfHandoffBlocker::InvalidDomain)
        );

        let sdf = prepare(SdfExpr::sphere(p(0, 0, 0), r(4)));
        let rejected_projection = sdf.replay_projection_proposal(SdfProjectionProposal::new(
            "package-fixture",
            SdfProjectionProposalKind::ClosestPoint,
            p(3, 0, 0),
            p(3, 0, 0),
        ));
        let rejected = sdf
            .handoff_package()
            .with_projection_replay(rejected_projection)
            .require_domain(SdfHandoffDomain::ProjectionReplay);
        assert_eq!(rejected.readiness, SdfHandoffReadiness::Blocked);
        assert!(
            rejected
                .blockers
                .contains(&SdfHandoffBlocker::ProjectionNotAccepted)
        );
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
        assert_eq!(report.negative_count, 1);
        assert_eq!(report.zero_count, 1);
        assert_eq!(report.positive_count, 1);
        assert_eq!(report.unknown_sign_count, 0);
        assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
        assert_eq!(report.samples[0].value, Some(-25.0));
        assert_eq!(report.samples[1].value, Some(0.0));
        assert_eq!(report.samples[2].value, Some(11.0));
        assert!(report.is_self_consistent());
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
        assert_eq!(report.samples.negative_count, 2);
        assert_eq!(report.samples.zero_count, 2);
        assert_eq!(report.samples.positive_count, 2);
        assert_eq!(report.samples.unknown_sign_count, 0);
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
    fn mesh_preview_report_counts_crossings_without_topology_on_degenerate_grid() {
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
    fn mesh_preview_runs_fast_surface_nets_as_preview_only_adapter() {
        let sdf = prepare(SdfExpr::plane(Plane3::new(p(1, 0, 0), r(0))));
        let grid = SdfPreviewGrid::new(p(-1, -1, -1), p(1, 1, 1), [3, 3, 3]);
        let report = sdf
            .mesh_preview_from_grid(grid, SdfSamplingPrecision::F32)
            .expect("valid preview grid");

        assert_eq!(report.backend, SdfMeshPreviewBackend::SurfaceNets);
        assert_eq!(report.topology_status, SdfSampleTopologyStatus::PreviewOnly);
        assert_eq!(
            report.normal_status,
            SdfPreviewNormalStatus::PreviewFiniteDifference
        );
        assert!(report.crossing_edge_count > 0);
        assert!(report.vertex_count > 0);
        assert!(report.triangle_count > 0);
        assert_eq!(report.vertex_count, report.vertices.len());
        assert_eq!(report.triangle_count, report.triangles.len());
        assert_eq!(report.non_finite_output_count, 0);
        assert!(report.is_self_consistent());
        assert!(report.vertices.iter().all(|vertex| {
            vertex.position.iter().all(|value| value.is_finite())
                && vertex
                    .normal
                    .is_some_and(|normal| normal.iter().all(|value| value.is_finite()))
        }));
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
    fn hypervoxel_handoff_classifies_exact_cell_grid_and_frame_readiness() {
        let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(3, 3, 3)));
        let grid = SdfVoxelCellGrid::new(p(0, 0, 0), p(1, 1, 1), [2, 2, 2])
            .with_units(SdfVoxelLengthUnit::Millimeter)
            .with_source(SdfVoxelGridSource::new("sdf:box", 1));
        let report = sdf
            .classify_voxel_grid_for_handoff(grid.clone())
            .expect("valid exact voxel grid");

        assert_eq!(report.grid, grid);
        assert_eq!(report.cell_count, 8);
        assert_eq!(report.filled_count, 8);
        assert_eq!(report.empty_count, 0);
        assert_eq!(report.boundary_count, 0);
        assert_eq!(report.unknown_count, 0);
        assert_eq!(report.frame.depth, Some(1));
        assert!(report.frame.hypervoxel_frame_ready);
        assert!(report.hypervoxel_ready());
        assert!(report.is_self_consistent());
        assert_eq!(
            report.as_voxel_handoff_report().cells,
            report
                .cells
                .iter()
                .map(|cell| cell.classification.clone())
                .collect::<Vec<_>>()
        );

        let manifest = report.interchange_manifest();
        assert_eq!(manifest.source, grid.source);
        assert_eq!(
            manifest.coordinate_system,
            SdfVoxelCoordinateSystem::HyperGrid
        );
        assert_eq!(manifest.row_order, SdfVoxelRowOrder::ZMajorYThenXFast);
        assert_eq!(manifest.declared_depth, Some(1));
        assert_eq!(manifest.declared_dimensions, [2, 2, 2]);
        assert_eq!(manifest.declared_cell_count, 8);
        assert!(report.interchange_report(&manifest).exact_interchange_ready);
    }

    #[test]
    fn hypervoxel_handoff_keeps_non_octree_frames_explicit() {
        let sdf = prepare(SdfExpr::x());
        let grid = SdfVoxelCellGrid::new(p(0, 0, 0), p(1, 1, 1), [3, 2, 2]);
        let report = sdf
            .classify_voxel_grid_for_handoff(grid)
            .expect("positive non-octree grid still classifies");

        assert_eq!(report.cell_count, 12);
        assert!(report.frame.positive_step);
        assert!(!report.frame.cubic_dimensions);
        assert!(!report.frame.power_of_two_dimensions);
        assert_eq!(report.frame.depth, None);
        assert!(!report.frame.hypervoxel_frame_ready);
        assert!(!report.hypervoxel_ready());
        assert!(report.is_self_consistent());
    }

    #[test]
    fn hypervoxel_interchange_manifest_blocks_ambiguous_or_mismatched_metadata() {
        let sdf = prepare(SdfExpr::aabb(p(-1, -1, -1), p(3, 3, 3)));
        let report = sdf
            .classify_voxel_grid_for_handoff(SdfVoxelCellGrid::new(
                p(0, 0, 0),
                p(1, 1, 1),
                [2, 2, 2],
            ))
            .expect("valid grid");

        let manifest = report.interchange_manifest();
        let no_source = report.interchange_report(&manifest);
        assert!(!no_source.source_declared);
        assert!(!no_source.exact_interchange_ready);

        let bad = SdfHypervoxelInterchangeManifest {
            source: Some(SdfVoxelGridSource::new("sdf:box", 1)),
            coordinate_system: SdfVoxelCoordinateSystem::Unknown,
            row_order: SdfVoxelRowOrder::Unknown,
            declared_depth: Some(2),
            declared_dimensions: [2, 2, 1],
            declared_cell_count: 7,
        };
        let bad_report = report.interchange_report(&bad);
        assert!(bad_report.source_declared);
        assert!(!bad_report.coordinate_system_declared);
        assert!(!bad_report.row_order_declared);
        assert!(!bad_report.depth_matches_frame);
        assert!(!bad_report.dimensions_match_frame);
        assert!(!bad_report.cell_count_matches);
        assert!(!bad_report.exact_interchange_ready);
    }

    #[test]
    fn hypervoxel_handoff_rejects_empty_or_nonpositive_frames_before_classification() {
        let sdf = prepare(SdfExpr::x());

        assert_eq!(
            sdf.classify_voxel_grid_for_handoff(SdfVoxelCellGrid::new(
                p(0, 0, 0),
                p(1, 1, 1),
                [1, 0, 1]
            ))
            .unwrap_err(),
            SdfVoxelGridError::EmptyDimension
        );
        assert_eq!(
            sdf.classify_voxel_grid_for_handoff(SdfVoxelCellGrid::new(
                p(0, 0, 0),
                p(1, 0, 1),
                [1, 1, 1]
            ))
            .unwrap_err(),
            SdfVoxelGridError::NonPositiveStep { axis: 1 }
        );
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
