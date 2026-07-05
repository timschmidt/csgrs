use hyperlimit::{
    ConvexPointLocation, Plane3, Point3, PredicateOutcome, SupportDop3,
    SupportDopAabb3ValidationError, SupportDopPlane3ValidationError, SupportDopPlaneRelation,
    SupportDopRelation, SupportSlab3, support_dop3_from_points,
};
use hyperreal::{Rational, Real};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(num: i64, den: u64) -> Real {
    Real::from(Rational::fraction(num, den).expect("test rational denominator is nonzero"))
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    p(r(x), r(y), r(z))
}

fn decided<T: Clone>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("test predicate should decide")
}

#[test]
fn support_dop_build_records_exact_support_witnesses() {
    let axes = vec![pi(1, 0, 0), pi(0, 1, 0), pi(1, 1, 0)];
    let points = vec![pi(0, 0, 0), pi(4, 1, 0), pi(1, 5, 0), pi(-2, 3, 0)];

    let dop = decided(support_dop3_from_points(&axes, &points));

    assert_eq!(dop.source_point_count(), points.len());
    assert_eq!(dop.slabs().len(), axes.len());
    assert_eq!(dop.slabs()[0].min_witness, Some(3));
    assert_eq!(dop.slabs()[0].max_witness, Some(1));
    assert_eq!(dop.slabs()[1].min_witness, Some(0));
    assert_eq!(dop.slabs()[1].max_witness, Some(2));
    assert_eq!(dop.slabs()[2].min_witness, Some(0));
    assert_eq!(dop.slabs()[2].max_witness, Some(2));
    assert_eq!(dop.slabs()[2].min, r(0));
    assert_eq!(dop.slabs()[2].max, r(6));
}

#[test]
fn support_dop_classifies_point_inside_boundary_and_outside_exactly() {
    let axes = vec![pi(1, 0, 0), pi(0, 1, 0), pi(0, 0, 1), pi(1, 1, 1)];
    let points = vec![pi(0, 0, 0), pi(4, 0, 0), pi(0, 4, 0), pi(0, 0, 4)];
    let dop = decided(SupportDop3::from_points(&axes, &points));

    assert_eq!(
        dop.classify_point(&pi(1, 1, 1)).value(),
        Some(ConvexPointLocation::Inside)
    );
    assert_eq!(
        dop.classify_point(&pi(0, 2, 1)).value(),
        Some(ConvexPointLocation::Boundary)
    );
    assert_eq!(
        dop.classify_point(&pi(2, 2, 2)).value(),
        Some(ConvexPointLocation::Outside)
    );
}

#[test]
fn support_dop_aabb_relation_uses_exact_projection_intervals() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(4)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
        SupportSlab3::new(pi(1, 1, 0), r(0), r(6)),
    ]);

    assert_eq!(
        dop.classify_aabb3(&pi(1, 1, 0), &pi(2, 2, 1)).value(),
        Some(SupportDopRelation::ConservativeOverlap)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(4, 1, 0), &pi(5, 2, 1)).value(),
        Some(SupportDopRelation::BoundaryTouch)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(5, 1, 0), &pi(6, 2, 1)).value(),
        Some(SupportDopRelation::Separated)
    );
}

#[test]
fn support_dop_aabb_report_retains_boundary_projection_evidence() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(4)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
        SupportSlab3::new(pi(1, 1, 0), r(0), r(6)),
    ]);
    let min = pi(4, 1, 0);
    let max = pi(5, 2, 1);

    let report = decided(dop.classify_aabb3_report(&min, &max));

    assert_eq!(report.relation, SupportDopRelation::BoundaryTouch);
    assert_eq!(report.terminal_slab, None);
    assert_eq!(report.slab_reports.len(), 3);
    assert_eq!(report.slab_reports[0].query_min, Some(r(4)));
    assert_eq!(report.slab_reports[0].query_max, Some(r(5)));
    assert_eq!(
        report.slab_reports[0].relation,
        SupportDopRelation::BoundaryTouch
    );
    assert_eq!(report.slab_reports[2].query_min, Some(r(5)));
    assert_eq!(report.slab_reports[2].query_max, Some(r(7)));
    assert_eq!(report.validate_against_sources(&dop, &min, &max), Ok(()));
    assert_eq!(
        dop.classify_aabb3(&min, &max).value(),
        Some(report.relation)
    );
}

#[test]
fn support_dop_aabb_report_stops_at_first_separating_slab() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(4)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
    ]);
    let min = pi(5, 1, 0);
    let max = pi(6, 2, 1);

    let report = decided(dop.classify_aabb3_report(&min, &max));

    assert_eq!(report.relation, SupportDopRelation::Separated);
    assert_eq!(report.terminal_slab, Some(0));
    assert_eq!(report.slab_reports.len(), 1);
    assert_eq!(report.slab_reports[0].query_min, Some(r(5)));
    assert_eq!(report.slab_reports[0].query_max, Some(r(6)));
    assert_eq!(
        report.slab_reports[0].relation,
        SupportDopRelation::Separated
    );
    assert_eq!(report.validate_against_sources(&dop, &min, &max), Ok(()));
}

#[test]
fn support_dop_aabb_report_records_invalid_retained_slab_as_degenerate() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(4), r(0)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
    ]);
    let min = pi(1, 1, 0);
    let max = pi(2, 2, 1);

    let report = decided(dop.classify_aabb3_report(&min, &max));

    assert_eq!(report.relation, SupportDopRelation::Degenerate);
    assert_eq!(report.terminal_slab, Some(0));
    assert_eq!(report.slab_reports.len(), 1);
    assert_eq!(report.slab_reports[0].query_min, None);
    assert_eq!(report.slab_reports[0].query_max, None);
    assert_eq!(report.validate_against_sources(&dop, &min, &max), Ok(()));
}

#[test]
fn support_dop_aabb_report_rejects_forged_relations_and_missing_evidence() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(4)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
    ]);
    let min = pi(1, 1, 0);
    let max = pi(2, 2, 1);
    let report = decided(dop.classify_aabb3_report(&min, &max));

    let mut forged = report.clone();
    forged.relation = SupportDopRelation::Separated;
    assert_eq!(
        forged.validate(),
        Err(SupportDopAabb3ValidationError::RelationMismatch)
    );

    let mut truncated = report;
    truncated.slab_reports.pop();
    assert_eq!(
        truncated.validate(),
        Err(SupportDopAabb3ValidationError::MissingSlabEvidence)
    );
}

#[test]
fn support_dop_plane_report_classifies_strict_sides_and_intersections() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(1)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(1)),
        SupportSlab3::new(pi(0, 0, 1), r(0), r(1)),
    ]);

    let below_plane = Plane3::new(pi(1, 0, 0), r(-2));
    assert_eq!(
        dop.classify_plane3(&below_plane).value(),
        Some(SupportDopPlaneRelation::Below)
    );
    let above_plane = Plane3::new(pi(1, 0, 0), r(1));
    assert_eq!(
        dop.classify_plane3(&above_plane).value(),
        Some(SupportDopPlaneRelation::Above)
    );

    let tangent_plane = Plane3::new(pi(1, 0, 0), r(-1));
    let tangent = decided(dop.classify_plane3_report(&tangent_plane));
    assert_eq!(tangent.relation, SupportDopPlaneRelation::Intersecting);
    assert_eq!(tangent.slab_halfspaces.len(), 6);
    assert!(
        tangent
            .below_feasibility
            .as_ref()
            .expect("below side report")
            .is_feasible()
    );
    assert!(
        tangent
            .above_feasibility
            .as_ref()
            .expect("above side report")
            .is_feasible()
    );
    assert_eq!(
        tangent.validate_against_sources(&dop, &tangent_plane),
        Ok(())
    );

    let crossing_plane = Plane3::new(pi(1, 1, 1), q(-3, 2));
    let crossing = decided(dop.classify_plane3_report(&crossing_plane));
    assert_eq!(crossing.relation, SupportDopPlaneRelation::Intersecting);
    assert_eq!(
        crossing.validate_against_sources(&dop, &crossing_plane),
        Ok(())
    );
}

#[test]
fn support_dop_plane_report_detects_invalid_and_infeasible_carriers() {
    let invalid = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(2), r(1))]);
    let plane = Plane3::new(pi(1, 0, 0), r(0));
    let invalid_report = decided(invalid.classify_plane3_report(&plane));
    assert_eq!(invalid_report.relation, SupportDopPlaneRelation::Degenerate);
    assert!(invalid_report.carrier_feasibility.is_none());
    assert_eq!(
        invalid_report.validate_against_sources(&invalid, &plane),
        Ok(())
    );

    let infeasible = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(0)),
        SupportSlab3::new(pi(1, 0, 0), r(1), r(1)),
    ]);
    let infeasible_report = decided(infeasible.classify_plane3_report(&plane));
    assert_eq!(
        infeasible_report.relation,
        SupportDopPlaneRelation::Degenerate
    );
    assert!(
        !infeasible_report
            .carrier_feasibility
            .as_ref()
            .expect("carrier feasibility report")
            .is_feasible()
    );
    assert!(infeasible_report.below_feasibility.is_none());
    assert!(infeasible_report.above_feasibility.is_none());
    assert_eq!(
        infeasible_report.validate_against_sources(&infeasible, &plane),
        Ok(())
    );
}

#[test]
fn support_dop_plane_report_rejects_forged_side_evidence() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(1)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(1)),
    ]);
    let plane = Plane3::new(pi(1, 0, 0), r(-2));
    let report = decided(dop.classify_plane3_report(&plane));
    assert_eq!(report.relation, SupportDopPlaneRelation::Below);

    let mut forged = report.clone();
    forged.relation = SupportDopPlaneRelation::Intersecting;
    assert_eq!(
        forged.validate(),
        Err(SupportDopPlane3ValidationError::RelationMismatch)
    );

    let mut missing_side = report;
    missing_side.below_feasibility = None;
    assert_eq!(
        missing_side.validate(),
        Err(SupportDopPlane3ValidationError::MissingSideFeasibility)
    );
}

#[test]
fn support_dop_reversed_axis_preserves_same_closed_region() {
    let positive = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(0), r(4))]);
    let negative = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(-1, 0, 0), r(-4), r(0))]);

    for point in [pi(0, 0, 0), pi(2, 0, 0), pi(4, 0, 0), pi(5, 0, 0)] {
        assert_eq!(
            positive.classify_point(&point).value(),
            negative.classify_point(&point).value()
        );
    }
}

#[test]
fn support_dop_keeps_dyadic_f64_imports_exact_at_boundaries() {
    let tiny = Real::try_from(f64::from_bits(1)).expect("subnormal dyadic imports exactly");
    let axis = p(r(1), r(0), r(0));
    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(axis, r(0), q(1, 2) + tiny.clone())]);

    assert_eq!(
        dop.classify_point(&p(q(1, 2), r(0), r(0))).value(),
        Some(ConvexPointLocation::Inside)
    );
    assert_eq!(
        dop.classify_point(&p(q(1, 2) + tiny, r(0), r(0))).value(),
        Some(ConvexPointLocation::Boundary)
    );
}

#[test]
fn support_dop_reports_inverted_explicit_slabs_as_degenerate() {
    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(4), r(0))]);

    assert_eq!(
        dop.classify_point(&pi(2, 0, 0)).value(),
        Some(ConvexPointLocation::Degenerate)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(1, 0, 0), &pi(2, 0, 0)).value(),
        Some(SupportDopRelation::Degenerate)
    );
}
