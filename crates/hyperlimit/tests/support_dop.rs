use hyperlimit::{
    ConvexPointLocation, HalfspaceFeasibilityReport, Plane3, Point3, PredicateOutcome,
    PredicatePolicy, SupportDop3, SupportDopAabb3ValidationError, SupportDopAxis3,
    SupportDopExpansionKind, SupportDopExpansionReport, SupportDopPlane3ValidationError,
    SupportDopPlaneRelation, SupportDopRelation, SupportDopValidationError, SupportSlab3,
    WitnessedSupportDop3, support_dop3_from_points, witnessed_support_dop3_from_points,
};
use hyperreal::{Rational, Real};

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

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

    let dop = decided(support_dop3_from_points(&axes, &points, POLICY));

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
    let dop = decided(SupportDop3::from_points(&axes, &points, POLICY));

    assert_eq!(
        dop.classify_point(&pi(1, 1, 1), POLICY).value(),
        Some(ConvexPointLocation::Inside)
    );
    assert_eq!(
        dop.classify_point(&pi(0, 2, 1), POLICY).value(),
        Some(ConvexPointLocation::Boundary)
    );
    assert_eq!(
        dop.classify_point(&pi(2, 2, 2), POLICY).value(),
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
        dop.classify_aabb3(&pi(1, 1, 0), &pi(2, 2, 1), POLICY)
            .value(),
        Some(SupportDopRelation::ConservativeOverlap)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(4, 1, 0), &pi(5, 2, 1), POLICY)
            .value(),
        Some(SupportDopRelation::BoundaryTouch)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(5, 1, 0), &pi(6, 2, 1), POLICY)
            .value(),
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

    let report = decided(dop.classify_aabb3_report(&min, &max, POLICY));

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
    assert_eq!(
        report.validate_against_sources(&dop, &min, &max, POLICY),
        Ok(())
    );
    assert_eq!(
        dop.classify_aabb3(&min, &max, POLICY).value(),
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

    let report = decided(dop.classify_aabb3_report(&min, &max, POLICY));

    assert_eq!(report.relation, SupportDopRelation::Separated);
    assert_eq!(report.terminal_slab, Some(0));
    assert_eq!(report.slab_reports.len(), 1);
    assert_eq!(report.slab_reports[0].query_min, Some(r(5)));
    assert_eq!(report.slab_reports[0].query_max, Some(r(6)));
    assert_eq!(
        report.slab_reports[0].relation,
        SupportDopRelation::Separated
    );
    assert_eq!(
        report.validate_against_sources(&dop, &min, &max, POLICY),
        Ok(())
    );
}

#[test]
fn support_dop_aabb_report_records_invalid_retained_slab_as_degenerate() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(4), r(0)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
    ]);
    let min = pi(1, 1, 0);
    let max = pi(2, 2, 1);

    let report = decided(dop.classify_aabb3_report(&min, &max, POLICY));

    assert_eq!(report.relation, SupportDopRelation::Degenerate);
    assert_eq!(report.terminal_slab, Some(0));
    assert_eq!(report.slab_reports.len(), 1);
    assert_eq!(report.slab_reports[0].query_min, None);
    assert_eq!(report.slab_reports[0].query_max, None);
    assert_eq!(
        report.validate_against_sources(&dop, &min, &max, POLICY),
        Ok(())
    );
}

#[test]
fn support_dop_aabb_report_rejects_forged_relations_and_missing_evidence() {
    let dop = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(4)),
        SupportSlab3::new(pi(0, 1, 0), r(0), r(4)),
    ]);
    let min = pi(1, 1, 0);
    let max = pi(2, 2, 1);
    let report = decided(dop.classify_aabb3_report(&min, &max, POLICY));

    let mut forged = report.clone();
    forged.relation = SupportDopRelation::Separated;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::RelationMismatch)
    );

    let mut truncated = report;
    truncated.slab_reports.pop();
    assert_eq!(
        truncated.validate(POLICY),
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
        dop.classify_plane3(&below_plane, POLICY).value(),
        Some(SupportDopPlaneRelation::Below)
    );
    let above_plane = Plane3::new(pi(1, 0, 0), r(1));
    assert_eq!(
        dop.classify_plane3(&above_plane, POLICY).value(),
        Some(SupportDopPlaneRelation::Above)
    );

    let tangent_plane = Plane3::new(pi(1, 0, 0), r(-1));
    let tangent = decided(dop.classify_plane3_report(&tangent_plane, POLICY));
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
        tangent.validate_against_sources(&dop, &tangent_plane, POLICY),
        Ok(())
    );

    let crossing_plane = Plane3::new(pi(1, 1, 1), q(-3, 2));
    let crossing = decided(dop.classify_plane3_report(&crossing_plane, POLICY));
    assert_eq!(crossing.relation, SupportDopPlaneRelation::Intersecting);
    assert_eq!(
        crossing.validate_against_sources(&dop, &crossing_plane, POLICY),
        Ok(())
    );
}

#[test]
fn support_dop_plane_report_detects_invalid_and_infeasible_carriers() {
    let invalid = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(2), r(1))]);
    let plane = Plane3::new(pi(1, 0, 0), r(0));
    let invalid_report = decided(invalid.classify_plane3_report(&plane, POLICY));
    assert_eq!(invalid_report.relation, SupportDopPlaneRelation::Degenerate);
    assert!(invalid_report.carrier_feasibility.is_none());
    assert_eq!(
        invalid_report.validate_against_sources(&invalid, &plane, POLICY),
        Ok(())
    );

    let infeasible = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(0)),
        SupportSlab3::new(pi(1, 0, 0), r(1), r(1)),
    ]);
    let infeasible_report = decided(infeasible.classify_plane3_report(&plane, POLICY));
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
        infeasible_report.validate_against_sources(&infeasible, &plane, POLICY),
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
    let report = decided(dop.classify_plane3_report(&plane, POLICY));
    assert_eq!(report.relation, SupportDopPlaneRelation::Below);

    let mut forged = report.clone();
    forged.relation = SupportDopPlaneRelation::Intersecting;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::RelationMismatch)
    );

    let mut missing_side = report;
    missing_side.below_feasibility = None;
    assert_eq!(
        missing_side.validate(POLICY),
        Err(SupportDopPlane3ValidationError::MissingSideFeasibility)
    );
}

#[test]
fn support_dop_reversed_axis_preserves_same_closed_region() {
    let positive = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(0), r(4))]);
    let negative = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(-1, 0, 0), r(-4), r(0))]);

    for point in [pi(0, 0, 0), pi(2, 0, 0), pi(4, 0, 0), pi(5, 0, 0)] {
        assert_eq!(
            positive.classify_point(&point, POLICY).value(),
            negative.classify_point(&point, POLICY).value()
        );
    }
}

#[test]
fn support_dop_keeps_dyadic_f64_imports_exact_at_boundaries() {
    let tiny = Real::try_from(f64::from_bits(1)).expect("subnormal dyadic imports exactly");
    let axis = p(r(1), r(0), r(0));
    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(axis, r(0), q(1, 2) + tiny.clone())]);

    assert_eq!(
        dop.classify_point(&p(q(1, 2), r(0), r(0)), POLICY).value(),
        Some(ConvexPointLocation::Inside)
    );
    assert_eq!(
        dop.classify_point(&p(q(1, 2) + tiny, r(0), r(0)), POLICY)
            .value(),
        Some(ConvexPointLocation::Boundary)
    );
}

#[test]
fn support_dop_reports_inverted_explicit_slabs_as_degenerate() {
    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(4), r(0))]);

    assert_eq!(
        dop.classify_point(&pi(2, 0, 0), POLICY).value(),
        Some(ConvexPointLocation::Degenerate)
    );
    assert_eq!(
        dop.classify_aabb3(&pi(1, 0, 0), &pi(2, 0, 0), POLICY)
            .value(),
        Some(SupportDopRelation::Degenerate)
    );
}

#[test]
fn witnessed_axis_and_expansion_metadata_contracts_are_replayable() {
    let orthogonal = SupportDopAxis3::orthogonal_axes();
    assert_eq!(
        orthogonal.map(|axis| axis.direction),
        [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    );
    for axis in orthogonal {
        assert!(axis.is_nonzero());
        assert_eq!(axis.validate(), Ok(()));
        assert_eq!(axis.to_point3().structural_facts().known_nonzero_count(), 1);
    }
    assert_eq!(
        SupportDopAxis3::new([0, 0, 0]).validate(),
        Err(SupportDopValidationError::ZeroAxis)
    );

    let kdop26 = SupportDopAxis3::kdop26_axes();
    assert_eq!(kdop26.len(), 13);
    assert!(kdop26.iter().all(|axis| axis.is_nonzero()));
    for (index, axis) in kdop26.iter().enumerate() {
        assert!(!kdop26[..index].contains(axis));
    }

    let exact = SupportDopExpansionReport::exact(1);
    assert_eq!(exact.kind, SupportDopExpansionKind::None);
    assert_eq!(exact.validate(POLICY), Ok(()));

    let rounded = SupportDopExpansionReport::integer_grid_rounding(1, r(2));
    assert_eq!(rounded.kind, SupportDopExpansionKind::IntegerGridRounding);
    assert_eq!(rounded.expanded_slabs, 1);
    assert_eq!(rounded.validate(POLICY), Ok(()));

    let lossy = SupportDopExpansionReport::lossy_adapter(1);
    assert_eq!(lossy.kind, SupportDopExpansionKind::LossyAdapter);
    assert_eq!(lossy.validate(POLICY), Ok(()));

    let points = [pi(-2, 0, 0), pi(4, 0, 0), pi(1, 0, 0)];
    let dop = WitnessedSupportDop3::from_points_with_expansion(
        &points,
        &[SupportDopAxis3::new([1, 0, 0])],
        rounded.clone(),
        POLICY,
    )
    .unwrap();
    assert_eq!(dop.slabs[0].conservative_min_distance(&rounded), r(-4));
    assert_eq!(dop.slabs[0].conservative_max_distance(&rounded), r(6));
    assert_eq!(dop.slabs[0].to_support_slab3().min_witness, Some(0));
    assert_eq!(dop.slabs[0].to_support_slab3().max_witness, Some(1));

    assert_eq!(
        SupportDopExpansionReport::integer_grid_rounding(1, r(-1)).validate(POLICY),
        Err(SupportDopValidationError::NegativeExpansion)
    );
    let mut inconsistent = SupportDopExpansionReport::exact(1);
    inconsistent.expanded_slabs = 1;
    assert_eq!(
        inconsistent.validate(POLICY),
        Err(SupportDopValidationError::ExpansionKindMismatch)
    );
    let mut inconsistent = SupportDopExpansionReport::exact(1);
    inconsistent.expansion = r(1);
    assert_eq!(
        inconsistent.validate(POLICY),
        Err(SupportDopValidationError::ExpansionKindMismatch)
    );
    let mut inconsistent = SupportDopExpansionReport::lossy_adapter(1);
    inconsistent.expanded_slabs = 0;
    assert_eq!(
        inconsistent.validate(POLICY),
        Err(SupportDopValidationError::ExpansionKindMismatch)
    );
}

#[test]
fn witnessed_support_dop_rejects_every_forgeable_source_mismatch() {
    let points = [pi(0, 0, 0), pi(4, 0, 0), pi(2, 0, 0)];
    let axes = [SupportDopAxis3::new([1, 0, 0])];

    assert_eq!(
        WitnessedSupportDop3::from_points(&[], &axes, POLICY),
        Err(SupportDopValidationError::EmptyPointSet)
    );
    assert_eq!(
        WitnessedSupportDop3::from_points(&points, &[], POLICY),
        Err(SupportDopValidationError::EmptyAxisSet)
    );
    assert_eq!(
        WitnessedSupportDop3::from_points_with_expansion(
            &points,
            &axes,
            SupportDopExpansionReport::exact(2),
            POLICY,
        ),
        Err(SupportDopValidationError::ExpansionAxisCountMismatch)
    );

    let dop = witnessed_support_dop3_from_points(&points, &axes, POLICY).unwrap();
    assert_eq!(dop.validate_against_points(&points, POLICY), Ok(()));
    assert_eq!(
        dop.validate_against_points(&[], POLICY),
        Err(SupportDopValidationError::EmptyPointSet)
    );

    let mut forged = dop.clone();
    forged.slabs.clear();
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::EmptyAxisSet)
    );

    let mut forged = dop.clone();
    forged.vertex_count += 1;
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::VertexCountMismatch)
    );

    let mut forged = dop.clone();
    forged.expansion.axis_count += 1;
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::ExpansionAxisCountMismatch)
    );

    let mut forged = dop.clone();
    forged.slabs[0].axis = SupportDopAxis3::new([0, 0, 0]);
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::ZeroAxis)
    );

    let mut forged = dop.clone();
    forged.slabs[0].min.vertex = points.len();
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::WitnessOutOfRange)
    );

    let mut forged = dop.clone();
    forged.slabs[0].min.point = pi(99, 0, 0);
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::WitnessPointMismatch)
    );

    let mut forged = dop.clone();
    forged.slabs[0].min.distance = r(1);
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::WitnessDistanceMismatch)
    );

    let mut forged = dop.clone();
    forged.slabs[0].min = forged.slabs[0].max.clone();
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::WitnessNotMinimal)
    );

    let mut forged = dop;
    forged.slabs[0].max.vertex = 2;
    forged.slabs[0].max.point = points[2].clone();
    forged.slabs[0].max.distance = r(2);
    assert_eq!(
        forged.validate_against_points(&points, POLICY),
        Err(SupportDopValidationError::WitnessNotMaximal)
    );
}

#[test]
fn witnessed_support_dop_refresh_distinguishes_rebuild_extension_and_noop() {
    let initial = [pi(0, 0, 0), pi(4, 0, 0), pi(2, 0, 0)];
    let axes = [SupportDopAxis3::new([1, 0, 0])];

    let mut dop = WitnessedSupportDop3::from_points(&initial, &axes, POLICY).unwrap();
    let mut points = initial.clone();
    points[2] = pi(6, 0, 0);
    let report = dop
        .refresh_for_changed_vertices(&points, &[2], POLICY)
        .unwrap();
    assert_eq!(report.axes_extended, 1);
    assert_eq!(report.axes_rebuilt, 0);
    assert_eq!(report.axes_unchanged, 0);
    assert_eq!(report.invalidated_witness_axes, 0);
    assert_eq!(dop.slabs[0].max.vertex, 2);

    let mut dop = WitnessedSupportDop3::from_points(&initial, &axes, POLICY).unwrap();
    let mut points = initial.clone();
    points[2] = pi(-1, 0, 0);
    let report = dop
        .refresh_for_changed_vertices(&points, &[2], POLICY)
        .unwrap();
    assert_eq!(report.axes_extended, 1);
    assert_eq!(dop.slabs[0].min.vertex, 2);

    let mut dop = WitnessedSupportDop3::from_points(&initial, &axes, POLICY).unwrap();
    let mut points = initial.clone();
    points[2] = pi(3, 0, 0);
    let report = dop
        .refresh_for_changed_vertices(&points, &[2], POLICY)
        .unwrap();
    assert_eq!(report.axes_unchanged, 1);

    let mut dop = WitnessedSupportDop3::from_points(&initial, &axes, POLICY).unwrap();
    let mut points = initial.clone();
    points[1] = pi(5, 0, 0);
    let report = dop
        .refresh_for_changed_vertices(&points, &[1], POLICY)
        .unwrap();
    assert_eq!(report.axes_rebuilt, 1);
    assert_eq!(report.invalidated_witness_axes, 1);

    assert_eq!(
        dop.refresh_for_changed_vertices(&points, &[points.len()], POLICY),
        Err(SupportDopValidationError::ChangedVertexOutOfRange)
    );
    assert_eq!(
        dop.refresh_for_changed_vertices(&points[..2], &[], POLICY),
        Err(SupportDopValidationError::VertexCountMismatch)
    );
    dop.expansion.axis_count += 1;
    assert_eq!(
        dop.refresh_for_changed_vertices(&points, &[], POLICY),
        Err(SupportDopValidationError::ExpansionAxisCountMismatch)
    );
}

#[test]
fn support_dop_aabb_report_validation_rejects_each_retained_evidence_forge() {
    let empty = SupportDop3::from_slabs(Vec::new());
    let empty_report = decided(empty.classify_aabb3_report(&pi(0, 0, 0), &pi(1, 1, 1), POLICY));
    assert_eq!(empty_report.validate(POLICY), Ok(()));
    assert_eq!(
        empty.classify_point(&pi(0, 0, 0), POLICY).value(),
        Some(ConvexPointLocation::Degenerate)
    );
    let mut forged = empty_report;
    forged.relation = SupportDopRelation::ConservativeOverlap;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::EmptyDopRelationMismatch)
    );

    let empty_from_axes = decided(SupportDop3::from_points(&[], &[pi(0, 0, 0)], POLICY));
    assert!(empty_from_axes.slabs().is_empty());
    assert_eq!(empty_from_axes.source_point_count(), 1);
    let empty_from_points = decided(SupportDop3::from_points(&[pi(1, 0, 0)], &[], POLICY));
    assert!(empty_from_points.slabs().is_empty());
    assert_eq!(empty_from_points.source_point_count(), 0);

    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(0), r(4))]);
    let min = pi(1, 0, 0);
    let max = pi(2, 1, 1);
    let report = decided(dop.classify_aabb3_report(&min, &max, POLICY));
    assert_eq!(report.validate(POLICY), Ok(()));

    let mut forged = report.clone();
    forged.slab_reports[0].slab_index = 1;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::SlabIndexMismatch)
    );

    let mut forged = report.clone();
    forged.slab_reports[0].slab.min = r(5);
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::SlabBoundsInvalid)
    );

    let mut forged = report.clone();
    forged.slab_reports[0].query_min = None;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::MissingQueryInterval)
    );

    let mut forged = report.clone();
    forged.slab_reports[0].query_min = Some(r(3));
    forged.slab_reports[0].query_max = Some(r(2));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::QueryIntervalInvalid)
    );

    let mut forged = report.clone();
    forged.slab_reports[0].relation = SupportDopRelation::BoundaryTouch;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::SlabRelationMismatch)
    );

    let mut forged = report.clone();
    forged.terminal_slab = Some(0);
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::TerminalSlabMismatch)
    );

    assert_eq!(
        report.validate_against_sources(&dop, &pi(2, 0, 0), &pi(3, 1, 1), POLICY),
        Err(SupportDopAabb3ValidationError::SourceReplayMismatch)
    );

    let invalid = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(4), r(0))]);
    let invalid_report = decided(invalid.classify_aabb3_report(&min, &max, POLICY));
    let mut forged = invalid_report.clone();
    forged.slab_reports[0].query_min = Some(r(1));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::DegenerateSlabHasQueryInterval)
    );
    let mut forged = invalid_report;
    forged.terminal_slab = None;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopAabb3ValidationError::TerminalSlabMismatch)
    );
}

#[test]
fn support_dop_plane_report_validation_rejects_each_retained_evidence_forge() {
    let plane = Plane3::new(pi(1, 0, 0), r(-2));
    let empty = SupportDop3::from_slabs(Vec::new());
    let empty_report = decided(empty.classify_plane3_report(&plane, POLICY));
    assert_eq!(empty_report.validate(POLICY), Ok(()));
    let mut forged = empty_report;
    forged.relation = SupportDopPlaneRelation::Below;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::EmptyDopRelationMismatch)
    );

    let invalid = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(2), r(1))]);
    let invalid_report = decided(invalid.classify_plane3_report(&plane, POLICY));
    assert_eq!(invalid_report.validate(POLICY), Ok(()));
    let mut forged = invalid_report;
    forged.relation = SupportDopPlaneRelation::Intersecting;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::SlabHalfspaceCountMismatch)
    );

    let dop = SupportDop3::from_slabs(vec![SupportSlab3::new(pi(1, 0, 0), r(0), r(1))]);
    let report = decided(dop.classify_plane3_report(&plane, POLICY));
    assert_eq!(report.validate(POLICY), Ok(()));

    let mut forged = report.clone();
    forged.slab_halfspaces.pop();
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::SlabHalfspaceCountMismatch)
    );

    let mut forged = report.clone();
    forged.carrier_feasibility = None;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::CarrierFeasibilityMismatch)
    );

    let mut forged = report.clone();
    forged
        .carrier_feasibility
        .as_mut()
        .expect("carrier report")
        .witness = Some(pi(99, 0, 0));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::CarrierFeasibilityMismatch)
    );

    let mut forged = report.clone();
    forged
        .below_feasibility
        .as_mut()
        .expect("below report")
        .witness = Some(pi(99, 0, 0));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::BelowFeasibilityMismatch)
    );

    let mut forged = report.clone();
    forged.above_feasibility = Some(HalfspaceFeasibilityReport::feasible(
        pi(99, 0, 0),
        [None; 3],
    ));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::AboveFeasibilityMismatch)
    );

    assert_eq!(
        report.validate_against_sources(&dop, &Plane3::new(pi(1, 0, 0), r(-3)), POLICY,),
        Err(SupportDopPlane3ValidationError::SourceReplayMismatch)
    );

    let infeasible = SupportDop3::from_slabs(vec![
        SupportSlab3::new(pi(1, 0, 0), r(0), r(0)),
        SupportSlab3::new(pi(1, 0, 0), r(1), r(1)),
    ]);
    let infeasible_report = decided(infeasible.classify_plane3_report(&plane, POLICY));
    let mut forged = infeasible_report.clone();
    forged.below_feasibility = Some(HalfspaceFeasibilityReport::infeasible(None));
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::UnexpectedSideFeasibility)
    );
    let mut forged = infeasible_report;
    forged.relation = SupportDopPlaneRelation::Above;
    assert_eq!(
        forged.validate(POLICY),
        Err(SupportDopPlane3ValidationError::RelationMismatch)
    );
}
