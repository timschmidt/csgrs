//! Operation receipts compose without promoting an approximate dependency.
//!
//! Unresolved contacts and curve closure produce fixtures that consume the
//! approximate terminal. Later calls distinguish local decision receipts from
//! inherited construction/topology qualification and missing history. Rebuilt
//! controls explicitly adopt retained coordinates/boundaries as new models;
//! agreement with those controls does not alone prove internal cache admissibility
//! or fidelity to the original source. Chained outcomes preserve the weakest receipt.
#[path = "support/uncertainty.rs"]
/// Proved scalar control and independent unresolved zero for contact fixtures.
mod uncertainty;

use csgrs::{
    ConstructionEvidence, GeometryCertainty, GeometryContext, Real, TriangleMesh, solid,
};
use hyperlattice::Matrix4;
use hypermesh::{BooleanOp, HypermeshError, MeshCertainty};

/// Checks the retained Pythagorean zero and returns the shared strictly unresolved sine-addition zero.
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

/// Exact one-half for overlap placement and interior query coordinates.
fn half() -> Real {
    (Real::one() / Real::from(2)).unwrap()
}

/// Builds an exact X-only affine translation without modifying the scalar expression.
fn translation(x: Real) -> Matrix4 {
    Matrix4::affine_translation([x, Real::zero(), Real::zero()])
}

/// The union of the unit cube with its copy translated by `1 + zero`.
/// Requires strict failure followed by an approximate-terminal receipt, then returns the mesh
/// for tests of its retained construction evidence and reusable geometry facts.
fn approximate_union() -> TriangleMesh {
    let cube = solid::cube(Real::one()).unwrap();
    let touching =
        solid::transform(&cube, &translation(Real::one() + unresolved_zero())).unwrap();
    let strict = solid::boolean_with_context(
        &cube,
        &touching,
        BooleanOp::Union,
        GeometryContext::STRICT,
    );
    assert!(matches!(
        strict,
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    let union = solid::boolean_with_context(
        &cube,
        &touching,
        BooleanOp::Union,
        GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(union.certainty, GeometryCertainty::Approximate512Consumed);
    union.value
}

/// Reads aggregate predicate ancestry from construction evidence, independently of local outcomes.
fn predicate_certainty(mesh: &TriangleMesh) -> Option<MeshCertainty> {
    mesh.construction_evidence().predicate_certainty()
}

#[test]
/// Compares strict Booleans on a qualified union and a newly adopted coordinate copy against two boxes.
/// Requires matching geometry and certified local receipts while ancestry remains different.
/// Also checks qualified approximate fact reuse versus the adopted control and a certified translation
/// whose output still retains approximate ancestry. Rebuilding is not source-fidelity replay.
fn strict_steps_after_an_approximate_mesh_are_local_proofs() {
    let union = approximate_union();
    assert_eq!(
        predicate_certainty(&union),
        Some(MeshCertainty::Approximate512Consumed)
    );
    let rebuilt = TriangleMesh::from_construction(
        union.positions.to_vec(),
        union.triangles.to_vec(),
        ConstructionEvidence::DECLARED_COORDINATES,
    );
    let enclosing = solid::cube(Real::from(3)).unwrap();
    let overlapping = solid::transform(&enclosing, &translation(half())).unwrap();
    for other in [&enclosing, &overlapping] {
        for operation in [
            BooleanOp::Union,
            BooleanOp::Difference,
            BooleanOp::Intersection,
        ] {
            let retained =
                solid::boolean_with_context(&union, other, operation, GeometryContext::STRICT)
                    .unwrap();
            let fresh = solid::boolean_with_context(
                &rebuilt,
                other,
                operation,
                GeometryContext::STRICT,
            )
            .unwrap();
            assert_eq!(retained.certainty, GeometryCertainty::Certified);
            assert_eq!(fresh.certainty, GeometryCertainty::Certified);
            assert_eq!(retained.value, fresh.value);
            // The local proofs agree; only the inherited evidence differs.
            assert_eq!(
                predicate_certainty(&retained.value),
                Some(MeshCertainty::Approximate512Consumed)
            );
            assert_eq!(
                predicate_certainty(&fresh.value),
                Some(MeshCertainty::Certified)
            );
        }
    }

    // Reusing the approximate output's embedded-complex fact consumes the
    // terminal; the rebuilt copy has no such fact to reuse.
    let reused = solid::boolean_with_context(
        &union,
        &enclosing,
        BooleanOp::Difference,
        GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(reused.certainty, GeometryCertainty::Approximate512Consumed);
    let fresh = solid::boolean_with_context(
        &rebuilt,
        &enclosing,
        BooleanOp::Difference,
        GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(fresh.certainty, GeometryCertainty::Certified);

    let moved = solid::transform_with_context(
        &union,
        &translation(Real::from(5)),
        GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(moved.certainty, GeometryCertainty::Certified);
    assert_eq!(
        predicate_certainty(&moved.value),
        Some(MeshCertainty::Approximate512Consumed)
    );
}

#[cfg(feature = "curve")]
#[test]
/// Flattens a qualified mesh and extrudes the region under strict policy, requiring certified local receipts.
/// The resulting mesh must report missing source history and no aggregate predicate certainty,
/// rather than certifying ancestry that this region conversion did not carry.
fn a_curve_conversion_never_reports_certified_mesh_history() {
    let union = approximate_union();
    let section = solid::flatten_with_context(&union, GeometryContext::STRICT).unwrap();
    assert_eq!(section.certainty, GeometryCertainty::Certified);
    let extruded =
        csgrs::curve::try_extrude(&section.value, Real::one(), GeometryContext::STRICT)
            .unwrap();
    assert_eq!(extruded.certainty, GeometryCertainty::Certified);
    // The region carries no construction evidence, so the mesh reports
    // missing history instead of certified history.
    let evidence = extruded.value.construction_evidence();
    assert!(evidence.source_history_missing());
    assert_eq!(evidence.predicate_certainty(), None);
}

#[cfg(feature = "curve")]
/// Curve topology qualification, explicit retained-boundary adoption and outcome chaining witnesses.
mod curve_receipts {
    use super::*;
    use csgrs::curve;
    use hypercurve::{BooleanOp as RegionOp, CurveRegion2, ExactCurveError, FillRule};

    #[test]
    /// Admits an unresolved closing seam provisionally, then rejects strict translation as undecided.
    /// Approximate translation and dependent membership retain qualification even after dropping
    /// the original region; this exercises a policy retry, not a larger-resource-budget retry.
    fn qualified_closure_translation_preserves_receipts_and_retryable_failure() {
        use csgrs::errors::{ValidationError, ValidationErrorKind};
        use hypercurve::{CurvePath2, LineSeg2, Point2};

        let z = (Real::e() + Real::one()).sin()
            - Real::e().sin() * Real::one().cos()
            - Real::e().cos() * Real::one().sin();
        let points = [
            Point2::from_values(0, 0),
            Point2::from_values(1, 0),
            Point2::from_values(1, 1),
            Point2::from_values(0, 1),
            Point2::new(z, Real::zero()),
        ];
        let path = CurvePath2::try_new(
            points
                .windows(2)
                .map(|pair| {
                    LineSeg2::try_new(pair[0].clone(), pair[1].clone())
                        .unwrap()
                        .into()
                })
                .collect(),
        )
        .unwrap();
        let region = hypercurve::provisional(|| {
            CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero)
        })
        .into_unverified()
        .unwrap();
        let strict = curve::translated_with_context(
            &region,
            3.into(),
            4.into(),
            GeometryContext::STRICT,
        )
        .unwrap_err();
        assert!(matches!(
            strict,
            ValidationError::ExactCurve(ExactCurveError::Blocked(_))
        ));
        assert_eq!(strict.kind(), ValidationErrorKind::Undecided);
        let moved = curve::translated_with_context(
            &region,
            3.into(),
            4.into(),
            GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        drop(region);
        assert_eq!(moved.certainty, GeometryCertainty::Approximate512Consumed);
        let located = moved
            .and_then(|region| {
                curve::contains_xy_with_context(
                    &region,
                    Real::from(3) + half(),
                    Real::from(4) + half(),
                    GeometryContext::APPROXIMATE_512,
                )
            })
            .unwrap();
        assert_eq!(located.value, Some(true));
        assert_eq!(located.certainty, GeometryCertainty::Approximate512Consumed);
    }

    /// The union of the unit square with its copy translated by `1 + zero`,
    /// and an explicitly adopted model of its retained boundary. Strict proofs
    /// about the adopted model do not certify fidelity to the original union.
    /// Checks strict union/path-extraction blockers and qualification of authorized path extraction
    /// before constructing the independent retained-boundary control.
    fn approximate_region() -> (CurveRegion2, CurveRegion2) {
        let square = curve::square(Real::one()).unwrap();
        let touching =
            curve::translated(&square, Real::one() + unresolved_zero(), Real::zero()).unwrap();
        assert!(matches!(
            square.boolean_region(&touching, RegionOp::Union),
            Err(ExactCurveError::Blocked(_))
        ));
        let union =
            hypercurve::provisional(|| square.boolean_region(&touching, RegionOp::Union));
        assert!(!union.is_certified());
        assert!(union.clone().certified().is_none());
        let union = union.into_unverified().unwrap();
        assert!(matches!(
            union.boundary_paths(),
            Err(ExactCurveError::Blocked(reason))
                if reason.reason() == hypercurve::UncertaintyReason::Predicate
        ));
        let paths =
            hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
                union.boundary_paths()
            });
        assert_eq!(
            paths.certainty(),
            hypercurve::CurveCertainty::Approximate512Consumed
        );
        let paths = paths.into_unverified().unwrap();
        // Intentional adoption: this control represents the retained boundary
        // as a new model, without claiming replay of the original source.
        let rebuilt =
            CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero).unwrap();
        (union, rebuilt)
    }

    /// Requires both the modeling undecided category and the nested exact-curve predicate blocker.
    fn assert_inherited_topology_blocker(error: csgrs::errors::ValidationError) {
        assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::Undecided);
        assert!(matches!(error,
            csgrs::errors::ValidationError::ExactCurve(ExactCurveError::Blocked(reason))
                if reason.reason() == hypercurve::UncertaintyReason::Predicate
        ));
    }

    #[test]
    /// Checks membership, translation and extrusion of the qualified region across three policy passes.
    /// Strict calls must retain inherited blockers; approximate calls keep local qualification, and
    /// extrusion reports missing history. The adopted-boundary control permits certified local queries.
    fn region_receipts_are_local_and_reuse_reports_the_terminal() {
        let (union, rebuilt) = approximate_region();
        for context in [
            GeometryContext::STRICT,
            GeometryContext::APPROXIMATE_512,
            GeometryContext::STRICT,
        ] {
            let inside = curve::contains_xy_with_context(&union, Real::one(), half(), context);
            let moved =
                curve::translated_with_context(&union, Real::from(5), Real::zero(), context);
            let extruded = curve::try_extrude(&union, Real::one(), context);
            if context == GeometryContext::STRICT {
                assert_inherited_topology_blocker(inside.unwrap_err());
                assert_inherited_topology_blocker(moved.unwrap_err());
                assert_inherited_topology_blocker(extruded.unwrap_err());
            } else {
                let inside = inside.unwrap();
                assert_eq!(inside.certainty, GeometryCertainty::Approximate512Consumed);
                assert_eq!(inside.value, Some(true));
                let moved = moved.unwrap();
                assert_eq!(moved.certainty, GeometryCertainty::Approximate512Consumed);
                let moved_query = curve::contains_xy_with_context(
                    &moved.value,
                    Real::from(6),
                    half(),
                    context,
                )
                .unwrap();
                assert_eq!(
                    (moved_query.certainty, moved_query.value),
                    (GeometryCertainty::Approximate512Consumed, Some(true))
                );
                assert_inherited_topology_blocker(
                    curve::contains_xy_with_context(
                        &moved.value,
                        Real::from(6),
                        half(),
                        GeometryContext::STRICT,
                    )
                    .unwrap_err(),
                );
                let extruded = extruded.unwrap();
                assert_eq!(extruded.certainty, GeometryCertainty::Approximate512Consumed);
                assert_eq!(
                    extruded.value.construction_evidence().predicate_certainty(),
                    None
                );
                assert!(
                    extruded
                        .value
                        .construction_evidence()
                        .source_history_missing()
                );
            }
            // Certified relative to the explicitly adopted retained boundary.
            let inside =
                curve::contains_xy_with_context(&rebuilt, Real::one(), half(), context)
                    .unwrap();
            assert_eq!(
                (inside.certainty, inside.value),
                (GeometryCertainty::Certified, Some(true))
            );
            let moved =
                curve::translated_with_context(&rebuilt, Real::from(5), Real::zero(), context)
                    .unwrap();
            assert_eq!(moved.certainty, GeometryCertainty::Certified);
        }
    }

    #[test]
    /// Contrasts fallible receipt-combining chains with maps that only retain their incoming receipt.
    /// Covers strict blockers, approximate extrusion, an error nested inside a mapped value, explicitly
    /// authorized area readback with an approximate equality decision, a certified adopted control,
    /// and zero-height failure. The mapped area comparison is not an exact area proof.
    fn and_then_keeps_an_approximate_dependency_and_map_cannot_record_work() {
        let (union, rebuilt) = approximate_region();
        let moved = curve::translated_with_context(
            &union,
            Real::from(5),
            Real::zero(),
            GeometryContext::APPROXIMATE_512,
        )
        .unwrap();
        assert_eq!(moved.certainty, GeometryCertainty::Approximate512Consumed);
        assert_inherited_topology_blocker(
            curve::try_extrude(&moved.value, Real::one(), GeometryContext::STRICT)
                .unwrap_err(),
        );
        let strict_chain = moved.clone().and_then(|region| {
            curve::try_extrude(&region, Real::one(), GeometryContext::STRICT)
        });
        assert_inherited_topology_blocker(strict_chain.unwrap_err());
        let local =
            curve::try_extrude(&moved.value, Real::one(), GeometryContext::APPROXIMATE_512)
                .unwrap();
        assert_eq!(local.certainty, GeometryCertainty::Approximate512Consumed);
        let chained = moved
            .clone()
            .and_then(|region| {
                curve::try_extrude(&region, Real::one(), GeometryContext::APPROXIMATE_512)
            })
            .unwrap();
        assert_eq!(chained.certainty, GeometryCertainty::Approximate512Consumed);
        assert_eq!(chained.value, local.value);
        // A map preserves its existing receipt; it cannot turn a typed failure
        // from its closure into successful geometry or infer new decisions.
        let mapped = moved.clone().map(|region| region.filled_area());
        assert_eq!(mapped.certainty, GeometryCertainty::Approximate512Consumed);
        assert!(matches!(mapped.value, Err(ExactCurveError::Blocked(reason))
            if reason.reason() == hypercurve::UncertaintyReason::Predicate));
        let authorized = moved.clone().map(|region| {
            hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
                region.filled_area()
            })
        });
        assert_eq!(
            authorized.certainty,
            GeometryCertainty::Approximate512Consumed
        );
        assert_eq!(
            authorized.value.certainty(),
            hypercurve::CurveCertainty::Approximate512Consumed
        );
        let area = authorized
            .value
            .into_unverified()
            .unwrap()
            .expect("nonempty union area");
        // The inherited sine-addition expression is deliberately outside the
        // strict scalar normal form. This numeric readback uses the same
        // explicitly authorized terminal and cannot count as an exact proof.
        assert!(matches!(
            hyperlimit::compare_reals(
                &area,
                &Real::from(2),
                hyperlimit::PredicatePolicy::APPROXIMATE_512
            ),
            hyperlimit::PredicateOutcome::Decided {
                value: std::cmp::Ordering::Equal,
                certainty: hyperlimit::Certainty::Approximate,
                ..
            }
        ));

        let strict = curve::translated_with_context(
            &rebuilt,
            Real::from(5),
            Real::zero(),
            GeometryContext::STRICT,
        )
        .unwrap()
        .and_then(|region| curve::try_extrude(&region, Real::one(), GeometryContext::STRICT))
        .unwrap();
        assert_eq!(strict.certainty, GeometryCertainty::Certified);

        let failed = moved.and_then(|region| {
            curve::try_extrude(&region, Real::zero(), GeometryContext::STRICT)
        });
        assert!(failed.is_err());
    }
}
