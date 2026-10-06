use hyperlimit::error::PredicateError;
use hyperlimit::{
    Aabb2Intersection, Aabb2PointLocation, Aabb3Intersection, Aabb3PointLocation,
    AabbSphereIntersection, Certainty, ClosedIntervalIntersection, ConvexPointLocation,
    CoplanarTriangleRelation, LineSide, PlaneSide, PointSegmentLocation, RayTriangleIntersection,
    RealIntervalLocation, RingEvenOddEdgeReport, RingPointLocation, SegmentIntersection,
    SegmentTriangleIntersection, Sign, SignKnowledge, SphereIntersection, SupportDopPlaneRelation,
    SupportDopRelation, TriangleTriangleIntersection,
};

#[test]
fn sign_side_conversions_cover_every_sign() {
    assert_eq!(LineSide::from(Sign::Negative), LineSide::Right);
    assert_eq!(LineSide::from(Sign::Zero), LineSide::On);
    assert_eq!(LineSide::from(Sign::Positive), LineSide::Left);

    assert_eq!(PlaneSide::from(Sign::Negative), PlaneSide::Below);
    assert_eq!(PlaneSide::from(Sign::Zero), PlaneSide::On);
    assert_eq!(PlaneSide::from(Sign::Positive), PlaneSide::Above);

    assert_eq!(Sign::Negative.reversed(), Sign::Positive);
    assert_eq!(Sign::Zero.reversed(), Sign::Zero);
    assert_eq!(Sign::Positive.reversed(), Sign::Negative);
}

#[test]
fn sign_knowledge_exposes_only_concrete_known_signs() {
    assert_eq!(
        SignKnowledge::exact(Sign::Negative).sign(),
        Some(Sign::Negative)
    );
    assert_eq!(
        SignKnowledge::filtered(Sign::Positive).sign(),
        Some(Sign::Positive)
    );
    assert_eq!(SignKnowledge::NonZero.sign(), None);
    assert_eq!(SignKnowledge::Unknown.sign(), None);
    assert_eq!(
        SignKnowledge::filtered(Sign::Zero),
        SignKnowledge::Known {
            sign: Sign::Zero,
            certainty: Certainty::Filtered,
        }
    );
}

#[test]
fn inclusive_classification_helpers_cover_every_variant() {
    for (value, expected) in [
        (ConvexPointLocation::Degenerate, false),
        (ConvexPointLocation::Outside, false),
        (ConvexPointLocation::Boundary, true),
        (ConvexPointLocation::Inside, true),
    ] {
        assert_eq!(value.is_inside_or_boundary(), expected);
    }

    for (value, expected) in [
        (SupportDopRelation::Degenerate, false),
        (SupportDopRelation::Separated, false),
        (SupportDopRelation::BoundaryTouch, true),
        (SupportDopRelation::ConservativeOverlap, true),
    ] {
        assert_eq!(value.may_intersect(), expected);
    }

    for (value, expected) in [
        (SupportDopPlaneRelation::Degenerate, false),
        (SupportDopPlaneRelation::Below, false),
        (SupportDopPlaneRelation::Above, false),
        (SupportDopPlaneRelation::Intersecting, true),
    ] {
        assert_eq!(value.intersects_plane(), expected);
    }

    for (value, expected) in [
        (SphereIntersection::Disjoint, false),
        (SphereIntersection::Touching, true),
        (SphereIntersection::Overlapping, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (AabbSphereIntersection::Disjoint, false),
        (AabbSphereIntersection::Touching, true),
        (AabbSphereIntersection::Overlapping, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (PointSegmentLocation::OffLine, false),
        (PointSegmentLocation::CollinearOutside, false),
        (PointSegmentLocation::OnEndpoint, true),
        (PointSegmentLocation::OnSegment, true),
    ] {
        assert_eq!(value.is_on_segment(), expected);
    }

    for (value, expected) in [
        (SegmentTriangleIntersection::Disjoint, false),
        (SegmentTriangleIntersection::Proper, true),
        (SegmentTriangleIntersection::BoundaryTouch, true),
        (SegmentTriangleIntersection::Coplanar, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (RayTriangleIntersection::Disjoint, false),
        (RayTriangleIntersection::Proper, true),
        (RayTriangleIntersection::BoundaryTouch, true),
        (RayTriangleIntersection::Coplanar, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (RingPointLocation::Outside, false),
        (RingPointLocation::Boundary, true),
        (RingPointLocation::Inside, true),
    ] {
        assert_eq!(value.is_inside_or_boundary(), expected);
    }

    for (value, expected) in [
        (RealIntervalLocation::Below, false),
        (RealIntervalLocation::AtLowerEndpoint, true),
        (RealIntervalLocation::Interior, true),
        (RealIntervalLocation::AtUpperEndpoint, true),
        (RealIntervalLocation::Above, false),
    ] {
        assert_eq!(value.is_inside_or_boundary(), expected);
    }

    for (value, expected) in [
        (ClosedIntervalIntersection::Disjoint, false),
        (ClosedIntervalIntersection::Touching, true),
        (ClosedIntervalIntersection::Overlapping, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (Aabb2PointLocation::Outside, false),
        (Aabb2PointLocation::Boundary, true),
        (Aabb2PointLocation::Inside, true),
    ] {
        assert_eq!(value.is_inside_or_boundary(), expected);
    }

    for (value, expected) in [
        (Aabb3PointLocation::Outside, false),
        (Aabb3PointLocation::Boundary, true),
        (Aabb3PointLocation::Inside, true),
    ] {
        assert_eq!(value.is_inside_or_boundary(), expected);
    }

    for (value, expected) in [
        (Aabb2Intersection::Disjoint, false),
        (Aabb2Intersection::Touching, true),
        (Aabb2Intersection::Overlapping, true),
    ] {
        assert_eq!(value.intersects(), expected);
    }

    for (value, expected) in [
        (Aabb3Intersection::Disjoint, false),
        (Aabb3Intersection::Touching, true),
        (Aabb3Intersection::Overlapping, true),
    ] {
        assert_eq!(value.intersects(), expected);
        assert_eq!(value.needs_narrow_phase(), expected);
    }
}

#[test]
fn segment_classification_helpers_cover_every_variant() {
    for value in [
        SegmentIntersection::Disjoint,
        SegmentIntersection::Proper,
        SegmentIntersection::EndpointTouch,
        SegmentIntersection::CollinearOverlap,
        SegmentIntersection::Identical,
    ] {
        assert_eq!(value.is_disjoint(), value == SegmentIntersection::Disjoint);
        assert_eq!(value.intersects(), value != SegmentIntersection::Disjoint);
        assert_eq!(
            value.is_proper_crossing(),
            value == SegmentIntersection::Proper
        );
        assert_eq!(
            value.is_endpoint_touch(),
            value == SegmentIntersection::EndpointTouch
        );
        assert_eq!(
            value.has_positive_length_overlap(),
            matches!(
                value,
                SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical
            )
        );
    }
}

#[test]
fn triangle_triangle_helpers_cover_every_variant() {
    for (value, intersects, needs_construction) in [
        (TriangleTriangleIntersection::Degenerate, false, false),
        (TriangleTriangleIntersection::Disjoint, false, false),
        (
            TriangleTriangleIntersection::NonCoplanarIntersection,
            true,
            true,
        ),
        (TriangleTriangleIntersection::BoundaryTouch, true, true),
        (TriangleTriangleIntersection::CoplanarDisjoint, false, false),
        (TriangleTriangleIntersection::CoplanarTouching, true, true),
        (
            TriangleTriangleIntersection::CoplanarOverlapping,
            true,
            true,
        ),
    ] {
        assert_eq!(value.intersects(), intersects);
        assert_eq!(value.needs_construction(), needs_construction);
    }
}

#[test]
fn coplanar_and_ring_evidence_helpers_cover_all_states() {
    for (relation, expected) in [
        (CoplanarTriangleRelation::Disjoint, false),
        (CoplanarTriangleRelation::Touching, true),
        (CoplanarTriangleRelation::Overlapping, true),
        (CoplanarTriangleRelation::Unknown, true),
    ] {
        assert_eq!(relation.needs_graph_construction(), expected);
    }

    let boundary = RingEvenOddEdgeReport {
        edge_index: 0,
        segment_location: PointSegmentLocation::OnSegment,
        a_above: None,
        b_above: None,
        upward: None,
        orientation: None,
        crosses_right: false,
    };
    assert!(boundary.is_boundary());
    assert!(!boundary.is_y_straddling());

    let straddling = RingEvenOddEdgeReport {
        edge_index: 1,
        segment_location: PointSegmentLocation::OffLine,
        a_above: Some(false),
        b_above: Some(true),
        upward: Some(true),
        orientation: Some(Sign::Positive),
        crosses_right: true,
    };
    assert!(!straddling.is_boundary());
    assert!(straddling.is_y_straddling());

    let non_straddling = RingEvenOddEdgeReport {
        a_above: Some(true),
        b_above: Some(true),
        upward: None,
        orientation: None,
        crosses_right: false,
        ..straddling
    };
    assert!(!non_straddling.is_y_straddling());
}

#[test]
fn predicate_errors_have_stable_human_readable_messages() {
    let capability = PredicateError::CapabilityUnavailable("exact sign");
    let real = PredicateError::Real("refinement aborted");

    assert_eq!(
        capability.to_string(),
        "predicate capability unavailable: exact sign"
    );
    assert_eq!(real.to_string(), "predicate Real error: refinement aborted");
    #[cfg(feature = "std")]
    {
        assert!(std::error::Error::source(&capability).is_none());
        assert!(std::error::Error::source(&real).is_none());
    }
}
