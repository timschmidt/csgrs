//! Curve primitives built on the hyper geometry stack.
//!
//! This crate starts with the narrow line/circular-arc core needed to preserve
//! Cavalier-style polyline semantics while using `hyperlattice` scalars and the
//! `hyperlimit` predicate policy model.

mod bulge;
mod classify;
mod error;
mod intersect;
mod point;
mod policy;
mod segment;

pub use bulge::BulgeVertex2;
pub use classify::{Classification, LineSide, UncertaintyReason};
pub use error::{CurveError, CurveResult};
pub use intersect::{IntersectionKind, LineLineIntersection, ParamRange};
pub use point::Point2;
pub use policy::{CurvePolicy, NumericMode, Tolerance};
pub use segment::{CircularArc2, LineSeg2, Segment2};

pub use hyperlattice::{Backend, DefaultBackend, Scalar, ScalarSign, ZeroStatus};

#[cfg(feature = "approx")]
pub use hyperlattice::ApproxBackend;
#[cfg(feature = "hyperreal")]
pub use hyperlattice::{HyperrealBackend, Rational, Real};

#[cfg(feature = "predicates")]
pub use hyperlimit::PredicatePolicy;

#[cfg(test)]
mod tests {
    use super::*;

    fn s<B: Backend>(value: i32) -> Scalar<B> {
        value.into()
    }

    fn p<B: Backend>(x: i32, y: i32) -> Point2<B> {
        Point2::new(s(x), s(y))
    }

    fn topology_policy() -> CurvePolicy {
        #[cfg(feature = "hyperreal")]
        {
            CurvePolicy::certified()
        }
        #[cfg(not(feature = "hyperreal"))]
        {
            CurvePolicy::approximate(Tolerance::new(1e-9, 1e-9))
        }
    }

    #[test]
    fn line_segment_rejects_zero_length() {
        let err = LineSeg2::try_new(p::<DefaultBackend>(1, 2), p::<DefaultBackend>(1, 2))
            .expect_err("zero-length segment should be rejected");
        assert_eq!(err, CurveError::ZeroLengthLine);
    }

    #[test]
    fn line_segment_interpolates_midpoint() {
        let line = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 4)).unwrap();
        let midpoint = line.point_at(Scalar::try_from(0.5_f64).unwrap());
        assert_eq!(midpoint, p::<DefaultBackend>(1, 2));
    }

    #[test]
    fn bulge_zero_constructs_line_segment() {
        let segment =
            Segment2::from_bulge(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 0), s(0))
                .unwrap();
        assert!(matches!(segment, Segment2::Line(_)));
    }

    #[test]
    fn positive_semicircle_bulge_constructs_arc() {
        let segment =
            Segment2::from_bulge(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 0), s(1))
                .unwrap();
        let Segment2::Arc(arc) = segment else {
            panic!("semicircle bulge should construct an arc");
        };

        assert_eq!(arc.center(), &p::<DefaultBackend>(1, 0));
        assert_eq!(arc.radius_squared(), s(1));
        assert!(!arc.is_clockwise());
        assert_eq!(arc.bulge(), Some(&s(1)));
    }

    #[test]
    fn negative_semicircle_bulge_constructs_clockwise_arc() {
        let segment =
            Segment2::from_bulge(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 0), s(-1))
                .unwrap();
        let Segment2::Arc(arc) = segment else {
            panic!("semicircle bulge should construct an arc");
        };

        assert_eq!(arc.center(), &p::<DefaultBackend>(1, 0));
        assert!(arc.is_clockwise());
        assert_eq!(arc.bulge(), Some(&s(-1)));
    }

    #[test]
    fn cavalier_bulge_import_rejects_larger_than_half_circle() {
        let err = Segment2::from_cavalier_bulge(
            p::<DefaultBackend>(0, 0),
            p::<DefaultBackend>(1, 0),
            s(2),
        )
        .expect_err("Cavalier-compatible import should reject unsupported sweeps");
        assert_eq!(err, CurveError::UnsupportedBulge);
    }

    #[test]
    fn bulge_vertex_builds_segment_to_next_vertex() {
        let a = BulgeVertex2::new(p::<DefaultBackend>(0, 0), s(1));
        let b = BulgeVertex2::new(p::<DefaultBackend>(2, 0), s(0));
        let segment = a.segment_to(&b).unwrap();
        assert!(matches!(segment, Segment2::Arc(_)));
    }

    #[cfg(feature = "approx")]
    #[test]
    fn approx_backend_constructs_same_basic_arc() {
        let start = Point2::<ApproxBackend>::new(
            Scalar::<ApproxBackend>::try_from(0.0).unwrap(),
            Scalar::<ApproxBackend>::try_from(0.0).unwrap(),
        );
        let end = Point2::<ApproxBackend>::new(
            Scalar::<ApproxBackend>::try_from(2.0).unwrap(),
            Scalar::<ApproxBackend>::try_from(0.0).unwrap(),
        );
        let segment =
            Segment2::from_bulge(start, end, Scalar::<ApproxBackend>::try_from(1.0).unwrap())
                .unwrap();
        let Segment2::Arc(arc) = segment else {
            panic!("semicircle bulge should construct an arc");
        };
        assert_eq!(arc.radius_squared().to_f64_approx(), Some(1.0));
    }

    #[test]
    fn line_side_classifies_left_right_and_on() {
        let line = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 0)).unwrap();
        assert_eq!(
            line.classify_point(&p::<DefaultBackend>(1, 1), &topology_policy()),
            Classification::Decided(LineSide::Left)
        );
        assert_eq!(
            line.classify_point(&p::<DefaultBackend>(1, -1), &topology_policy()),
            Classification::Decided(LineSide::Right)
        );
        assert_eq!(
            line.classify_point(&p::<DefaultBackend>(1, 0), &topology_policy()),
            Classification::Decided(LineSide::On)
        );
    }

    #[test]
    fn line_line_intersection_crosses_at_point() {
        let a = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(2, 2)).unwrap();
        let b = LineSeg2::try_new(p::<DefaultBackend>(0, 2), p::<DefaultBackend>(2, 0)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        } = intersection
        else {
            panic!("expected one point intersection");
        };

        let half = Scalar::<DefaultBackend>::try_from(0.5_f64).unwrap();
        assert_eq!(point, p::<DefaultBackend>(1, 1));
        assert_eq!(a_param, half);
        assert_eq!(
            b_param,
            Scalar::<DefaultBackend>::try_from(0.5_f64).unwrap()
        );
        assert_eq!(kind, IntersectionKind::Crossing);
    }

    #[test]
    fn line_line_intersection_detects_endpoint_touch() {
        let a = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(1, 0)).unwrap();
        let b = LineSeg2::try_new(p::<DefaultBackend>(1, 0), p::<DefaultBackend>(1, 1)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Point { point, kind, .. } = intersection else {
            panic!("expected endpoint point intersection");
        };

        assert_eq!(point, p::<DefaultBackend>(1, 0));
        assert_eq!(kind, IntersectionKind::Endpoint);
    }

    #[test]
    fn line_line_intersection_detects_collinear_overlap() {
        let a = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(4, 0)).unwrap();
        let b = LineSeg2::try_new(p::<DefaultBackend>(2, 0), p::<DefaultBackend>(6, 0)).unwrap();
        let intersection = a.intersect_line(&b, &topology_policy()).unwrap();

        let LineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } = intersection
        else {
            panic!("expected overlap");
        };

        assert_eq!(segment.start(), &p::<DefaultBackend>(2, 0));
        assert_eq!(segment.end(), &p::<DefaultBackend>(4, 0));
        assert_eq!(
            a_range.start(),
            &Scalar::<DefaultBackend>::try_from(0.5_f64).unwrap()
        );
        assert_eq!(a_range.end(), &s::<DefaultBackend>(1));
        assert_eq!(b_range.start(), &s::<DefaultBackend>(0));
        assert_eq!(
            b_range.end(),
            &Scalar::<DefaultBackend>::try_from(0.5_f64).unwrap()
        );
    }

    #[test]
    fn line_line_intersection_detects_parallel_disjoint() {
        let a = LineSeg2::try_new(p::<DefaultBackend>(0, 0), p::<DefaultBackend>(1, 0)).unwrap();
        let b = LineSeg2::try_new(p::<DefaultBackend>(0, 1), p::<DefaultBackend>(1, 1)).unwrap();
        assert_eq!(
            a.intersect_line(&b, &topology_policy()).unwrap(),
            LineLineIntersection::None
        );
    }
}
