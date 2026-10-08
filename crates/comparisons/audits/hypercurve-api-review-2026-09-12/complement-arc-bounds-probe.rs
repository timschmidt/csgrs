#![allow(unused_imports,dead_code)]
use hypercurve::*;
fn p(x:i64,y:i64)->Point2{Point2::new(Real::from(x),Real::from(y))}
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
trait IntoCertifiedClassification<T> {
    fn into_certified_classification(self) -> Classification<T>;
}

impl<T> IntoCertifiedClassification<T> for Classification<T> {
    fn into_certified_classification(self) -> Classification<T> {
        self
    }
}

impl<T> IntoCertifiedClassification<T> for CurveOutcome<Classification<T>> {
    fn into_certified_classification(self) -> Classification<T> {
        assert_eq!(self.certainty, CurveCertainty::Certified);
        self.value
    }
}

fn decided<T>(classification: impl IntoCertifiedClassification<T>) -> T {
    match classification.into_certified_classification() {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => panic!("expected decided result, got {reason:?}"),
    }
}

fn certified<T>(outcome: CurveOutcome<T>) -> T {
    assert_eq!(outcome.certainty, CurveCertainty::Certified);
    outcome.value
}
fn source_path() -> CurvePath2 {
        let corner = p(1, 1);
        let center = Point2::new(Real::one(), q(3923, 2150));
        let arc_end = Point2::new(q(3923, 2150), q(3923, 2150));
        CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(
                p(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                corner.clone(),
            )),
            Curve2::from(
                CircularArc2::try_from_center(corner, arc_end.clone(), center, false).unwrap(),
            ),
            Curve2::from(LineSeg2::try_new(arc_end, p(0, 3)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(0, 3), p(0, 0)).unwrap()),
        ])
        .unwrap()
    }

    fn corner_index(region: &CurveRegion2) -> usize {
        let fragments = region.boundary_loops()[0].fragments();
        (0..fragments.len())
            .find(|index| {
                let previous = &fragments[(index + fragments.len() - 1) % fragments.len()];
                let next = &fragments[*index];
                matches!(
                    (previous, next),
                    (
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::Quadratic(previous),
                            ..
                        },
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::RationalQuadratic(next),
                            ..
                        }
                    ) if previous.end() == &p(1, 1) && next.start() == &p(1, 1)
                ) || matches!(
                    (previous, next),
                    (
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::RationalQuadratic(previous),
                            ..
                        },
                        BezierSplitFragment2::Materialized {
                            curve: BezierSubcurve2::Quadratic(next),
                            ..
                        }
                    ) if previous.end() == &p(1, 1) && next.start() == &p(1, 1)
                )
            })
            .expect("the arc/parabola corner remains explicit")
    }

    fn candidates(solutions: CurveCornerSolutions2<CurveRegion2>) -> Vec<CurveRegion2> {
        match solutions {
            CurveCornerSolutions2::Unique(candidate) => vec![candidate],
            CurveCornerSolutions2::Multiple(candidates) => candidates,
            CurveCornerSolutions2::NoSolution(reason) => {
                panic!("the incident arc/parabola cells must contain a fillet: {reason:?}")
            }
        }
    }


fn point_box(point:&CurvePoint2,policy:&CurveContext)->[Option<f64>;4]{let b=decided(point.bounds(policy));[b.min_x().to_f64_lossy(),b.max_x().to_f64_lossy(),b.min_y().to_f64_lossy(),b.max_y().to_f64_lossy()]}
fn main() {
 let policy=CurveContext::STRICT;
 let region=CurveRegion2::try_from_boundary_paths(&[source_path()],&policy).unwrap().into_value();
 let edited=region.fillet_loop_vertex_by_radius(0,corner_index(&region),q(1,2),CurveCornerMode2::TrimOrExtend,&policy).unwrap().into_value();
 let candidates=candidates(edited);let candidate=&candidates[1];
 for path in decided(candidate.boundary_paths(&policy).unwrap()) {
   for (i,curve) in path.curves().iter().enumerate() {println!("curve {i} family={:?} native={} start={:?} end={:?}",curve.family(),curve.geometry().is_some(),point_box(&curve.start(),&policy),point_box(&curve.end(),&policy));}
 }
 for path in decided(candidate.boundary_paths(&policy).unwrap()) {
 for (i,c) in path.curves().iter().enumerate() {println!("bounds {i}: {:?}", c.bounds().map(|b| [b.min_x().to_f64_lossy(),b.max_x().to_f64_lossy(),b.min_y().to_f64_lossy(),b.max_y().to_f64_lossy()]));}
 }
}
