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

fn main() {
 let policy=CurveContext::STRICT;
 let path=source_path();
 let region=CurveRegion2::try_from_boundary_paths(&[path],&policy).unwrap().into_value();
 let edited=region.fillet_loop_vertex_by_radius(0,corner_index(&region),q(1,2),CurveCornerMode2::TrimOrExtend,&policy).unwrap();
 println!("edit certainty={:?}",edited.certainty);
 let candidates=candidates(edited.value);
 let parabola=Curve2::from(QuadraticBezier2::new(p(0,0),p(1,0),p(2,4)));
 for (ci,candidate) in candidates.iter().enumerate() {
   println!("candidate={ci} loops={} sample={:?}",candidate.boundary_loops().len(),candidate.classify_point(&Point2::new(q(1,4),q(3,2)),&policy));
   for path in decided(candidate.boundary_paths(&policy).unwrap()) {
     for curve in path.curves() {
       let start=curve.start();let end=curve.end();
       let zero:CurvePoint2=p(0,0).into();let corner:CurvePoint2=p(1,1).into();
       let endpoint=if decided(start.coincides_with(&zero,&policy)) {Some(end)} else if decided(end.coincides_with(&zero,&policy)) {Some(start)} else {None};
       if let Some(endpoint)=endpoint {
          println!("  origin incident family={:?} native={} x_vs_1={:?}",curve.family(),curve.geometry().is_some(),endpoint.compare_coordinate(&corner,Axis2::X,&policy));
          match curve.intersect_curve(&parabola,&policy) {
             Ok(result)=>println!("  retained/independent parabola: certainty={:?} complete={} contacts={} overlaps={} blockers={:?}",result.certainty,result.value.is_complete(),result.value.contacts().len(),result.value.overlaps().len(),result.value.blockers()),
             Err(error)=>println!("  retained/independent parabola: {error:?}")
          }
       }
     }
   }
 }
}
