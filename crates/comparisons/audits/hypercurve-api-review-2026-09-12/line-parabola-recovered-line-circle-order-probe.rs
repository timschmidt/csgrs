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
fn source_path(line_end: Point2) -> CurvePath2 {
        let corner = p(1, 1);
        CurvePath2::try_new(vec![
            Curve2::from(QuadraticBezier2::new(
                p(0, 0),
                Point2::new(q(1, 2), Real::zero()),
                corner.clone(),
            )),
            Curve2::from(LineSeg2::try_new(corner, line_end.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(line_end, p(-2, 3)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-2, 3), p(-2, -2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(-2, -2), p(0, -2)).unwrap()),
            Curve2::from(LineSeg2::try_new(p(0, -2), p(0, 0)).unwrap()),
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
                            curve: previous, ..
                        },
                        BezierSplitFragment2::Materialized { curve: next, .. }
                    ) if previous.end() == &p(1, 1) && next.start() == &p(1, 1)
                )
            })
            .expect("the line/parabola corner remains explicit")
    }

    fn candidates(solutions: CurveCornerSolutions2<CurveRegion2>) -> Vec<CurveRegion2> {
        match solutions {
            CurveCornerSolutions2::Unique(candidate) => vec![candidate],
            CurveCornerSolutions2::Multiple(candidates) => candidates,
            CurveCornerSolutions2::NoSolution(reason) => {
                panic!("the incident cell must contain a fillet: {reason:?}")
            }
        }
    }


fn main() {
 let policy=CurveContext::STRICT;
 let end=Point2::new(Real::one()+q(38280,91901), Real::one()+q(83549,91901));
 let region=certified(CurveRegion2::try_from_boundary_paths(&[source_path(end)],&policy).unwrap());
 let solutions=candidates(certified(region.fillet_loop_vertex_by_radius(0,corner_index(&region),q(299,125),CurveCornerMode2::TrimOrExtend,&policy).unwrap()));
 let cut:CurvePoint2=Point2::new(q(6,5),q(36,25)).into();
 let candidate=solutions.iter().find(|candidate|decided(candidate.boundary_paths(&policy).unwrap()).iter().flat_map(CurvePath2::curves).any(|c|decided(c.start().coincides_with(&cut,&policy))||decided(c.end().coincides_with(&cut,&policy)))).unwrap();
 let normalized=certified(candidate.regularized_region(&policy).unwrap());
 let ps=[p(8,8),p(9,8),p(9,9),p(8,9)];
 let square=Contour2::try_new((0..4).map(|i|Segment2::Line(LineSeg2::try_new(ps[i].clone(),ps[(i+1)%4].clone()).unwrap())).collect()).unwrap();
 let disjoint=certified(CurveRegion2::try_from_native_material_contours(vec![square],&policy).unwrap());
 let union=certified(candidate.boolean_region(&disjoint,BooleanOp::Union,&policy).unwrap());
 let recovered=certified(union.boolean_region(&disjoint,BooleanOp::Difference,&policy).unwrap());
 println!("normalized loops={} recovered loops={}",normalized.boundary_loops().len(),recovered.boundary_loops().len());
 println!("same set xor: {:?}",recovered.boolean_region(&normalized,BooleanOp::Xor,&policy).map(|v|(v.certainty,v.value.is_empty())));
 for (name,first,second) in [("self normalized",&normalized,&normalized),("self recovered",&recovered,&recovered),("mixed",&recovered,&normalized)] {
  println!("region {name}: {:?}",first.intersect_region(second,&policy).map(|v|(v.certainty,v.value.is_complete(),v.value.contacts().len(),v.value.overlaps().len())));
 }
 let first=decided(recovered.boundary_paths(&policy).unwrap()).into_iter().flat_map(|p|p.curves().to_vec()).collect::<Vec<_>>();
 let second=decided(normalized.boundary_paths(&policy).unwrap()).into_iter().flat_map(|p|p.curves().to_vec()).collect::<Vec<_>>();
 for (i,c) in first.iter().enumerate() {if [8,9,10,14].contains(&i) {println!("source {i} native={} bounds={:?}",c.geometry().is_some(),c.bounds().map(|b|[b.min_x().to_f64_lossy(),b.max_x().to_f64_lossy(),b.min_y().to_f64_lossy(),b.max_y().to_f64_lossy()]));}}
 let center=Point2::new(q(-126,125),q(59,25));let radius=q(299,125);
 let arc=CircularArc2::try_from_center(Point2::new(center.x()+&radius,center.y().clone()),Point2::new(center.x().clone(),center.y()+&radius),center,false).unwrap();
 for i in [8,14] {
  let Some(CurveGeometry2::QuadraticBezier(q))=first[i].geometry() else{panic!()};let line=LineSeg2::try_new(q.start().clone(),q.end().clone()).unwrap();
  match line.supporting_line_circle_relation(&arc,&policy).unwrap(){
    LineCircleRelation::Disjoint=>println!("primitive {i}: disjoint"),
    LineCircleRelation::Uncertain {reason}=>println!("primitive {i}: uncertain {reason:?}"),
    LineCircleRelation::Tangent {line_param,..}=>println!("primitive {i}: tangent {:?}",line_param.to_f64_lossy()),
    LineCircleRelation::Secant {first_param,second_param,..}=>{
     println!("primitive {i}: secant {:?} {:?}",first_param.to_f64_lossy(),second_param.to_f64_lossy());
     for t in [&first_param,&second_param] {let t=BezierParameter2::Exact(t.clone()); println!("  root vs 0: {:?}; vs 1: {:?}",t.cmp_by_refinement(&BezierParameter2::Exact(Real::zero()),&policy),t.cmp_by_refinement(&BezierParameter2::Exact(Real::one()),&policy));}
    },
  }
 }
}
