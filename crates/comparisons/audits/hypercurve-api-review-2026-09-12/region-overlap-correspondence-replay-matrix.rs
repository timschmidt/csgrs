use hypercurve::*;
use std::cmp::Ordering;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn certified<T>(x:CurveOutcome<T>)->T {assert_eq!(x.certainty,CurveCertainty::Certified);x.value}
fn decided<T>(x:Classification<T>)->T {match x {Classification::Decided(x)=>x,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn bounds(r:&CurveParameterRange2)->[CurveParameter2;2] {[r.start().clone(),r.end().clone()]}
fn same(a:CurvePoint2,b:CurvePoint2,policy:&CurveContext){assert!(decided(certified(a.coincides_with(&b,policy))));}
#[derive(Default)] struct Counts {queries:usize, restrictions:usize, replays:usize, repeated:usize, regions:usize}
fn replay(a:&Curve2,b:&Curve2,overlap:&CurveIntersectionOverlap2,policy:&CurveContext,c:&mut Counts){
 for (x,y) in [(overlap.first_range().start(),overlap.second_range().start()),(overlap.first_range().end(),overlap.second_range().end())] {
  same(certified(a.point_at(x,policy).unwrap()),certified(b.point_at(y,policy).unwrap()),policy);c.replays+=2;
 }
}
fn exercise(a:&Curve2,b:&Curve2,overlap:&CurveIntersectionOverlap2,policy:&CurveContext,c:&mut Counts){
 replay(a,b,overlap,policy,c);
 let mut cut=None;
 'cuts:for axis in 0..2 {for n in -15..=15 {
  let v=q(n,4);
  let line=Curve2::from(LineSeg2::try_new(if axis==0 {Point2::new(v.clone(),(-10).into())}else{Point2::new((-10).into(),v.clone())},if axis==0 {Point2::new(v,10.into())}else{Point2::new(10.into(),v)}).unwrap());
  let result=certified(a.intersect_curve(&line,policy).unwrap());c.queries+=1;assert!(result.is_complete(),"{:?}",result.blockers());
  for contact in result.contacts(){let t=contact.first().local_parameter();let lo=decided(certified(t.compare(overlap.first_range().start(),policy).unwrap()));let hi=decided(certified(t.compare(overlap.first_range().end(),policy).unwrap()));
   if (lo==Ordering::Less&&hi==Ordering::Greater)||(lo==Ordering::Greater&&hi==Ordering::Less){cut=Some(t.clone());break 'cuts;}
  }
 }}
 let cut=cut.expect("fixture has a represented grid crossing inside every overlap");
 let clipped=decided(certified(overlap.restrict([overlap.first_range().start().clone(),cut.clone()],bounds(overlap.second_range()),policy).unwrap())).unwrap();c.restrictions+=1;
 replay(a,b,&clipped,policy,c);
 for _ in 0..8 {let again=decided(certified(clipped.restrict(bounds(overlap.first_range()),bounds(overlap.second_range()),policy).unwrap())).unwrap();assert_eq!(again,clipped);c.repeated+=1;}
 assert!(decided(certified(clipped.restrict([cut.clone(),overlap.first_range().end().clone()],bounds(overlap.second_range()),policy).unwrap())).is_none());c.restrictions+=1;
 assert!(decided(certified(overlap.restrict([cut.clone(),cut],bounds(overlap.second_range()),policy).unwrap())).is_none());c.restrictions+=1;
}
fn rectangle(x:i32,y:i32)->CurveRegion2{
 let points=[p(x,0),p(x+4,0),p(x+4,y),p(x,y)];let curves=(0..4).map(|i|Curve2::from(LineSeg2::try_new(points[i].clone(),points[(i+1)%4].clone()).unwrap())).collect();
 certified(CurveRegion2::try_from_boundary_paths(&[CurvePath2::try_new(curves).unwrap()],&CurveContext::STRICT).unwrap())
}
fn main(){let mut c=Counts::default();
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let nonlinear=Curve2::from(QuadraticBezier2::new(p(0,0),p(0,0),p(4,0)));
  let line=Curve2::from(LineSeg2::try_new(p(1,0),p(3,0)).unwrap());
  let arc=Curve2::from(CircularArc2::try_from_center(p(4,0),p(0,4),p(0,0),false).unwrap());
  let other_arc=Curve2::from(CircularArc2::try_from_center(Point2::new(q(12,5),q(16,5)),p(-4,0),p(0,0),false).unwrap());
  for (a,b) in [(&nonlinear,&line),(&arc,&other_arc)] {
   for reverse in [false,true]{let b=if reverse {certified(b.reversed(&policy).unwrap())}else{b.clone()};
    for (a,b) in [(a,&b),(&b,a)] {let result=certified(a.intersect_curve(b,&policy).unwrap());c.queries+=1;assert!(result.is_complete());assert!(!result.overlaps().is_empty());
     let a_spans=certified(a.native_bezier_fragments(&policy).unwrap());let b_spans=certified(b.native_bezier_fragments(&policy).unwrap());
     for overlap in result.overlaps(){let a=Curve2::from(a_spans[overlap.first_span_index()].curve().clone());let b=Curve2::from(b_spans[overlap.second_span_index()].curve().clone());exercise(&a,&b,overlap,&policy,&mut c);}
    }
   }
  }
  for (a,b) in [(rectangle(0,4),rectangle(2,3)),(rectangle(2,3),rectangle(0,4))] {
   let report=certified(a.intersect_region(&b,&policy).unwrap());assert!(report.is_complete());assert!(!report.overlaps().is_empty());c.regions+=1;
   drop(a);drop(b);
   for overlap in report.overlaps(){exercise(overlap.first().curve(),overlap.second().curve(),overlap.overlap(),&policy,&mut c);}
  }
 }
 println!("{{\"queries\":{},\"restrictions\":{},\"point_replays\":{},\"repeated_restrictions\":{},\"region_reports_after_inputs_dropped\":{}}}",c.queries,c.restrictions,c.replays,c.repeated,c.regions);
}
