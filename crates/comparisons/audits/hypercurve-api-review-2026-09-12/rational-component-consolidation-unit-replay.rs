use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn decided<T>(x:Classification<T>)->T {match x {Classification::Decided(x)=>x,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn main(){
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let selecting=Curve2::from(QuadraticBezier2::new(p(0,0),p(0,0),p(1,0)));
  let crossing=Curve2::from(LineSeg2::try_new(Point2::new(q(1,2),(-1).into()),Point2::new(q(1,2),1.into())).unwrap());
  let selected=selecting.intersect_curve(&crossing,&policy).unwrap().into_value();
  let cut=decided(selected.contacts()[0].first().parameter(&policy).unwrap());
  let retraced=Curve2::from(QuadraticBezier2::new(p(0,0),p(2,0),p(0,0)));
  let (a,b)=retraced.split_at(cut,&policy).unwrap().into_value();
  for (name,a,b) in [("complete retrace",&retraced,&retraced),("selected opposite ranges",&a,&b)] {
   let out=a.intersect_curve(b,&policy).unwrap();
   println!("{name}: complete={} contacts={} overlaps={} certainty={:?} blockers={:?}",out.value.is_complete(),out.value.contacts().len(),out.value.overlaps().len(),out.certainty,out.value.blockers());
   assert_eq!(out.certainty,CurveCertainty::Certified);
   assert!(out.value.is_complete(),"{:?}",out.value.blockers());
   assert_eq!(out.value.overlaps().len(),1);
   if name == "selected opposite ranges" {assert_eq!(out.value.contacts().len(),1);}

  }
 }
 println!("{{\"complete_queries\":4,\"retraced_selected_domain_queries\":2}}");
}
