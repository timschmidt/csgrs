use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn main(){
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let retraced=Curve2::from(QuadraticBezier2::new(p(0,0),p(1,0),p(0,0)));
  let line=Curve2::from(LineSeg2::try_new(p(-1,0),p(0,0)).unwrap());
  for swapped in [false,true]{
   let (a,b)=if swapped {(&line,&retraced)} else {(&retraced,&line)};
   let result=a.intersect_curve(b,&policy).unwrap();
   assert_eq!(result.certainty,CurveCertainty::Certified);
   let result=result.value;
   println!("{policy:?} swapped={swapped} complete={} contacts={} blockers={}",result.is_complete(),result.contacts().len(),result.blockers().len());
   for contact in result.contacts(){
    let parameter=if swapped {contact.second()} else {contact.first()};
    println!("retained visit={:?}",parameter.local_parameter().scalar());
   }
   // The same geometric point has both endpoint parameters on this trace.
   assert_eq!(retraced.start().coincides_with(&retraced.end(),&policy).value,Classification::Decided(true));
  }
 }
}
