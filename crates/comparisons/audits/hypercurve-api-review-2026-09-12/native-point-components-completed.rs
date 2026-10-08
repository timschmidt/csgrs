use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x,y) }
fn main() {
 let mut complete=0;
 let mut components=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let point=Curve2::from(QuadraticBezier2::new(p(0,0),p(0,0),p(0,0)));
  let crossing=Curve2::from(LineSeg2::try_new(p(-1,0),p(1,0)).unwrap());
  let ending=Curve2::from(LineSeg2::try_new(p(0,0),p(1,0)).unwrap());
  for (name,other) in [("interior",&crossing),("endpoint",&ending),("point",&point)] {
   for swapped in [false,true] {
    let (a,b)=if swapped {(other,&point)}else{(&point,other)};
    let outcome=a.intersect_curve(b,&policy).unwrap();
    assert_eq!(outcome.certainty,CurveCertainty::Certified);
    let r=outcome.value;
    println!("{policy:?} {name} swapped={swapped} complete={} contacts={} overlaps={} components={} blockers={}",r.is_complete(),r.contacts().len(),r.overlaps().len(),r.parameter_components().len(),r.blockers().len());
    assert!(r.is_complete());complete+=1;
    assert!(r.contacts().is_empty() && r.overlaps().is_empty());
    let [component]=r.parameter_components() else {panic!("one full parameter component")};
    components+=1;
    let (free,fixed)=if swapped {(component.second_parameters(),component.first_parameters())}else{(component.first_parameters(),component.second_parameters())};
    assert!(matches!(free,CurveParameterSet2::Range(_)));
    if name=="point" {assert!(matches!(fixed,CurveParameterSet2::Range(_)));}
    else {let CurveParameterSet2::Single(parameter)=fixed else {panic!("one line parameter")};assert_eq!(parameter.scalar(),Some(&if name=="interior"{(Real::one()/Real::from(2)).unwrap()}else{Real::zero()}));}
    let replay=component.point().coincides_with(&p(0,0).into(),&policy);
    assert_eq!(replay.certainty,CurveCertainty::Certified);assert_eq!(replay.value,Classification::Decided(true));
   }
  }
 }
 println!("{{\"complete_queries\":{complete},\"parameter_components\":{components}}}");
}
