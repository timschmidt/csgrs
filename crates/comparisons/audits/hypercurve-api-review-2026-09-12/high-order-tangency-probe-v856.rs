use hypercurve::{BooleanOp,Classification,Curve2,CurveCertainty,CurveContext,CurveOutcome,CurvePath2,CurvePoint2,CurveRegion2,ExactCurveError,FillRule,HomogeneousControl2,LineSeg2,Point2,RationalBezier2,Real,RegionPointLocation};
fn decided<T>(v:Classification<T>)->T {match v{Classification::Decided(x)=>x,Classification::Uncertain(r)=>panic!("unexpected construction blocker {r:?}")}}
fn q(n:u64,d:u64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn result(label:&str,v:Result<CurveOutcome<CurveRegion2>,ExactCurveError>)->Option<CurveRegion2>{
 match v{
  Ok(v)=>{println!("stage={label} certified={} loops={}",v.certainty==CurveCertainty::Certified,v.value.boundary_loops().len());assert_eq!(v.certainty,CurveCertainty::Certified);Some(v.value)},
  Err(ExactCurveError::Blocked(b))=>{println!("stage={label} blocked=true operation={:?} family={:?} reason={:?}",b.operation(),b.family(),b.reason());None},
  Err(ExactCurveError::Invalid{operation,family,..})=>{println!("stage={label} invalid=true operation={operation:?} family={family:?}");None}
 }
}
fn cap(degree:u64,scale:u64,policy:&CurveContext)->Option<CurveRegion2>{
 let controls=(0..=degree).map(|i|HomogeneousControl2::new(q(i,degree),Real::from(if i==degree{scale}else{0}),Real::one())).collect();
 let curve=decided(RationalBezier2::from_homogeneous_controls(controls,policy).unwrap());
 let point=|x,y|Point2::from_values(x,y);
 let path=CurvePath2::try_new_with_policy(vec![Curve2::from(curve),Curve2::from(LineSeg2::try_new(point(1,scale),point(1,0)).unwrap()),Curve2::from(LineSeg2::try_new(point(1,0),point(0,0)).unwrap())],policy).unwrap().into_value();
 result(if scale==1{"admit_inner"}else{"admit_outer"},CurveRegion2::try_from_boundary_paths(&[path],FillRule::NonZero,policy))
}
fn check(region:&CurveRegion2,degree:u64,expected:[bool;3],policy:&CurveContext){
 let unit=1_u64<<degree;
 for ((num,den),inside)in [(1,4*unit),(3,2*unit),(3,unit)].into_iter().zip(expected){
  let point=CurvePoint2::from(Point2::new(q(1,2),q(num,den)));
  let value=region.classify_point(&point,policy).unwrap();assert_eq!(value.certainty,CurveCertainty::Certified);
  assert_eq!(value.value,Classification::Decided(if inside{RegionPointLocation::Inside}else{RegionPointLocation::Outside}));
 }
}
fn main(){
 let args:Vec<_>=std::env::args().collect();let degree:u64=args[1].parse().unwrap();let policy_index:u32=args[2].parse().unwrap();let policy=if policy_index==0{CurveContext::STRICT}else{CurveContext::APPROXIMATE_512};
 println!("start degree={degree} policy={policy_index}");
 let Some(inner)=cap(degree,1,&policy)else{return};let Some(outer)=cap(degree,2,&policy)else{return};
 check(&inner,degree,[true,false,false],&policy);check(&outer,degree,[true,true,false],&policy);
 let mut successes=0;let mut difference=None;
 for(operation,expected)in [(BooleanOp::Union,[true,true,false]),(BooleanOp::Intersection,[true,false,false]),(BooleanOp::Difference,[false,true,false]),(BooleanOp::Xor,[false,true,false])]{
  println!("request={operation:?}");
  if let Some(region)=result(&format!("{operation:?}"),outer.boolean_region(&inner,operation,&policy)){
   check(&region,degree,expected,&policy);successes+=1;
   if operation==BooleanOp::Difference{difference=Some(region)}
  }
 }
 if let Some(difference)=difference{
  println!("request=recompose");
  if let Some(region)=result("recompose",difference.boolean_region(&inner,BooleanOp::Union,&policy)){check(&region,degree,[true,true,false],&policy);successes+=1;}
 }
 println!("complete degree={degree} policy={policy_index} successful_operations={successes}");
}
