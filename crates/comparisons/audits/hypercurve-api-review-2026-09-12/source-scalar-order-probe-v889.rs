use hypercurve::{BezierAlgebraicEndpointImage2,BezierAlgebraicParameter2,BezierAlgebraicTangentVector2,BezierArrangementFragment2,BezierArrangementGraph2,BezierParameter2,BezierParameterInterval,BezierParameterPolynomial,BezierSplitFragment2,BezierSubcurve2,Classification,CurveContext,HomogeneousControl2,Point2,QuadraticBezier2,RationalBezier2,Real};
fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(v)=>v,Classification::Uncertain(r)=>panic!("unexpected source blocker {r:?}")}}
fn line(index:usize,start:Point2,end:Point2)->BezierArrangementFragment2{let two=Real::from(2);let mid=Point2::new(((start.x()+end.x())/&two).unwrap(),((start.y()+end.y())/two).unwrap());BezierArrangementFragment2::new(index,0,BezierSplitFragment2::Materialized{start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),curve:BezierSubcurve2::Quadratic(QuadraticBezier2::new(start,mid,end))})}
fn choose(n:i32,k:i32)->i32 {if n<k{return 0}(0..k).fold(1,|a,j|a*(n-j))/(1..=k).product::<i32>()}
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn main(){let mut blocked=0;for(pi,policy)in[CurveContext::STRICT,CurveContext::APPROXIMATE_512].into_iter().enumerate(){
 let parameter=decided(BezierAlgebraicParameter2::try_isolate(decided(BezierParameterPolynomial::try_new_power_basis(vec![-Real::pi(),Real::zero(),Real::zero(),Real::from(4)],&policy).unwrap()),decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),&policy).unwrap()),&policy).unwrap());
 // A=(P,t*P), B=(P,t*P+P^2), P=4*t^3-pi, both degree-elevated to six.
 // At P(alpha)=0 they share a nonzero tangent. At matched x=P(t)>0,
 // B_y-A_y=x^2>0, so A is encountered first from the incoming horizontal.
 let mut fragments=vec![line(0,Point2::from_values(-1,0),Point2::from_values(0,0))];
 for second in [false,true] {
  let controls=(0..=6).map(|k|{
   let x=-Real::pi()+q(choose(k,3),5);
   let mut y=-Real::pi()*q(k,6)+q(4*choose(k,4),15);
   if second {y=y+Real::pi()*Real::pi()-Real::pi()*q(2*choose(k,3),5)+Real::from(if k==6{16}else{0});}
   HomogeneousControl2::new(x,y,Real::one())
  }).collect();
  let source=BezierSubcurve2::Rational(decided(RationalBezier2::from_homogeneous_controls(controls,&policy).unwrap()));
  let image=decided(BezierAlgebraicEndpointImage2::from_source_curve(&source,&parameter,&policy).unwrap());
  let point=decided(image.point().unwrap());
  let represented_point=point.x().and_then(|c|c.representation()).is_some()&&point.y().and_then(|c|c.representation()).is_some();
  let tangent=decided(image.tangent().unwrap());let retained=tangent.retained_parameter().is_some();let represented_vector=BezierAlgebraicTangentVector2::from_image(tangent).represented_coordinates().is_some();
  println!("policy={pi} second={second} point_represented={represented_point} retained_tangent={retained} vector_represented={represented_vector}");assert!(represented_point&&retained&&!represented_vector);
  fragments.push(BezierArrangementFragment2::new(if second{2}else{1},0,BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Algebraic(parameter.clone()),end:BezierParameter2::Exact(Real::one()),source_curve:source,start_image:Some(image),end_image:None}));
 }
 let graph=BezierArrangementGraph2::new(fragments).unwrap();
 match graph.traverse_retained_with_tangent_order(&policy){Classification::Decided(v)=>{assert_eq!(v.chains()[0].fragment_indices(),[0,1]);assert_eq!(v.chains()[1].fragment_indices(),[2]);println!("policy={pi} decided=true");},Classification::Uncertain(r)=>{blocked+=1;println!("policy={pi} blocked={r:?}");}}
}println!("complete requests=2 blocked={blocked}");}
