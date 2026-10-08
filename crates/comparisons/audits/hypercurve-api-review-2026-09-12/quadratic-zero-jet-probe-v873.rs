use hypercurve::{BezierAlgebraicEndpointImage2,BezierAlgebraicParameter2,BezierArrangementFragment2,BezierArrangementGraph2,BezierArrangementTraversal2,BezierParameter2,BezierParameterInterval,BezierParameterPolynomial,BezierSplitFragment2,BezierSubcurve2,Classification,CurveContext,Point2,QuadraticBezier2,RationalBezier2,RationalQuadraticBezier2,Real};
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(v)=>v,Classification::Uncertain(r)=>panic!("unexpected source blocker {r:?}")}}
fn source(family:usize,second:bool,shifted:bool)->BezierSubcurve2{
 let [a,b,c]=if shifted{[Point2::new(if second{q(-1,4)}else{q(-1,2)},q(1,4)),Point2::new(if second{q(-1,4)}else{Real::zero()},q(-1,4)),Point2::new(if second{q(3,4)}else{q(1,2)},q(1,4))]}else{[Point2::from_values(0,0),Point2::new(q(1,2),Real::zero()),Point2::from_values(if second{2}else{1},1)]};
 if family==0{BezierSubcurve2::Quadratic(QuadraticBezier2::new(a,b,c))}else{let conic=RationalQuadraticBezier2::try_new(a,b,c,Real::one(),Real::one(),Real::one()).unwrap();if family==1{BezierSubcurve2::RationalQuadratic(conic)}else{BezierSubcurve2::Rational(RationalBezier2::from(conic))}}
}
fn materialized(index:usize,curve:BezierSubcurve2)->BezierArrangementFragment2{BezierArrangementFragment2::new(index,0,BezierSplitFragment2::Materialized{start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),curve})}
fn incoming()->BezierArrangementFragment2{materialized(0,BezierSubcurve2::Quadratic(QuadraticBezier2::new(Point2::from_values(-1,0),Point2::new(q(-1,2),Real::zero()),Point2::from_values(0,0))))}
fn outcome(value:Classification<BezierArrangementTraversal2>,label:&str)->usize{match value{Classification::Decided(v)=>{assert_eq!(v.chains()[0].fragment_indices(),[0,2]);assert_eq!(v.chains()[1].fragment_indices(),[1]);println!("{label} decided=true");0},Classification::Uncertain(r)=>{println!("{label} blocked={r:?}");1}}}
fn main(){let mut blocked=0;for(pi,policy)in[CurveContext::STRICT,CurveContext::APPROXIMATE_512].into_iter().enumerate(){
 let parameter=decided(BezierAlgebraicParameter2::try_isolate(decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-1),Real::from(2)],&policy).unwrap()),decided(BezierParameterInterval::try_new(Real::zero(),Real::one(),&policy).unwrap()),&policy).unwrap());
 for family in 0..3{
  let graph=BezierArrangementGraph2::new(vec![incoming(),materialized(1,source(family,false,false)),materialized(2,source(family,true,false))]).unwrap();
  blocked+=outcome(graph.traverse_with_tangent_order(&policy),&format!("policy={pi} family={family} kind=native"));
  blocked+=outcome(graph.traverse_retained_with_tangent_order(&policy),&format!("policy={pi} family={family} kind=retained"));
  let endpoint=|index:usize,second:bool|{let source=source(family,second,true);let image=decided(BezierAlgebraicEndpointImage2::from_source_curve(&source,&parameter,&policy).unwrap());BezierArrangementFragment2::new(index,0,BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Algebraic(parameter.clone()),end:BezierParameter2::Exact(Real::one()),source_curve:source,start_image:Some(image),end_image:None})};
  let graph=BezierArrangementGraph2::new(vec![incoming(),endpoint(1,false),endpoint(2,true)]).unwrap();
  blocked+=outcome(graph.traverse_retained_with_tangent_order(&policy),&format!("policy={pi} family={family} kind=selected"));
 }
}println!("complete requests=18 blocked={blocked}");}
