use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn exact<T>(v:CurveOutcome<T>)->T {assert_eq!(v.certainty,CurveCertainty::Certified);v.value}
fn decided<T>(v:Classification<T>)->T {match v {Classification::Decided(v)=>v,_=>panic!("exact fixture")}}
fn main(){
 let mut failures=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  for finite in [false,true] {
   let source=if finite {
    let source=RationalBezier2::try_new(vec![p(0,0),p(0,0),Point2::new(q(1,6),0.into()),Point2::new(q(1,2),0.into()),p(1,1)],vec![Real::one();5]).unwrap();
    let fragment=BezierSplitFragment2::RetainedBezier{source_curve:BezierSubcurve2::Rational(source),start:BezierParameter2::Exact((-2).into()),end:BezierParameter2::Exact(1.into()),reversed:false,start_image:None,end_image:None};
    let chord=BezierSplitFragment2::AlgebraicChord(decided(BezierAlgebraicChord2::try_new(p(1,1).into(),p(4,16).into(),&policy).unwrap()));
    let raw=CurveRegion2::new(vec![CurveRegionBoundaryLoop2::new(vec![fragment,chord],&policy).unwrap()]).unwrap();
    decided(exact(raw.boundary_paths(&policy).unwrap()))[0].curves()[0].clone()
   } else {Curve2::from(RationalBezier2::try_new(vec![p(4,16),p(1,-8),Point2::new(q(-1,2),4.into()),Point2::new(q(-1,2),(-2).into()),p(1,1)],vec![Real::one();5]).unwrap())};
   for reversed in [false,true] {
    let mut curves=vec![source.clone(),Curve2::from(LineSeg2::try_new(p(1,1),p(4,16)).unwrap())];
    if reversed {curves=curves.into_iter().rev().map(|c|exact(c.reversed(&policy).unwrap())).collect();}
    let path=CurvePath2::try_new(curves).unwrap();
    let raw=exact(CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],&policy).unwrap());
    let label=format!("finite={finite} reversed={reversed} policy={policy:?}");
    match raw.regularized_region(&policy) {
     Ok(out)=>{let region=exact(out);println!("{label} loops={}",region.boundary_loops().len());
      for (point,expected) in [(p(2,5),RegionPointLocation::Inside),(p(0,0),RegionPointLocation::Outside),(Point2::new(q(1,2),q(1,4)),RegionPointLocation::Outside),(p(1,1),RegionPointLocation::Boundary),(p(2,4),RegionPointLocation::Boundary),(p(2,6),RegionPointLocation::Boundary),(p(2,7),RegionPointLocation::Outside)] {
       let result=region.classify_point(&point,&policy);let valid=matches!(&result,Ok(out) if out.certainty==CurveCertainty::Certified && out.value==Classification::Decided(expected));
       println!("{label} point={point:?} expected={expected:?} result={result:?} valid={valid}");if !valid {failures+=1;}
      }
     },Err(error)=>{println!("{label} ERROR {error:?}");failures+=1;}
    }
   }
  }
 }
 println!("failures={failures}");assert_eq!(failures,0);
}
