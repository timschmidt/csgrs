use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn region(path:CurvePath2,policy:&CurveContext)->Result<CurveRegion2,Box<dyn std::error::Error>> {
 Ok(CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],policy)?.into_value())
}
fn main()->Result<(),Box<dyn std::error::Error>> {
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
  let Classification::Decided(curve)=RationalBezier2::from_homogeneous_controls(vec![
    HomogeneousControl2::new(Real::one(),Real::zero(),Real::one()),
    HomogeneousControl2::new(Real::zero(),Real::one(),Real::zero()),
    HomogeneousControl2::new(-Real::one(),Real::zero(),Real::one())], &policy)? else {panic!("construction");};
  let path=CurvePath2::try_new(vec![Curve2::from(curve),Curve2::from(LineSeg2::try_new(p(-1,0),p(1,0))?)])?;
  println!("region admission");
  let material=region(path,&policy)?;
  println!("location: {:?}",material.classify_point(&Point2::new(Real::zero(),q(1,2)),&policy)?);
  let rectangle=region(CurvePath2::try_new([p(0,-1),p(2,-1),p(2,2),p(0,2),p(0,-1)].windows(2)
   .map(|w|Curve2::from(LineSeg2::try_new(w[0].clone(),w[1].clone()).unwrap())).collect())?, &policy)?;
  println!("boolean clipping");
  let results=material.boolean_regions(&rectangle,&policy)?.into_value();
  let clipped=results.intersection();
  println!("clipped loops: {}",clipped.boundary_loops().len());
  for fillet in [false,true] {
   let solutions=if fillet {
     println!("fillet after boolean");
     clipped.fillet_loop_vertex_by_radius(0,1,q(1,8),CurveCornerMode2::TrimOnly,&policy)?.into_value()
   } else {
     println!("chamfer after boolean");
     clipped.chamfer_loop_vertex_by_setbacks(0,1,q(1,8),q(1,8),CurveCornerMode2::TrimOnly,&policy)?.into_value()
   };
   let regions=match solutions {CurveCornerSolutions2::Unique(r)=>vec![r],CurveCornerSolutions2::Multiple(r)=>r,CurveCornerSolutions2::NoSolution(r)=>panic!("no corner: {r:?}")};
   for edited in regions {
     let displaced=edited.offset(q(1,32),&OffsetCornerStyle2::Round,&policy)?.into_value();
     let replay=displaced.boolean_regions(&rectangle,&policy)?.into_value();
     println!("corner-offset-boolean loops: {}",replay.intersection().boundary_loops().len());
   }
  }
  println!("round offset after boolean");
  let offset=clipped.offset(q(1,4),&OffsetCornerStyle2::Round,&policy)?.into_value();
  println!("offset loops: {}",offset.boundary_loops().len());
 }
 Ok(())
}
