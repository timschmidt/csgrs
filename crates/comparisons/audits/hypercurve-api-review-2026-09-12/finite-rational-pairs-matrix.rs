use hypercurve::*;
fn q(n:i32,d:i32)->Real {(Real::from(n)/Real::from(d)).unwrap()}
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn exact<T>(x:Classification<T>)->T {match x {Classification::Decided(x)=>x,Classification::Uncertain(e)=>panic!("{e:?}")}}
fn certified<T>(x:CurveOutcome<T>)->T {assert_eq!(x.certainty,CurveCertainty::Certified);x.value}
fn chord(a:Point2,b:Point2,policy:&CurveContext)->BezierSplitFragment2 {
 BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(a.into(),b.into(),policy).unwrap()))
}
fn export(fragments:Vec<BezierSplitFragment2>,policy:&CurveContext)->Vec<Curve2> {
 let boundary=CurveRegionBoundaryLoop2::new(fragments,policy).unwrap();
 let region=CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap();
 exact(certified(region.boundary_paths(policy).unwrap()))[0].curves().to_vec()
}
fn chord_curve(a:Point2,b:Point2,_policy:&CurveContext)->Curve2 {
 Curve2::from(LineSeg2::try_new(a,b).unwrap())
}
fn source(start:i32,end:i32,line:bool,policy:&CurveContext)->Vec<Curve2> {
 let curve=if line {QuadraticBezier2::new(p(0,0),p(0,0),p(1,0))}
           else {QuadraticBezier2::new(p(0,0),Point2::new(q(1,2),Real::zero()),p(1,1))};
 let a=curve.point_at(Real::from(start));let b=curve.point_at(Real::from(end));
 let fragment=BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Exact(start.into()),end:BezierParameter2::Exact(end.into()),source_curve:BezierSubcurve2::Quadratic(curve),start_image:None,end_image:None};
 if line {
  let c=Point2::new(a.x()-(b.y()-a.y()),a.y()+(b.x()-a.x()));
  export(vec![fragment,chord(b,c.clone(),policy),chord(c,a,policy)],policy)
 } else {export(vec![fragment,chord(b,a,policy)],policy)}
}
fn nonlinear_pair(policy:&CurveContext)->(Curve2,Curve2) {
 let first=RationalBezier2::try_new(vec![p(0,0),Point2::new(q(1,2),Real::zero()),p(1,1)],vec![Real::one();3]).unwrap();
 let second=RationalBezier2::try_new(vec![p(0,0),p(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),p(1,1)],vec![Real::one();5]).unwrap();
 let mut publish=|source,hi| {
  let fragment=BezierSplitFragment2::RetainedBezier{reversed:false,start:BezierParameter2::Exact(Real::one()),end:BezierParameter2::Exact(Real::from(hi)),source_curve:BezierSubcurve2::Rational(source),start_image:None,end_image:None};
  export(vec![fragment,chord(p(4,16),p(1,1),policy)],policy).remove(0)
 };
 (publish(first,4),publish(second,2))
}
fn root(degree:usize,policy:&CurveContext)->CurveParameter2 {
 let mut c=vec![Real::zero();degree+1];c[0]=(-2).into();c[degree]=Real::one();
 let poly=exact(BezierParameterPolynomial::try_new_power_basis(c,policy).unwrap());
 let r=exact(poly.isolate_incident_ray_roots(&Real::one(),BezierParameterRayDirection2::Increasing,policy).unwrap());
 assert_eq!(r.len(),1);r[0].clone().into()
}
fn oriented(c:&Curve2,reverse:bool,policy:&CurveContext)->Curve2 {if reverse {certified(c.reversed(policy).unwrap())} else {c.clone()}}
fn same(a:&CurvePoint2,b:&CurvePoint2,policy:&CurveContext){assert_eq!(certified(a.coincides_with(b,policy)),Classification::Decided(true));}
fn main(){let(mut queries,mut replays,mut restrictions)=(0,0,0);
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  for (lo,hi,y,count) in [(1,2,2,1),(2,4,8,1),(-2,-1,2,1),(-2,2,1,2)] {
   let curves=source(lo,hi,false,&policy);let cutter=chord_curve(p(lo-1,y),p(hi+1,y),&policy);
   for (other,count) in [(&curves[1],2),(&cutter,count)] {
    for reverse_a in [false,true] {for reverse_b in [false,true]{
     let a=oriented(&curves[0],reverse_a,&policy);let b=oriented(other,reverse_b,&policy);
     for (a,b) in [(&a,&b),(&b,&a)]{
      let result=certified(a.intersect_curve(b,&policy).unwrap());queries+=1;
      assert!(result.is_complete(),"{:?}",result.blockers());assert_eq!(result.contacts().len(),count);assert!(result.overlaps().is_empty());
      for contact in result.contacts(){for (curve,location) in [(a,contact.first()),(b,contact.second())]{
       let point=certified(curve.point_at(&exact(location.parameter(&policy).unwrap()),&policy).unwrap());same(&point,contact.point(),&policy);replays+=1;
      }}
     }
    }}
   }
  }
  let (first,second)=nonlinear_pair(&policy);
  let x=root(2,&policy);let u=root(4,&policy);
  for reverse in [false,true] {let first=oriented(&first,reverse,&policy);
   for swapped in [false,true] {
    let (a,b,first_cut,second_cut,first_end,second_end)=if swapped {(&second,&first,u.clone(),x.clone(),2,4)} else {(&first,&second,x.clone(),u.clone(),4,2)};
    let result=certified(a.intersect_curve(b,&policy).unwrap());queries+=1;
    assert!(result.is_complete(),"{:?}",result.blockers());assert!(result.contacts().is_empty());assert_eq!(result.overlaps().len(),1);
    let mut overlap=result.overlaps()[0].clone();
    for _ in 0..8 {
     overlap=exact(certified(overlap.restrict([first_cut.clone(),Real::from(first_end).into()],[second_cut.clone(),Real::from(second_end).into()],&policy).unwrap())).unwrap();restrictions+=1;
     for (x,y) in [(overlap.first_range().start(),overlap.second_range().start()),(overlap.first_range().end(),overlap.second_range().end())] {
      same(&certified(a.point_at(x,&policy).unwrap()),&certified(b.point_at(y,&policy).unwrap()),&policy);replays+=2;
     }
    }
    let topology=certified(a.intersection_topology(b,&policy).unwrap());
    assert!(topology.result().is_complete());
    for piece in topology.first() {let result=certified(piece.intersect_curve(b,&policy).unwrap());queries+=1;assert!(result.is_complete());assert!(!result.overlaps().is_empty());}
    let parts=certified(a.split_at(first_cut,&policy).unwrap());
    for piece in [parts.0,parts.1] {let result=certified(piece.intersect_curve(b,&policy).unwrap());queries+=1;assert!(result.is_complete(),"{:?}",result.blockers());assert_eq!(result.overlaps().len(),1);}
   }
  }
  for (lo,hi,cut_lo,cut_hi) in [(1,2,2,3),(-2,-1,2,3)]{
   let a=source(lo,hi,true,&policy).remove(0);let b=chord_curve(p(cut_lo,0),p(cut_hi,0),&policy);
   for reverse in [false,true]{let a=oriented(&a,reverse,&policy);
    for swapped in [false,true]{let (first,second)=if swapped{(&b,&a)}else{(&a,&b)};
     let result=certified(first.intersect_curve(second,&policy).unwrap());queries+=1;
     assert!(result.is_complete(),"{:?}",result.blockers());assert!(result.contacts().is_empty());assert_eq!(result.overlaps().len(),1);
     let overlap=&result.overlaps()[0];
     let range=if swapped{overlap.first_range()}else{overlap.second_range()};
     let source_limits=if lo<0 {[Real::from(-2).into(),(-q(3,2)).into()]}else{[q(3,2).into(),Real::from(2).into()]};
     let chord_limits=[range.start().clone(),range.end().clone()];
     let (fa,fb)=if swapped{(chord_limits,source_limits)}else{(source_limits,chord_limits)};
     let clipped=exact(certified(overlap.restrict(fa,fb,&policy).unwrap())).unwrap();restrictions+=1;
     for (x,y) in [(clipped.first_range().start(),clipped.second_range().start()),(clipped.first_range().end(),clipped.second_range().end())]{
      same(&certified(first.point_at(x,&policy).unwrap()),&certified(second.point_at(y,&policy).unwrap()),&policy);replays+=2;
     }
    }
   }
  }
 }
 println!("{{\"queries\":{queries},\"point_replays\":{replays},\"restrictions\":{restrictions}}}");
}
