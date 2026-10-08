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
fn chord_curve(a:Point2,b:Point2,policy:&CurveContext)->Curve2 {
 let c=Point2::new(a.x()-(b.y()-a.y()),a.y()+(b.x()-a.x()));
 export(vec![chord(a.clone(),b.clone(),policy),chord(b,c.clone(),policy),chord(c,a,policy)],policy).remove(0)
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
