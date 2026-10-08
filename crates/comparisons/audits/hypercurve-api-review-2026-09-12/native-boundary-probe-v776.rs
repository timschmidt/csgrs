use hypercurve::{BooleanOp, Classification, Contour2, Curve2, CurveContext, CurvePath2, CurveRegion2, FillRule, LineSeg2, Point2, RegionPointLocation, Segment2};
fn rectangle(x0:i32,y0:i32,x1:i32,y1:i32)->Contour2 {
    let vertices=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)];
    Contour2::try_new(vertices.windows(2).map(|edge|Segment2::Line(LineSeg2::try_new(Point2::from_values(edge[0].0,edge[0].1),Point2::from_values(edge[1].0,edge[1].1)).unwrap())).collect()).unwrap()
}
fn paths(contours:&[Contour2])->Vec<CurvePath2> {
    contours.iter().map(|contour|CurvePath2::try_new(contour.segments().iter().map(|s|match s{Segment2::Line(l)=>Curve2::from(l.clone()),Segment2::Arc(a)=>Curve2::from(a.clone())}).collect()).unwrap()).collect()
}
fn main(){
 let mut controls=0;let mut blocked_roundtrips=0;
 for (pi,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  for (label,contours,probes) in [
   ("nested",vec![rectangle(0,0,8,8),rectangle(2,2,6,6)],vec![(1,1,RegionPointLocation::Inside),(4,4,RegionPointLocation::Outside)]),
   ("disjoint",vec![rectangle(0,0,2,2),rectangle(4,0,6,2)],vec![(1,1,RegionPointLocation::Inside),(3,1,RegionPointLocation::Outside)]),
   ("crossing",vec![rectangle(0,0,4,4),rectangle(2,-1,6,3)],vec![(1,1,RegionPointLocation::Inside),(3,1,RegionPointLocation::Outside)]),
   ("shared_edge",vec![rectangle(0,0,4,4),rectangle(4,0,8,4)],vec![(4,2,RegionPointLocation::Inside),(7,2,RegionPointLocation::Inside)]),
   ("point_touch",vec![rectangle(0,0,4,4),rectangle(4,4,8,8)],vec![(4,4,RegionPointLocation::Boundary),(6,6,RegionPointLocation::Inside)]),
  ] {
   let generic=CurveRegion2::try_from_boundary_paths(&paths(&contours),FillRule::EvenOdd,policy).unwrap().into_value();
   for (x,y,want) in probes{
    assert_eq!(generic.classify_point(&Point2::from_values(x,y).into(),policy).unwrap().into_value(),Classification::Decided(want));
   }
   let replay=generic.boolean_region(&generic,BooleanOp::Xor,policy).unwrap().into_value();assert!(replay.is_empty());
   let accepted=match CurveRegion2::try_from_native_boundary_contours(contours,policy){Ok(v)=>matches!(v.into_value(),Classification::Decided(_)),Err(_)=>false};
   if label=="nested"||label=="disjoint"{assert!(accepted);controls+=1;}else{assert!(!accepted);blocked_roundtrips+=1;}
   println!("policy={pi} case={label} generic_exact=true native_admitted={accepted}");
  }
 }
 println!("controls={controls} blocked_roundtrips={blocked_roundtrips}");
 assert_eq!(controls,4);assert_eq!(blocked_roundtrips,6);
}
