use hypercurve::*;
fn r(v:i32)->Real { v.into() }
fn q(n:i32,d:i32)->Real { (r(n)/r(d)).unwrap() }
fn p(x:i32,y:i32)->Point2 {Point2::new(r(x),r(y))}
fn main() {
    let source=RationalBezier2::try_new(vec![p(0,0),p(0,0),p(0,0),Point2::new(q(1,30),r(0)),Point2::new(q(2,15),q(1,10)),Point2::new(q(2,15),q(1,2))],vec![r(1);6]).unwrap();
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        println!("policy {policy:?}");
        for distance in [q(-1,20),q(1,20)] {
            let parallel=source.parallel_left(distance.clone()).unwrap();
            println!("parallel endpoint {:?}",parallel.point_at(&r(0),&policy));
            println!("global PH {:?}",parallel.exact_pythagorean_hodograph_offset(&policy).map(|v|v.map(|c|c.map(|c|c.curve().degree()))));
            let path=CurvePath2::try_new(vec![Curve2::from(source.clone()),Curve2::from(LineSeg2::try_new(source.end().clone(),Point2::new(r(-1),q(1,2))).unwrap()),Curve2::from(LineSeg2::try_new(Point2::new(r(-1),q(1,2)),p(-1,0)).unwrap()),Curve2::from(LineSeg2::try_new(p(-1,0),p(0,0)).unwrap())]).unwrap();
            let region=match CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],&policy) {
                Ok(region)=>{println!("admission {:?} {} loops",region.certainty,region.value.boundary_loops().len());region.into_value()},
                Err(e)=>{println!("admission error {e:?}");continue;}
            };
            match region.offset(distance.clone(),&OffsetCornerStyle2::Round,&policy) {
                Ok(result)=>{println!("offset {distance:?}: {:?}, {} loops",result.certainty,result.value.boundary_loops().len());println!("location {:?}",result.value.classify_point(&Point2::new(q(-1,2),q(1,4)),&policy));},
                Err(e)=>println!("offset {distance:?} error {e:?}"),
            }
        }
    }
}
