use hypercurve::*;
fn main() {
 let points=[Point2::from_values(-5,-5),Point2::from_values(5,-5),Point2::from_values(5,5),Point2::from_values(-5,5)];
 let contour=Contour2::try_new((0..4).map(|i| Segment2::Line(LineSeg2::try_new(points[i].clone(),points[(i+1)%4].clone()).unwrap())).collect()).unwrap();
 let region=CurveRegion2::try_from_native_material_contours(vec![contour],&CurveContext::STRICT).unwrap().value;
 for x in [0,5,6] {
   let p=Point2::from_values(x,0);
   let curve=Curve2::from(QuadraticBezier2::new(p.clone(),p.clone(),p));
   match curve.trim_inside_region(&region,&CurveContext::STRICT) {
     Ok(result)=>println!("x={x} pieces={} certainty={:?}",result.value.len(),result.certainty),
     Err(error)=>println!("x={x} error={error:?}"),
   }
 }
}
