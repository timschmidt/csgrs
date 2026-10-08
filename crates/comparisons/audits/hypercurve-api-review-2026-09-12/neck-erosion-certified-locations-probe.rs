use hypercurve::*;
fn p(x:i64,y:i64)->Point2{Point2::new(Real::from(x),Real::from(y))}
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn dumbbell_shape() -> Contour2 {
    let points = [
        p(0, 0),
        p(4, 0),
        p(4, 1),
        p(8, 1),
        p(8, 0),
        p(12, 0),
        p(12, 4),
        p(8, 4),
        p(8, 3),
        p(4, 3),
        p(4, 4),
        p(0, 4),
        p(0, 0),
    ];
    Contour2::try_new(
        points
            .windows(2)
            .map(|edge| {
                Segment2::Line(LineSeg2::try_new(edge[0].clone(), edge[1].clone()).unwrap())
            })
            .collect(),
    )
    .unwrap()
}


fn main(){
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
  let source=CurveRegion2::try_from_native_material_contours(vec![dumbbell_shape()],&policy).unwrap();
  assert_eq!(source.certainty,CurveCertainty::Certified);
  for style in [OffsetCornerStyle2::Bevel,OffsetCornerStyle2::Round,OffsetCornerStyle2::Miter{limit:Real::one()}]{
   let erosion=source.value.offset(-q(3,2),&style,&policy).unwrap();
   assert_eq!(erosion.certainty,CurveCertainty::Certified);
   assert_eq!(erosion.value.boundary_loops().len(),2);
   for (point,expected) in [(p(2,2),RegionPointLocation::Inside),(p(10,2),RegionPointLocation::Inside),(p(6,2),RegionPointLocation::Outside)]{
    let result=erosion.value.classify_point(&point,&policy).unwrap();
    println!("policy={policy:?} style={style:?} point={point:?} certainty={:?} location={:?}",result.certainty,result.value);
    assert_eq!(result.certainty,CurveCertainty::Certified);
    assert_eq!(result.value,Classification::Decided(expected));
   }
  }
 }
}
