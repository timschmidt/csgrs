use hypercurve::{BezierAlgebraicImageStatus, BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification, CurveContext, CurvePoint2, Point2, RationalQuadraticBezier2, Real};
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn decided<T>(c:Classification<T>)->T{match c{Classification::Decided(v)=>v,Classification::Uncertain(r)=>panic!("unexpected control uncertainty: {r:?}")}}
fn parameter(n:i32,d:i32,policy:&CurveContext)->BezierAlgebraicParameter2{
 let polynomial=decided(BezierParameterPolynomial::try_new_power_basis(vec![Real::from(-n),Real::from(d)],policy).unwrap());
 let root=q(n,d);let interval=decided(BezierParameterInterval::try_new(&root-q(1,10),&root+q(1,10),policy).unwrap());
 decided(BezierAlgebraicParameter2::try_isolate(polynomial,interval,policy).unwrap())
}
fn main(){
 let curve=RationalQuadraticBezier2::try_new(Point2::from_values(0,0),Point2::from_values(1,1),Point2::from_values(2,0),Real::one(),-Real::one(),Real::one()).unwrap();
 let mut finite_controls=0;let mut failed_image_admissions=0;
 for (pi,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate(){
  for (numerator,expected_x) in [(1,-1),(3,3)]{
   let image=curve.point_at_algebraic_parameter(&parameter(numerator,4,policy),policy).unwrap();
   assert_eq!(image.status(),BezierAlgebraicImageStatus::Transformed);
   let point=CurvePoint2::from(image);let expected=CurvePoint2::from(Point2::new(Real::from(expected_x),q(-3,2)));
   assert!(matches!(point.coincides_with(&expected,policy).unwrap().into_value(),Classification::Decided(true)));
   finite_controls+=1;
  }
  // D(t)=(1-2t)^2 vanishes at1/2 while Ny(1/2)=-1/2: no affine point exists.
  let image=curve.point_at_algebraic_parameter(&parameter(1,2,policy),policy).unwrap();let status=image.status();
  assert!(matches!(status,BezierAlgebraicImageStatus::XImageFailed|BezierAlgebraicImageStatus::YImageFailed));
  assert!(image.x().is_none()&&image.y().is_none()&&image.retained_coordinate_polynomials().is_none());
  let point=CurvePoint2::from(image);
  let self_coincidence=match point.coincides_with(&point,policy){Ok(v)=>match v.into_value(){Classification::Decided(true)=>"certified_equal",Classification::Decided(false)=>"certified_unequal",Classification::Uncertain(_)=>"uncertain"},Err(_)=>"error"};
  let bounds_decided=matches!(point.bounds(policy).into_value(),Classification::Decided(_));
  println!("policy={pi} image_status={status:?} admitted_as_curve_point=true self_coincidence={self_coincidence} bounds_decided={bounds_decided}");
  failed_image_admissions+=1;
 }
 println!("finite_controls={finite_controls} failed_image_admissions={failed_image_admissions}");
}
