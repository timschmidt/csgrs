use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn decided<T>(v: Classification<T>)->T {match v {Classification::Decided(v)=>v, x=>panic!("unresolved: {}",matches!(x,Classification::Uncertain(_)))}}
fn main() {
 let policy=CurveContext::STRICT;
 let selecting=Curve2::from(QuadraticBezier2::new(p(0,0),p(0,0),p(1,0)));
 let q=(Real::one()/Real::from(2)).unwrap();
 let cross=Curve2::from(LineSeg2::try_new(Point2::new(q.clone(),Real::from(-1)),Point2::new(q,Real::one())).unwrap());
 let selected=selecting.intersect_curve(&cross,&policy).unwrap().value;
 let parameter=decided(selected.contacts()[0].first().parameter(&policy).unwrap());
 let source=Curve2::from(RationalBezier2::try_new(vec![p(9,0),p(-7,3),p(-7,-10),p(9,9)],vec![Real::one();4]).unwrap());
 let same=source.intersect_curve(&source,&policy).unwrap().value;
 println!("same complete={} contacts={} overlaps={}",same.is_complete(),same.contacts().len(),same.overlaps().len());
 let (first,second)=source.split_at(parameter,&policy).unwrap().value;
 let result=first.intersect_curve(&second,&policy).unwrap().value;
 println!("cuts complete={} contacts={} overlaps={} blockers={:?}",result.is_complete(),result.contacts().len(),result.overlaps().len(),result.blockers());
 for contact in result.contacts() {println!("origin={:?} cross={:?}",contact.point().coincides_with(&p(0,0).into(),&policy).value,contact.tangent_cross_sign());}
}
