use hypercurve::*;
fn p(x:i32,y:i32)->Point2 {Point2::from_values(x,y)}
fn main(){
 let policy=CurveContext::STRICT;
 let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into()]).unwrap();
 let f=path.fillet_vertex_by_radius(1,(Real::one()/Real::from(4)).unwrap(),CurveCornerMode2::TrimOnly,&policy).unwrap();
 let CurveCornerSolutions2::Unique(path)=f.value else {panic!()};
 for i in [0,2] {for reverse in [false,true] {
 let circle=if reverse {path.curves()[1].reversed(&policy).unwrap().value} else {path.curves()[1].clone()};
 let other=&path.curves()[i];
 for swap in [false,true] {let(a,b)=if swap{(other,&circle)}else{(&circle,other)};
 let r=a.intersect_curve(b,&policy).unwrap();
 println!("neighbor{i} reversed{reverse} swapped{swap} {:?} contacts{} blockers{:?}",r.certainty,r.value.contacts().len(),r.value.blockers());
 for c in r.value.contacts(){for(label,curve,loc)in[("first",a,c.first()),("second",b,c.second())]{
 let Classification::Decided(t)=loc.parameter(&policy).unwrap() else {panic!()};
 let p=curve.point_at(&t,&policy).unwrap();
 println!(" {label} family{:?} point{:?} equal{:?} reverse_equal{:?}",curve.family(),p.certainty,p.value.coincides_with(c.point(),&policy),c.point().coincides_with(&p.value,&policy));
 }}
 }}}
}
