use hypercurve::*;
fn exact<T:std::fmt::Debug>(v:Classification<T>)->T {match v{Classification::Decided(v)=>v,v=>panic!("{v:?}")}}
fn q(n:i32,d:i32)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn same(a:&CurvePoint2,b:&CurvePoint2,p:&CurveContext){let r=a.coincides_with(b,p);assert_eq!(r.certainty,CurveCertainty::Certified);assert_eq!(r.value,Classification::Decided(true));}
fn main(){
 let mut cases=0;let mut queries=0;let mut replays=0;let mut repeated=0;let mut topology=0;
 for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
 for scale in [1,2]{for (dx,dy) in [(0,0),(-3,5)]{for radius in [q(scale,8),q(scale,4),q(scale,2)]{for reverse in [false,true]{
 let p=|x:i32,y:i32|Point2::from_values(x*scale+dx,y*scale+dy);
 let path=CurvePath2::try_new(vec![LineSeg2::try_new(p(-4,0),p(0,0)).unwrap().into(),QuadraticBezier2::new(p(0,0),p(0,1),p(1,2)).into()]).unwrap();
 let path=if reverse {path.reversed(&policy).unwrap().value}else{path};
 let fillet=path.fillet_vertex_by_radius(1,radius.clone(),CurveCornerMode2::TrimOnly,&policy).unwrap();
 assert_eq!(fillet.certainty,CurveCertainty::Certified);
 let CurveCornerSolutions2::Unique(path)=fillet.value else{panic!("unique fillet")};
 assert_eq!(path.curves().len(),3);let circle=&path.curves()[1];assert_eq!(circle.family(),CurveFamily2::CircularArc);
 for curve in path.curves(){let r=curve.intersect_curve(curve,&policy).unwrap();assert_eq!(r.certainty,CurveCertainty::Certified);assert!(r.value.is_complete(),"self {:?}",r.value.blockers());assert_eq!(r.value.overlaps().len(),1);assert!(r.value.contacts().is_empty());queries+=1;}
 for i in [0,2]{for ar in [false,true]{for br in [false,true]{for swap in [false,true]{
 let a=if ar{circle.reversed(&policy).unwrap().value}else{circle.clone()};let b=if br{path.curves()[i].reversed(&policy).unwrap().value}else{path.curves()[i].clone()};
 let(a,b)=if swap{(&b,&a)}else{(&a,&b)};
 let r=a.intersect_curve(b,&policy).unwrap();assert_eq!(r.certainty,CurveCertainty::Certified);assert!(r.value.is_complete(),"case{cases} neighbor{i} ar{ar} br{br} swap{swap}: {:?}",r.value.blockers());assert_eq!(r.value.contacts().len(),1);assert!(r.value.overlaps().is_empty());
 let c=&r.value.contacts()[0];assert!(!c.is_certified_transverse());
 for(curve,loc)in[(a,c.first()),(b,c.second())]{let t=exact(loc.parameter(&policy).unwrap());let p=curve.point_at(&t,&policy).unwrap();assert_eq!(p.certainty,CurveCertainty::Certified);same(&p.value,c.point(),&policy);replays+=1;}queries+=1;
 }}}
 let r=circle.intersection_topology(&path.curves()[i],&policy).unwrap();assert_eq!(r.certainty,CurveCertainty::Certified);assert_eq!(r.value.first().len(),1);assert_eq!(r.value.second().len(),1);topology+=1;
 }
 let mut current=circle.clone();for _ in 0..8{let r=current.intersect_curve(circle,&policy).unwrap();assert_eq!(r.certainty,CurveCertainty::Certified);assert!(r.value.is_complete());assert_eq!(r.value.overlaps().len(),1);let domain=current.parameter_domain();current=current.subcurve(domain.start().clone(),domain.end().clone(),&policy).unwrap().value;repeated+=1;}
 cases+=1;println!("case {cases}: certified scale={scale} translation={dx}/{dy} reverse={reverse}");
 }}}}}
 println!("{{\"cases\":{cases},\"pair_queries\":{queries},\"point_replays\":{replays},\"topology_queries\":{topology},\"repeated_intersections\":{repeated}}}");
}
