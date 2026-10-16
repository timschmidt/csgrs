use csgrs::{GeometryContext, Real, solid};
use hypermesh::{Classification, Plane, PredicatePolicy};
fn main() {
    let cube=solid::cube(Real::try_from(1.5_f64).unwrap()).unwrap();
    let sphere=solid::sphere(Real::try_from(0.4_f64).unwrap(),16,8).unwrap();
    let context=GeometryContext::STRICT.mesh_context();
    println!("sphere positions={} triangles={}",sphere.positions.len(),sphere.triangles.len());
    println!("pwn {:?}",sphere.validate_closed_pwn(&context));
    println!("convex {:?}",hypermesh::certify_convex_mesh(&context,sphere.as_ref()));
    println!("original_sum {:?}",solid::minkowski_sum_with_context(&cube,&sphere,&GeometryContext::STRICT).map(|out|(out.value.positions.len(),out.value.triangles.len(),out.certainty)));
    let mut unknown=0;
    let mut negative=0;
    for (face,t) in sphere.triangles.iter().enumerate() {
        let [a,b,c]=sphere.triangle_points(*t).unwrap();
        let plane=Plane::from_points(a,b,c);
        for (vertex,point) in sphere.positions.iter().enumerate() {
            match hypermesh::classify_point(&context,point,&plane) {
                Err(error)=>{
                    let orientation=hyperlimit::orient3(a,b,c,point,PredicatePolicy::STRICT);
                    if unknown<6 {println!("unknown face={face} vertex={vertex} own={} plane={error:?} orient={orientation:?}",t.indices().contains(&vertex));}
                    unknown+=1;
                },
                Ok(out) if out.value==Classification::Positive=>negative+=1,
                _=>{},
            }
        }
    }
    println!("plane_unknown={unknown} plane_outside={negative}");
}
