use csgrs::{curve, solid, GeometryContext, Real, errors::{ValidationError,ValidationErrorKind}};
use hypercurve::CurveRegion2;
use std::{path::Path, panic::{catch_unwind,AssertUnwindSafe}};

trait Attempt { fn attempt(self)->Result<CurveRegion2,ValidationError>; }
impl Attempt for CurveRegion2 { fn attempt(self)->Result<CurveRegion2,ValidationError> {Ok(self)} }
impl Attempt for Result<CurveRegion2,ValidationError> {fn attempt(self)->Result<CurveRegion2,ValidationError>{self}}
fn r(n:i32)->Real {n.into()}
fn f(n:f64)->Real {Real::try_from(n).unwrap()}
fn q(n:i32,d:i32)->Real {(r(n)/r(d)).unwrap()}
fn zero()->Real {let s=Real::e().sin();let c=Real::e().cos();&s*&s+&c*&c-r(1)}
fn diagnose(label:&str,kind:ValidationErrorKind,build:impl FnOnce()->Result<CurveRegion2,ValidationError>,candidate:bool){
    match catch_unwind(AssertUnwindSafe(build)) {
        Ok(Ok(region))=>{println!("{label}: success_empty={}",region.is_empty());assert!(!candidate&&region.is_empty());},
        Ok(Err(error))=>{println!("{label}: kind={:?} error={error}",error.kind());assert!(candidate);assert_eq!(error.kind(),kind);},
        Err(_)=>{println!("{label}: panic");assert!(!candidate);},
    }
}
fn fixture(label:&str,region:impl Attempt,out:&Path){
    let region=region.attempt().unwrap_or_else(|e|panic!("{label}: {e}"));
    assert!(!region.is_empty(),"{label} must complete");
    let profiles=region.boundary_profiles().unwrap();assert_eq!(profiles.len(),1,"{label}");
    let (material,holes)=profiles[0].linear_contours().unwrap().unwrap();assert!(holes.is_empty());
    let rows=material.segments().iter().map(|edge|format!("[{},{}]",edge.start().x().to_json(),edge.start().y().to_json())).collect::<Vec<_>>();
    std::fs::write(out.join(format!("{label}.json")),format!("[{}]\n",rows.join(",\n"))).unwrap();
    println!("fixture {label}: one_material=true holes=0 exact_vertices={}",rows.len());
}
fn main(){
    let args=std::env::args().collect::<Vec<_>>();let candidate=args[1]=="candidate";let out=Path::new(&args[2]);std::fs::create_dir_all(out).unwrap();
    use ValidationErrorKind::{InvalidInput as I,Undecided as U,ResourceExhausted as R};
    diagnose("involute module zero",I,||curve::involute_gear(r(0),16,r(20),r(0),r(0),4).attempt(),candidate);
    diagnose("involute angle ninety",I,||curve::involute_gear(r(1),16,r(90),r(0),r(0),4).attempt(),candidate);
    diagnose("involute root negative",I,||curve::involute_gear(r(1),16,r(20),r(100),r(0),4).attempt(),candidate);
    diagnose("cycloidal teeth two",I,||curve::cycloidal_gear(r(1),2,r(1),r(0),4).attempt(),candidate);
    diagnose("cycloidal overlapping flanks",I,||curve::cycloidal_gear(r(1),16,q(3,4),r(0),8).attempt(),candidate);
    diagnose("involute rack no teeth",I,||curve::involute_rack(r(1),0,r(20),r(0),r(0)).attempt(),candidate);
    diagnose("involute rack closed tooth",I,||curve::involute_rack(r(1),4,r(20),r(0),r(4)).attempt(),candidate);
    diagnose("cycloidal rack samples three",I,||curve::cycloidal_rack(r(1),4,r(0),3).attempt(),candidate);
    diagnose("cycloidal rack closed root",I,||curve::cycloidal_rack(r(1),4,r(100),8).attempt(),candidate);
    diagnose("airfoil samples nine",I,||curve::airfoil_naca4(r(2),r(4),r(12),r(1),9).attempt(),candidate);
    diagnose("airfoil missing camber position",I,||curve::airfoil_naca4(r(2),r(0),r(12),r(1),20).attempt(),candidate);
    diagnose("airfoil thickness one hundred",I,||curve::airfoil_naca4(r(2),r(4),r(100),r(1),20).attempt(),candidate);
    diagnose("involute undecided module",U,||curve::involute_gear(zero(),16,r(20),r(0),r(0),4).attempt(),candidate);
    diagnose("cycloidal undecided module",U,||curve::cycloidal_gear(zero(),16,r(1),r(0),4).attempt(),candidate);
    diagnose("involute rack undecided module",U,||curve::involute_rack(zero(),4,r(20),r(0),r(0)).attempt(),candidate);
    diagnose("cycloidal rack undecided module",U,||curve::cycloidal_rack(zero(),4,r(0),8).attempt(),candidate);
    diagnose("airfoil undecided chord",U,||curve::airfoil_naca4(r(2),r(4),r(12),zero(),20).attempt(),candidate);
    for count in [usize::MAX,usize::MAX/2]{
        diagnose(&format!("involute flank capacity {count}"),R,||curve::involute_gear(r(1),16,r(20),r(0),r(0),count).attempt(),candidate);
        diagnose(&format!("cycloidal flank capacity {count}"),R,||curve::cycloidal_gear(r(1),16,r(1),r(0),count).attempt(),candidate);
        diagnose(&format!("involute rack capacity {count}"),R,||curve::involute_rack(r(1),count,r(20),r(0),r(0)).attempt(),candidate);
        diagnose(&format!("cycloidal rack capacity {count}"),R,||curve::cycloidal_rack(r(1),4,r(0),count).attempt(),candidate);
        diagnose(&format!("airfoil capacity {count}"),R,||curve::airfoil_naca4(r(0),r(0),r(12),r(1),count).attempt(),candidate);
    }
    for context in [GeometryContext::STRICT,GeometryContext::APPROXIMATE_512] {
        for (label,result) in [
            ("spur involute",solid::spur_gear_involute(r(0),16,r(20),r(0),r(0),4,r(1),&context)),
            ("spur cycloidal",solid::spur_gear_cycloid(r(0),16,r(1),r(0),4,r(1),&context)),
            ("helical involute",solid::helical_involute_gear(r(0),16,r(20),r(0),r(0),4,r(1),r(10),3,&context)),
        ]{
            match result {
                Ok(result)=>{println!("{label} {context:?}: success_empty={} certainty={:?}",result.value.triangles.is_empty(),result.certainty);assert!(!candidate&&result.value.triangles.is_empty());},
                Err(error)=>{println!("{label} {context:?}: kind={:?}",error.kind());assert!(candidate);assert_eq!(error.kind(),I);},
            }
        }
    }
    fixture("readme-involute",curve::involute_gear(f(0.22),14,f(20.0),f(0.02),f(0.01),5),out);
    fixture("readme-cycloidal",curve::cycloidal_gear(f(0.22),14,f(0.275),f(0.02),8),out);
    fixture("readme-involute-rack",curve::involute_rack(f(0.32),7,f(20.0),f(0.02),f(0.01)),out);
    fixture("readme-cycloidal-rack",curve::cycloidal_rack(f(0.32),7,f(0.02),12),out);
    fixture("readme-airfoil",curve::airfoil_naca4(f(2.0),f(4.0),f(12.0),f(3.0),96),out);
    fixture("bench-involute",curve::involute_gear(r(2),20,r(20),r(0),r(0),4),out);
    fixture("bench-cycloidal",curve::cycloidal_gear(r(2),12,r(2),r(0),4),out);
    fixture("bench-involute-rack",curve::involute_rack(r(2),4,r(20),r(0),r(0)),out);
    fixture("bench-cycloidal-rack",curve::cycloidal_rack(r(2),4,r(0),8),out);
    fixture("bench-airfoil",curve::airfoil_naca4(r(2),r(4),r(12),r(20),80),out);
    let rack=curve::involute_rack(r(1),2,r(30),r(0),r(0)).attempt().unwrap();
    let expected=q(15,4)*Real::pi()+q(29,24)*r(3).sqrt().unwrap();
    assert_eq!(rack.filled_area().unwrap().unwrap().certified_eq_until(&expected,-1024).as_bool(),Some(true));
    for (x,y,inside) in [(r(0),r(0),Some(true)),(Real::pi(),r(0),Some(true)),(Real::pi()/r(2)).map(|x|(x,r(0),Some(false))).unwrap(),(Real::pi()/r(2)).map(|x|(x,r(-2),Some(true))).unwrap()] {
        assert_eq!(curve::contains_xy(&rack,x,y).unwrap(),inside);
    }
    println!("independent rack area=15*pi/4+29*sqrt(3)/24 and four membership witnesses passed");
}
