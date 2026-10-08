use hypercurve::{Contour2,CurveContext,CurveRegion2,Real};
fn main(){
    let count:usize=std::env::args().nth(1).unwrap_or("1000000".into()).parse().unwrap();
    let points=(0..count).map(|i|{let theta=2.0*std::f64::consts::PI*(i as f64)/(count as f64);[1.0+theta.cos(),1.0+theta.sin()]}).collect::<Vec<_>>();
    eprintln!("projection built: {count} vertices");
    let mut doubled=Real::zero();
    for i in 0..count {let a=points[i];let b=points[(i+1)%count];doubled += Real::try_from(a[0]).unwrap()*Real::try_from(b[1]).unwrap()-Real::try_from(b[0]).unwrap()*Real::try_from(a[1]).unwrap();}
    eprintln!("area accumulated");
    assert_eq!(doubled.partial_cmp(&Real::zero()),Some(std::cmp::Ordering::Greater));
    eprintln!("orientation certified");
    let contour=Contour2::from_finite_ring(&points).unwrap();
    eprintln!("contour constructed");
    let region=CurveRegion2::try_from_native_contours(vec![contour],vec![],&CurveContext::STRICT).unwrap();
    eprintln!("region constructed: loops={} certainty={:?}",region.value.len(),region.certainty);
}
