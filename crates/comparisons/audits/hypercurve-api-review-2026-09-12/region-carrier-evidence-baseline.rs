use hypercurve::*;
fn square(x: i64) -> CurveRegion2 {
    let p = [Point2::from_values(x,0),Point2::from_values(x+4,0),Point2::from_values(x+4,4),Point2::from_values(x,4)];
    let path=CurvePath2::try_new((0..4).map(|i| Curve2::from(LineSeg2::try_new(p[i].clone(),p[(i+1)%4].clone()).unwrap())).collect()).unwrap();
    CurveRegion2::try_from_boundary_paths(&[path],&CurveContext::STRICT).unwrap().into_value()
}
fn main() {
    let first=square(0); let second=square(2); let policy=CurveContext::STRICT;
    let report=first.intersect_region(&second,&policy).unwrap();
    assert!(report.value.is_complete());
    let Classification::Decided(paths)=first.boundary_paths(&policy).unwrap().value else {panic!("paths")};
    let mut invalid=0;
    for contact in report.value.contacts() {
        let source=contact.first();
        let curve=&paths[source.loop_index()].curves()[source.fragment_index()];
        let replay=curve.point_at(contact.first_parameter(),&policy);
        println!("authored_family={:?} retained_parameter={:?} replay={:?}",curve.family(),contact.first_parameter(),replay);
        if replay.is_err() {invalid+=1;}
    }
    assert!(invalid>0);
    println!("{{\"unreplayable_authored_chart_contacts\":{invalid}}}");
}
