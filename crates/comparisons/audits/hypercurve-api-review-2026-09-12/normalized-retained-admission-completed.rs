use hypercurve::{Classification, Curve2, CurveBoundaryInteriorSide2, CurveContext,
    CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, Point2, QuadraticBezier2,
    Real, RegionPointLocation};
fn p(x:i64,y:i64)->Point2{Point2::new(Real::from(x),Real::from(y))}
fn rect(x0:i64,y0:i64,x1:i64,y1:i64)->CurvePath2 {
    let points=[p(x0,y0),p(x1,y0),p(x1,y1),p(x0,y1)];
    CurvePath2::try_new((0..4).map(|i| {
        let a=points[i].clone(); let b=points[(i+1)%4].clone();
        let mid=Point2::new(((a.x()+b.x())/Real::from(2)).unwrap(),((a.y()+b.y())/Real::from(2)).unwrap());
        Curve2::from(QuadraticBezier2::new(a,mid,b))
    }).collect()).unwrap()
}
fn main() {
    let mut cases=0; let mut failures=0; let mut replays=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for (name,bounds,roles,expected_loops,samples) in [
            ("even-odd-coincident",vec![(0,0,4,4),(0,0,4,4)],None,0,vec![(p(0,2),RegionPointLocation::Outside),(p(2,2),RegionPointLocation::Outside)]),
            ("material-hole-cancellation",vec![(0,0,4,4),(0,0,4,4)],Some(vec![CurveRegionLoopRole::Material,CurveRegionLoopRole::Hole]),0,vec![(p(0,2),RegionPointLocation::Outside),(p(2,2),RegionPointLocation::Outside)]),
            ("nested-material-seam",vec![(0,0,8,8),(2,2,6,6)],Some(vec![CurveRegionLoopRole::Material;2]),1,vec![(p(2,4),RegionPointLocation::Inside),(p(4,4),RegionPointLocation::Inside)]),
            ("adjacent-material-seam",vec![(0,0,4,4),(4,0,8,4)],Some(vec![CurveRegionLoopRole::Material;2]),1,vec![(p(4,2),RegionPointLocation::Inside),(p(6,2),RegionPointLocation::Inside)]),
        ] {
            let paths=bounds.into_iter().map(|(a,b,c,d)|rect(a,b,c,d)).collect::<Vec<_>>();
            let region=match roles {
                None=>CurveRegion2::try_from_boundary_paths(&paths,&policy),
                Some(roles)=>{
                    let sides=roles.iter().map(|role|if *role==CurveRegionLoopRole::Material {CurveBoundaryInteriorSide2::Left} else {CurveBoundaryInteriorSide2::Right}).collect::<Vec<_>>();
                    CurveRegion2::try_from_boundary_paths_with_loop_topology(&paths,&roles,&[FillRule::NonZero;2],&sides,&policy)
                },
            }.unwrap().into_value();
            let normalized=region.regularized_region(&policy).unwrap().into_value();
            assert_eq!(normalized.len(),expected_loops,"{name}: normalized loop count");
            let mut valid=region.len()==expected_loops;
            for (point,expected) in samples {
                let actual=region.classify_point(&point,&policy).unwrap().into_value();
                assert_eq!(normalized.classify_point(&point,&policy).unwrap().into_value(),Classification::Decided(expected),"{name}: replay membership");
                valid &= actual==Classification::Decided(expected); replays+=1;
            }
            cases+=1; failures+=usize::from(!valid);
            println!("case={name} policy={policy:?} normalized_at_admission={valid} loops={} expected={expected_loops}",region.len());
        }
    }
    assert_eq!(cases,8); assert_eq!(failures,0); assert_eq!(replays,16);
    println!("{{\"cases\":{cases},\"admission_failures\":{failures},\"normalized_membership_checks\":{replays}}}");
}
