use hypercurve::{BezierParameter2, BezierSplitFragment2, BezierSubcurve2, Classification,
    CurveBoundaryInteriorSide2, CurveContext, CurveRegion2, CurveRegionBoundaryLoop2,
    CurveRegionFragmentSource2, CurveRegionLoopRole, FillRule, Point2, QuadraticBezier2,
    Real, RegionPointLocation};
fn p(x:i64,y:i64)->Point2{Point2::new(Real::from(x),Real::from(y))}
fn rect(x0:i64,y0:i64,x1:i64,y1:i64,base:usize,policy:&CurveContext)->CurveRegionBoundaryLoop2 {
    let points=[p(x0,y0),p(x1,y0),p(x1,y1),p(x0,y1)];
    let fragments=(0..4).map(|i| {
        let a=points[i].clone(); let b=points[(i+1)%4].clone();
        let mid=Point2::new(((a.x()+b.x())/Real::from(2)).unwrap(),((a.y()+b.y())/Real::from(2)).unwrap());
        BezierSplitFragment2::Materialized{start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),curve:BezierSubcurve2::Quadratic(QuadraticBezier2::new(a,mid,b))}
    }).collect();
    CurveRegionBoundaryLoop2::try_new_with_arrangement_sources(fragments,(0..4).map(|i|CurveRegionFragmentSource2::new(base+i,base+i,0)).collect(),policy).unwrap()
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
            let loops=bounds.into_iter().enumerate().map(|(i,(a,b,c,d))|rect(a,b,c,d,i*4,&policy)).collect();
            let region=match roles {
                None=>CurveRegion2::new(loops),
                Some(roles)=>CurveRegion2::try_new_with_loop_topology(loops,roles,vec![FillRule::NonZero;2],vec![CurveBoundaryInteriorSide2::Left;2]),
            }.unwrap();
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
    println!("{{\"cases\":{cases},\"admission_failures\":{failures},\"normalized_membership_checks\":{replays}}}");
}
