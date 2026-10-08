use hypercurve::{Classification, Curve2, CurveContext, CurvePath2, CurveRegion2,
    CurveRegionLoopRole, FillRule, LineSeg2, Point2, QuadraticBezier2, Real, RegionPointLocation};
fn point(x:i64,y:i64)->Point2 {Point2::new(Real::from(x),Real::from(y))}
fn square(a:i64,b:i64)->CurvePath2 {
    let points=[point(a,a),point(b,a),point(b,b),point(a,b)];
    CurvePath2::try_new((0..4).map(|i|Curve2::from(LineSeg2::try_new(points[i].clone(),points[(i+1)%4].clone()).unwrap())).collect()).unwrap()
}
fn main(){
    let mut cases=0;let mut construction_failures=0;let mut normalized_replays=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let q=Curve2::from(QuadraticBezier2::new(point(-2,4),point(0,-4),point(2,4)));
        let l=Curve2::from(LineSeg2::try_new(point(2,4),point(-2,4)).unwrap());
        let twice=CurvePath2::try_new(vec![q.clone(),l.clone(),q,l]).unwrap();
        for (name,paths,roles,fill,samples,loops) in [
            ("cancellation",vec![square(0,4),square(0,4)],vec![CurveRegionLoopRole::Material,CurveRegionLoopRole::Hole],FillRule::NonZero,vec![(point(0,2),RegionPointLocation::Outside),(point(2,2),RegionPointLocation::Outside)],0),
            ("nested-material-seam",vec![square(0,8),square(2,6)],vec![CurveRegionLoopRole::Material;2],FillRule::NonZero,vec![(point(2,4),RegionPointLocation::Inside),(point(4,4),RegionPointLocation::Inside)],1),
            ("recursive-islands",vec![square(0,12),square(2,10),square(4,8)],vec![CurveRegionLoopRole::Material,CurveRegionLoopRole::Hole,CurveRegionLoopRole::Material],FillRule::NonZero,vec![(point(1,6),RegionPointLocation::Inside),(point(3,6),RegionPointLocation::Outside),(point(5,6),RegionPointLocation::Inside)],3),
            ("even-odd-double-traversal",vec![twice],vec![CurveRegionLoopRole::Material],FillRule::EvenOdd,vec![(point(0,4),RegionPointLocation::Outside),(point(0,2),RegionPointLocation::Outside)],0),
        ] {
            {
                let rules=vec![fill;paths.len()];
                let region=CurveRegion2::try_from_boundary_paths_with_loop_semantics(&paths,&roles,&rules,&policy).unwrap().into_value();
                let normalized=region.regularized_region(&policy).unwrap().into_value();
                assert_eq!(normalized.len(),loops,"{name}: normalized loops");
                let mut valid=region.len()==loops;
                for (p,expected) in &samples {
                    let actual=region.classify_point(p,&policy).unwrap().into_value();
                    let replay=normalized.classify_point(p,&policy).unwrap().into_value();
                    assert_eq!(replay,Classification::Decided(*expected),"{name}: normalized membership");
                    valid &= actual==Classification::Decided(*expected);normalized_replays+=1;
                }
                println!("case={name} policy={policy:?} loops={} expected={loops} constructor_normalized={valid}",region.len());
                cases+=1;construction_failures+=usize::from(!valid);
            }
        }
    }
    assert_eq!(construction_failures, 0);
    println!("{{\"cases\":{cases},\"construction_failures\":{construction_failures},\"normalized_point_replays\":{normalized_replays}}}");
}
