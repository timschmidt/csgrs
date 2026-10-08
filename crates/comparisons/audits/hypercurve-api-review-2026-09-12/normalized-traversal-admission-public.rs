use hypercurve::{BezierArrangementFragment2, BezierArrangementGraph2, BezierParameter2,
    BezierSplitFragment2, BezierSubcurve2, Classification, CurveContext, CurveRegion2,
    Point2, QuadraticBezier2, Real, RegionPointLocation};
fn p(x:i64,y:i64)->Point2 { Point2::new(Real::from(x),Real::from(y)) }
fn main(){
    let mut failures=0;let mut cases=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for (name,loops,expected_loops,samples) in [
            ("overlap-canceled-seam",vec![vec![p(0,0),p(4,0),p(4,4),p(0,4)],vec![p(2,0),p(6,0),p(6,4),p(2,4)]],2,vec![(p(3,0),RegionPointLocation::Outside),(p(3,2),RegionPointLocation::Outside),(p(1,2),RegionPointLocation::Inside),(p(5,2),RegionPointLocation::Inside)]),
            ("self-crossing",vec![vec![p(0,0),p(4,4),p(0,4),p(4,0)]],2,vec![(p(2,1),RegionPointLocation::Inside),(p(2,3),RegionPointLocation::Inside),(p(0,2),RegionPointLocation::Outside),(p(2,2),RegionPointLocation::Boundary)]),
        ] {
            let fragments=loops.into_iter().flat_map(|points|(0..points.len()).map(move|i| {
                let a=points[i].clone();let b=points[(i+1)%points.len()].clone();
                let mid=Point2::new(((a.x()+b.x())/Real::from(2)).unwrap(),((a.y()+b.y())/Real::from(2)).unwrap());
                BezierSplitFragment2::Materialized{start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),curve:BezierSubcurve2::Quadratic(QuadraticBezier2::new(a,mid,b))}
            })).enumerate().map(|(i,f)|BezierArrangementFragment2::new(i,0,f)).collect();
            let graph=BezierArrangementGraph2::new(fragments).unwrap();
            let Classification::Decided(traversal)=graph.traverse_branch_free(&policy) else {panic!("closed graph")};
            let Classification::Decided(region)=CurveRegion2::from_retained_arrangement_traversal(&graph,&traversal,&policy).into_value() else {panic!("region")};
            let normalized=region.regularized_region(&policy).unwrap().into_value();
            assert_eq!(normalized.len(),expected_loops);
            let mut valid=region.len()==expected_loops;
            for (point,expected) in samples {
                let actual=region.classify_point(&point,&policy).unwrap().into_value();
                assert_eq!(normalized.classify_point(&point,&policy).unwrap().into_value(),Classification::Decided(expected));
                valid &=actual==Classification::Decided(expected);
            }
            cases+=1;failures+=usize::from(!valid);
            println!("{name} {policy:?}: normalized_at_admission={valid}");
        }
    }
    println!("{{\"cases\":{cases},\"admission_failures\":{failures}}}");
}
