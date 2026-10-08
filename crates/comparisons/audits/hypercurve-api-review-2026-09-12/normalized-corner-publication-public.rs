use hypercurve::*;
fn p(x:i64,y:i64)->Point2{Point2::from_values(x,y)}
fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
fn square(a:i64,b:i64)->CurvePath2 {
    let points=[p(a,a),p(b,a),p(b,b),p(a,b)];
    CurvePath2::try_new((0..4).map(|i|LineSeg2::try_new(points[i].clone(),points[(i+1)%4].clone()).unwrap().into()).collect()).unwrap()
}
fn main(){
    let require_normalized=std::env::args().any(|arg|arg=="expect-normalized");
    let mut failures=0;
    let mut cases=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for fillet in [false,true] {
            let source=CurveRegion2::try_from_boundary_paths(&[square(0,8),square(1,3)],&policy).unwrap().into_value();
            let Classification::Decided(paths)=source.boundary_paths(&policy).unwrap().value else{panic!("exact source paths")};
            let corner=CurvePoint2::from(p(0,0));
            let (loop_index,vertex)=paths.iter().enumerate().find_map(|(i,path)|path.curves().iter().position(|c|c.start().coincides_with(&corner,&policy).value==Classification::Decided(true)).map(|j|(i,j))).unwrap();
            let outcome=if fillet {
                source.fillet_loop_vertex_by_radius(loop_index,vertex,Real::from(5),CurveCornerMode2::TrimOnly,&policy)
            } else {
                source.chamfer_loop_vertex_by_setbacks(loop_index,vertex,Real::from(5),Real::from(5),CurveCornerMode2::TrimOnly,&policy)
            }.unwrap();
            assert_eq!(outcome.certainty,CurveCertainty::Certified);
            let CurveCornerSolutions2::Unique(region)=outcome.into_value() else{panic!("unique exact corner cut")};
            let normalized=region.regularized_region(&policy).unwrap().into_value();
            assert_eq!(normalized.len(),1,"the cut opens the hole");
            let mut samples=vec![(p(6,6),RegionPointLocation::Inside),(p(0,6),RegionPointLocation::Boundary)];
            if fillet {
                let coordinate=Real::from(5)-Real::from(5)*q(1,2).sqrt().unwrap();
                samples.extend([(p(1,1),RegionPointLocation::Outside),(Point2::new(coordinate.clone(),coordinate),RegionPointLocation::Outside),(p(1,2),RegionPointLocation::Boundary),(p(2,1),RegionPointLocation::Boundary)]);
            } else {
                samples.extend([(p(1,2),RegionPointLocation::Outside),(p(2,1),RegionPointLocation::Outside),(Point2::new(q(5,2),q(5,2)),RegionPointLocation::Outside),(p(2,3),RegionPointLocation::Boundary),(p(3,2),RegionPointLocation::Boundary)]);
                assert_eq!(normalized.filled_area(&policy).unwrap().value,Classification::Decided(Some(Real::from(51))));
            }
            let mut valid=region.len()==1;
            let mut mismatches=0;
            for (point,expected) in samples {
                assert_eq!(normalized.classify_point(&point,&policy).unwrap().value,Classification::Decided(expected),"independent normalized membership oracle");
                if region.classify_point(&point,&policy).unwrap().value!=Classification::Decided(expected) {valid=false;mismatches+=1;}
            }
            cases+=1;failures+=usize::from(!valid);
            println!("fillet={fillet} policy={policy:?} normalized_at_publication={valid} raw_loops={} normalized_loops={} membership_mismatches={mismatches}",region.len(),normalized.len());
        }
    }
    assert_eq!(cases,4);
    println!("{{\"cases\":{cases},\"publication_failures\":{failures}}}");
    if require_normalized{assert_eq!(failures,0);}
}
