use hypercurve::{BulgeVertex2, Classification, Contour2, CurveContext, CurveRegion2, FiniteProjectionOptions, OffsetCornerStyle2, Point2, PolylineReconstructionOptions, Real, RegionPointLocation};
fn rectangle(x0: i32, x1: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[(x0,0),(x1,0),(x1,4),(x0,4)].map(|(x,y)| BulgeVertex2::new(Point2::from_values(x,y), Real::zero()))).unwrap()
}
fn main() {
    let mut empty_offset_failures=0;
    for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512].iter().enumerate() {
        for (style_index, style) in [OffsetCornerStyle2::Round,OffsetCornerStyle2::Bevel,OffsetCornerStyle2::Miter{limit:Real::from(4)}].iter().enumerate() {
            for distance in [-1,0,1] {
                match CurveRegion2::empty().offset(Real::from(distance),style,policy) {
                    Ok(result) => assert!(result.value.is_empty()),
                    Err(_) => {empty_offset_failures+=1;println!("empty-offset policy={policy_index} style={style_index} distance={distance} rejected=true");}
                }
            }
        }
        let options=FiniteProjectionOptions::try_new(0.01).unwrap();
        let mut profiles=Vec::new();
        for (x0,x1) in [(0,4),(2,6)] {
            let region=CurveRegion2::try_from_native_material_contours(vec![rectangle(x0,x1)],policy).unwrap().into_value();
            let Classification::Decided(projected)=region.project_to_finite_profiles(&options,policy).unwrap().into_value()else{panic!("rectangle projection must be decided")};
            profiles.extend(projected);
        }
        let recovered=CurveRegion2::recover_from_finite_profiles(&profiles,PolylineReconstructionOptions{min_arc_points:8,..PolylineReconstructionOptions::DEFAULT},policy).unwrap();
        let query=Point2::from_values(2,2).into();
        let seam_boundary=matches!(recovered.classify_point(&query,policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Boundary));
        let normalized=recovered.regularized_region(policy).unwrap().into_value();
        let seam_inside=matches!(normalized.classify_point(&query,policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Inside));
        println!("recovery policy={policy_index} loops={} seam_boundary={seam_boundary} normalized_loops={} normalized_seam_inside={seam_inside}",recovered.len(),normalized.len());
        assert_eq!(recovered.len(),2);assert!(seam_boundary);assert_eq!(normalized.len(),1);assert!(seam_inside);
    }
    println!("empty_offset_failures={empty_offset_failures}");
    assert_eq!(empty_offset_failures,6);
}
