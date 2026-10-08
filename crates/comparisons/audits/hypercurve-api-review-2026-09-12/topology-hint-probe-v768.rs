use hypercurve::{Classification, Curve2, CurveBoundaryInteriorSide2, CurveContext, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, OffsetCornerStyle2, Point2, Real, RegionPointLocation};
fn main() {
    let mut correct_controls=0;
    let mut inconsistent_accepted=0;
    let mut rejected_hints=0;
    for (policy_index,policy) in [CurveContext::STRICT,CurveContext::APPROXIMATE_512].iter().enumerate() {
        let corners=[(0,0),(4,0),(4,4),(0,4),(0,0)];
        let path=CurvePath2::try_new(corners.windows(2).map(|edge|Curve2::from(LineSeg2::try_new(Point2::from_values(edge[0].0,edge[0].1),Point2::from_values(edge[1].0,edge[1].1)).unwrap())).collect()).unwrap();
        for reverse in [false,true] {
            let path=if reverse{path.reversed(policy).unwrap().into_value()}else{path.clone()};
            for (side_index,side) in [CurveBoundaryInteriorSide2::Left,CurveBoundaryInteriorSide2::Right].iter().enumerate() {
                let correct_hint=(side_index==0)!=reverse;
                match CurveRegion2::try_from_boundary_paths_with_loop_topology(std::slice::from_ref(&path),&[CurveRegionLoopRole::Material],&[FillRule::NonZero],&[*side],policy) {
                    Err(_) => {assert!(!correct_hint);rejected_hints+=1;println!("policy={policy_index} reverse={reverse} side={side_index} correct_hint={correct_hint} rejected=true");}
                    Ok(result) => {
                        let region=result.into_value();
                        let center_inside=matches!(region.classify_point(&Point2::from_values(2,2).into(),policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Inside));
                        let positive_area=matches!(region.signed_area(policy).unwrap().into_value(),Classification::Decided(Some(area)) if area.partial_cmp(&Real::zero())==Some(std::cmp::Ordering::Greater));
                        let grown=region.offset(Real::one(),&OffsetCornerStyle2::Round,policy);
                        let offset_correct=match grown {
                            Ok(result) => {let grown=result.into_value();
                                matches!(grown.classify_point(&Point2::new((Real::from(-1)/Real::from(2)).unwrap(),Real::from(2)).into(),policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Inside))
                                &&matches!(grown.classify_point(&Point2::from_values(-1,2).into(),policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Boundary))
                                &&matches!(grown.classify_point(&Point2::from_values(-2,2).into(),policy).unwrap().into_value(),Classification::Decided(RegionPointLocation::Outside))
                            },
                            Err(_)=>false,
                        };
                        if correct_hint {assert!(center_inside&&positive_area&&offset_correct);correct_controls+=1;}
                        else if !center_inside||!positive_area||!offset_correct{inconsistent_accepted+=1;}
                        println!("policy={policy_index} reverse={reverse} side={side_index} correct_hint={correct_hint} loops={} center_inside={center_inside} positive_area={positive_area} offset_correct={offset_correct}",region.len());
                    }
                }
            }
        }
    }
    println!("correct_controls={correct_controls} inconsistent_accepted={inconsistent_accepted} rejected_hints={rejected_hints}");
    assert_eq!(correct_controls,4);
}
