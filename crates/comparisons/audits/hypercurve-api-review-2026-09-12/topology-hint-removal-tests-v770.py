from pathlib import Path
A=Path(__file__).resolve().parent;C=A/'topology-hint-removal-candidate-v770'
p=C/'hypercurve/tests/hypercurve_curve_region_promotion.rs';s=p.read_text();s+=r'''

#[test]
fn authored_region_sides_are_certified_before_offset_and_boolean_reentry() {
    use RegionPointLocation::{Boundary, Inside, Outside};
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for rational in [false, true] {
            let vertices=[p(0,0),p(4,0),p(4,4),p(0,4),p(0,0)];
            let path=CurvePath2::try_new(vertices.windows(2).map(|edge| {
                if rational {
                    let (dx,dy)=edge[0].delta_from(&edge[1]);
                    let middle=Point2::new(edge[0].x()-dx*q(1,2),edge[0].y()-dy*q(1,2));
                    RationalBezier2::try_new(vec![edge[0].clone(),middle,edge[1].clone()],vec![Real::one(),Real::from(2),Real::from(3)]).unwrap().into()
                } else {
                    LineSeg2::try_new(edge[0].clone(),edge[1].clone()).unwrap().into()
                }
            }).collect()).unwrap();
            for reverse in [false,true] {
                let path=if reverse {certified(path.reversed(&policy).unwrap())} else {path.clone()};
                for fill_rule in [FillRule::NonZero,FillRule::EvenOdd] {
                    let region=certified(CurveRegion2::try_from_boundary_paths_with_loop_semantics(std::slice::from_ref(&path),&[CurveRegionLoopRole::Material],&[fill_rule],&policy).unwrap());
                    assert_eq!(region.len(),1);
                    assert_eq!(decided(region.loop_roles(&policy).unwrap()),vec![CurveRegionLoopRole::Material]);
                    assert_eq!(decided(region.filled_side_is_left(&policy).unwrap()),&[true]);
                    let area=decided(region.signed_area(&policy).unwrap()).unwrap();
                    assert_eq!(area.partial_cmp(&Real::from(16)),Some(std::cmp::Ordering::Equal));
                    for (point,expected) in [(p(2,2),Inside),(p(0,2),Boundary),(p(-1,2),Outside)] {
                        assert_eq!(decided(region.classify_point(&point.into(),&policy).unwrap()),expected);
                    }
                    let grown=certified(region.offset(Real::one(),&OffsetCornerStyle2::Round,&policy).unwrap());
                    for (point,expected) in [(Point2::new(-q(1,2),Real::from(2)),Inside),(p(-1,2),Boundary),(p(-2,2),Outside)] {
                        assert_eq!(decided(grown.classify_point(&point.into(),&policy).unwrap()),expected);
                    }
                    let recovered=certified(grown.boolean_region(&region,hypercurve::BooleanOp::Intersection,&policy).unwrap());
                    for (point,expected) in [(p(2,2),Inside),(p(0,2),Boundary),(p(-1,2),Outside)] {
                        assert_eq!(decided(recovered.classify_point(&point.into(),&policy).unwrap()),expected);
                    }
                }
            }
        }
    }
}
''';p.write_text(s)
print('Added authored orientation regression with16policy/family/orientation/fill combinations')
