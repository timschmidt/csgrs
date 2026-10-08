use hypercurve::*;
fn p(x:i32,y:i32)->Point2 { Point2::from_values(x,y) }
fn region(hole:bool)->CurveRegion2 {
    fn rectangle(lo:i32,hi:i32)->Contour2 {
        let ps=[p(lo,lo),p(hi,lo),p(hi,hi),p(lo,hi)];
        Contour2::try_new((0..4).map(|i| Segment2::Line(LineSeg2::try_new(ps[i].clone(),ps[(i+1)%4].clone()).unwrap())).collect()).unwrap()
    }
    CurveRegion2::try_from_native_contours(vec![rectangle(-5,5)], if hole {vec![rectangle(-2,2)]} else {vec![]}, &CurveContext::STRICT).unwrap().value
}
fn constants(point:Point2,policy:&CurveContext)->Vec<Curve2> {
    let knots=[2,2,3,4,5,5].into_iter().map(Real::from).collect::<Vec<_>>();
    let Classification::Decided(h)=RationalBezier2::from_homogeneous_controls(vec![
        HomogeneousControl2::new(point.x().clone(),point.y().clone(),Real::one()),
        HomogeneousControl2::new(Real::zero(),Real::zero(),Real::zero()),
        HomogeneousControl2::new(point.x().clone(),point.y().clone(),Real::one()),
    ],policy).unwrap() else {panic!("constant homogeneous fixture")};
    vec![
        QuadraticBezier2::new(point.clone(),point.clone(),point.clone()).into(),
        CubicBezier2::new(point.clone(),point.clone(),point.clone(),point.clone()).into(),
        RationalQuadraticBezier2::try_new(point.clone(),point.clone(),point.clone(),Real::one(),Real::from(2),Real::from(3)).unwrap().into(),
        RationalBezier2::try_new(vec![point.clone();4],[1,2,3,1].into_iter().map(Real::from).collect()).unwrap().into(),
        h.into(),
        Curve2::try_polynomial_bspline(1,vec![point.clone();4],knots.clone(),policy).unwrap().value,
        Curve2::try_nurbs(1,vec![point;4],[1,2,3,1].into_iter().map(Real::from).collect(),knots,policy).unwrap().value,
    ]
}
fn assert_point(actual:CurvePoint2, expected:Point2, policy:&CurveContext) {
    let equal=actual.coincides_with(&expected.into(),policy);
    assert_eq!(equal.certainty,CurveCertainty::Certified);
    assert_eq!(equal.value,Classification::Decided(true));
}
fn main() {
    let mut empty_clips=0;
    let mut path_clips=0;
    let mut replayed_paths=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for hole in [false,true] {
            let region=region(hole);
            for point in [p(0,0),p(2,0),p(5,0),p(6,0),Point2::new(Real::from(2).sqrt().unwrap(),-Real::from(3).sqrt().unwrap())] {
                for source in constants(point,&policy) {
                    for reverse in [false,true] {
                        let curve=if reverse {source.reversed(&policy).unwrap().value} else {source.clone()};
                        let result=curve.trim_inside_region(&region,&policy).unwrap();
                        assert_eq!(result.certainty,CurveCertainty::Certified);
                        assert!(result.value.is_empty());
                        empty_clips+=1;
                    }
                }
            }
            for constant in constants(p(0,0),&policy) {
                for reverse in [false,true] {
                    let mut curves=vec![
                        LineSeg2::try_new(p(-6,0),p(0,0)).unwrap().into(),
                        constant.clone(),
                        LineSeg2::try_new(p(0,0),p(6,0)).unwrap().into(),
                    ];
                    if reverse {
                        curves=curves.into_iter().rev().map(|c:Curve2|c.reversed(&policy).unwrap().value).collect();
                    }
                    let mut paths=CurvePath2::try_new(curves).unwrap().trim_inside_region(&region,&policy).unwrap();
                    assert_eq!(paths.certainty,CurveCertainty::Certified);
                    assert_eq!(paths.value.len(),if hole {2} else {1});
                    let mut expected=if hole {vec![(-5,-2),(2,5)]} else {vec![(-5,5)]};
                    if reverse {expected=expected.into_iter().rev().map(|(a,b)|(b,a)).collect();}
                    for generation in 0..4 {
                        for (path,(a,b)) in paths.value.iter().zip(&expected) {
                            let first=path.fragments().first().unwrap();
                            let last=path.fragments().last().unwrap();
                            assert_eq!(path.fragments().len(),if hole {1} else {2});
                            assert_point(first.trim_fragment().curve().start(),p(*a,0),&policy);
                            assert_point(last.trim_fragment().curve().end(),p(*b,0),&policy);
                            if generation==0 {
                                for f in path.fragments() { assert_ne!(f.source_curve_index(),1); }
                            }
                        }
                        if generation==3 {break;}
                        let mut next=vec![];
                        for path in paths.value {
                            let curves=path.into_fragments().into_iter().map(|f|f.into_trim_fragment().into_curve()).collect();
                            let result=CurvePath2::try_new(curves).unwrap().trim_inside_region(&region,&policy).unwrap();
                            assert_eq!(result.certainty,CurveCertainty::Certified);
                            assert_eq!(result.value.len(),1);
                            next.extend(result.value);
                            replayed_paths+=1;
                        }
                        paths.value=next;
                    }
                    path_clips+=1;
                }
            }
        }
    }

    let mut clipped_spline_paths=0;
    let mut spline_endpoint_replays=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for hole in [false,true] {
            let region=region(hole);
            let controls=vec![p(-6,0),p(-6,0),p(0,0),p(0,0),p(6,0),p(6,0)];
            let knots=[2,2,3,4,5,6,7,7].into_iter().map(Real::from).collect::<Vec<_>>();
            for source in [
                Curve2::try_polynomial_bspline(1,controls.clone(),knots.clone(),&policy).unwrap().value,
                Curve2::try_nurbs(1,controls, [1,2,3,4,5,6].into_iter().map(Real::from).collect(),knots,&policy).unwrap().value,
            ] {
                let restricted=source.subcurve((Real::from(5)/Real::from(2)).unwrap().into(),(Real::from(13)/Real::from(2)).unwrap().into(),&policy).unwrap().value;
                for (curve,reversed) in [(source.clone(),false),(source.reversed(&policy).unwrap().value,true),(restricted.clone(),false),(restricted.reversed(&policy).unwrap().value,true)] {
                    let outcome=CurvePath2::try_new(vec![curve.clone()]).unwrap().trim_inside_region(&region,&policy).unwrap();
                    assert_eq!(outcome.certainty,CurveCertainty::Certified);
                    let mut expected=if hole {vec![(-5,-2),(2,5)]} else {vec![(-5,5)]};
                    if reversed {expected=expected.into_iter().rev().map(|(a,b)|(b,a)).collect();}
                    assert_eq!(outcome.value.len(),expected.len());
                    let mut indices=vec![];
                    for (path,(a,b)) in outcome.value.iter().zip(expected) {
                        assert_point(path.fragments()[0].trim_fragment().curve().start(),p(a,0),&policy);
                        assert_point(path.fragments().last().unwrap().trim_fragment().curve().end(),p(b,0),&policy);
                        assert!(!path.fragments()[0].trim_fragment().start_boundary_contacts().is_empty());
                        assert!(!path.fragments().last().unwrap().trim_fragment().end_boundary_contacts().is_empty());
                        for f in path.fragments() {
                            let f=f.trim_fragment();
                            indices.push(f.span_index());
                            let Classification::Decided(range)=f.parameter_range(&policy).unwrap() else {panic!("source chart replay")};
                            for (t,point) in [(range.start(),f.curve().start()),(range.end(),f.curve().end())] {
                                let evaluated=curve.point_at(t,&policy).unwrap();
                                assert_eq!(evaluated.certainty,CurveCertainty::Certified);
                                let equal=evaluated.value.coincides_with(&point,&policy);
                                assert_eq!(equal.certainty,CurveCertainty::Certified);
                                assert_eq!(equal.value,Classification::Decided(true));
                                spline_endpoint_replays+=1;
                            }
                        }
                    }
                    assert_eq!(indices,vec![1,3]);
                    clipped_spline_paths+=1;
                }
            }
        }
    }
    println!("{{\"empty_clips\":{empty_clips},\"path_clips\":{path_clips},\"replayed_paths\":{replayed_paths},\"clipped_spline_paths\":{clipped_spline_paths},\"spline_endpoint_replays\":{spline_endpoint_replays}}}");
}
