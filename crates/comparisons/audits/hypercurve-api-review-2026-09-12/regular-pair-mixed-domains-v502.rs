#[cfg(test)]
mod regular_pair_mixed_domain_probe {
    use super::*;
    fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
    fn p(x:i64,y:i64)->Point2{Point2::from_values(x,y)}
    fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(value)=>value,Classification::Uncertain(reason)=>panic!("mixed retained pair blocked: {reason:?}")}}
    #[test]
    fn mixed_regular_pairs_keep_both_exterior_parameter_domains(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            for stationary_chart in [false,true]{
                let points=if stationary_chart{vec![p(0,0),p(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),p(1,1)]}else{vec![p(0,0),Point2::new(q(1,2),Real::zero()),p(1,1)]};
                let weights=vec![Real::one();points.len()];
                let first=RationalBezier2::try_new(points,weights).unwrap().parallel_left(Real::from(65).sqrt().unwrap()).unwrap();
                let first_range=if stationary_chart{CurveParameterRange2::new_validated(q(3,2).into(),q(5,2).into())}else{CurveParameterRange2::new_validated(Real::from(3).into(),Real::from(5).into())};
                let first_value=Real::from(if stationary_chart{2}else{4});
                let second_range=CurveParameterRange2::new_validated(Real::from(16).into(),Real::from(18).into());
                let center:CurvePoint2=p(-4,17).into();
                for distance in [0_i64,1]{
                    let x=Real::from(-4+distance);
                    let second=QuadraticBezier2::new(Point2::new(x.clone(),Real::zero()),Point2::new(x.clone(),q(1,2)),Point2::new(x,Real::one())).parallel_left(Real::from(distance)).unwrap();
                    for (parallel,range,value)in [(&first,&first_range,first_value.clone()),(&second,&second_range,Real::from(17))]{
                        let point=decided(parallel.point_evidence_on_regular_range(&value.into(),range,&policy).unwrap());
                        assert_eq!(point.same_point(&center,&policy),Classification::Decided(true));
                    }
                    for swapped in [false,true]{
                        let (a,b,ar,br,av,bv)=if swapped{(&second,&first,&second_range,&first_range,Real::from(17),first_value.clone())}else{(&first,&second,&first_range,&second_range,first_value.clone(),Real::from(17))};
                        let result=decided(a.parallel_intersections_on_regular_ranges(b,ar,br,&policy).unwrap());
                        eprintln!("stationary={stationary_chart} distance={distance} swapped={swapped} complete={} contacts={}",result.is_complete(),result.contacts().len());
                        assert!(result.is_complete());assert_eq!(result.contacts().len(),1);
                        let contact=&result.contacts()[0];
                        for (parameter,value)in [(contact.first_parameter(),av),(contact.second_parameter(),bv)]{
                            assert_eq!(parameter.polynomial_sign(&[-value,Real::one()],&policy).unwrap(),Classification::Decided(RealSign::Zero));
                        }
                        assert_eq!(contact.tangent_cross_sign(),Some(if swapped{RealSign::Negative}else{RealSign::Positive}));
                        assert_eq!(contact.tangent_dot_sign(),Some(RealSign::Positive));
                    }
                }
            }
        }
    }
    #[test]
    fn rational_regular_pairs_use_the_requested_normal_sheet() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            // x=(u-2)^2 has a downward left normal on its authored unit span,
            // but an upward one on [3,4]. Its retained offset meets x=2 at y=1.
            let first = QuadraticBezier2::new(p(4,0), p(2,0), p(1,0))
                .parallel_left(Real::one()).unwrap();
            let second = QuadraticBezier2::new(p(2,0), p(2,1), p(2,2))
                .parallel_left(Real::zero()).unwrap();
            let first_range = CurveParameterRange2::new_validated(Real::from(3).into(), Real::from(4).into());
            let second_range = CurveParameterRange2::unit();
            let first_value = Real::from(2) + Real::from(2).sqrt().unwrap();
            let second_value = q(1,2);
            let center: CurvePoint2 = p(2,1).into();
            for (parallel, range, value) in [(&first, &first_range, first_value.clone()), (&second, &second_range, second_value.clone())] {
                let point = decided(parallel.point_evidence_on_regular_range(&value.into(), range, &policy).unwrap());
                assert_eq!(point.same_point(&center, &policy), Classification::Decided(true));
            }
            for swapped in [false, true] {
                let (a,b,ar,br,av,bv) = if swapped {(&second,&first,&second_range,&first_range,second_value.clone(),first_value.clone())} else {(&first,&second,&first_range,&second_range,first_value.clone(),second_value.clone())};
                let result = decided(a.parallel_intersections_on_regular_ranges(b,ar,br,&policy).unwrap());
                eprintln!("rational normal sheet swapped={swapped} complete={} contacts={}",result.is_complete(),result.contacts().len());
                assert!(result.is_complete()); assert_eq!(result.contacts().len(),1);
                let contact=&result.contacts()[0];
                for (parameter,value) in [(contact.first_parameter(),av),(contact.second_parameter(),bv)] {
                    assert_eq!(parameter.polynomial_sign(&[-value,Real::one()],&policy).unwrap(),Classification::Decided(RealSign::Zero));
                }
                assert_eq!(contact.tangent_cross_sign(),Some(if swapped {RealSign::Negative} else {RealSign::Positive}));
                assert_eq!(contact.tangent_dot_sign(),Some(RealSign::Zero));
            }
        }
    }
}
