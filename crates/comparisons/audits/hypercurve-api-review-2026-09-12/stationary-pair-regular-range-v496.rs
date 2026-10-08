#[cfg(test)]
mod stationary_pair_regular_range_probe {
    use super::*;
    fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
    fn p(x:i64,y:i64)->Point2{Point2::from_values(x,y)}
    fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(value)=>value,Classification::Uncertain(reason)=>panic!("regular-cell query blocked: {reason:?}")}}
    #[test]
    fn stationary_endpoint_line_chart_reuses_regular_pair_incidence(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            let radius=q(15,16);
            let first=QuadraticBezier2::new(p(0,-2),p(0,-1),p(0,0)).parallel_left(-radius.clone()).unwrap();
            let second=RationalBezier2::try_new(vec![p(0,0),p(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),p(1,1)],vec![Real::one();5]).unwrap().parallel_left(-radius).unwrap();
            let unit=CurveParameterRange2::unit();
            let result=decided(first.parallel_intersections_on_regular_ranges(&second,&unit,&unit,&policy).unwrap());
            eprintln!("complete={} contacts={} overlaps={}",result.is_complete(),result.contacts().len(),result.overlaps().len());
            assert!(result.is_complete());assert!(result.overlaps().is_empty());assert_eq!(result.contacts().len(),1);
            let contact=&result.contacts()[0];
            assert_eq!(contact.first_parameter().polynomial_sign(&[Real::from(-89),Real::from(128)],&policy).unwrap(),Classification::Decided(RealSign::Zero));
            assert_eq!(contact.second_parameter().polynomial_sign(&[Real::from(-3),Real::zero(),Real::from(8)],&policy).unwrap(),Classification::Decided(RealSign::Zero));
        }
    }
    #[test]
    fn interior_source_cusp_reuses_owned_left_cell(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            let first=CubicBezier2::new(p(1,-1),Point2::new(q(-1,3),Real::one()),Point2::new(q(-1,3),-Real::one()),p(1,1)).parallel_left(Real::one()).unwrap();
            let second=QuadraticBezier2::new(p(1,1),Point2::new(-Real::one(),q(-1,2)),p(-3,-2)).parallel_left(Real::one()).unwrap();
            // On t in [-1/4,0), curvature is greater than one: its
            // reciprocal squared is at most 73^3 / (36*256^2) < 1.
            // Both this center locus and its source therefore have regular
            // interiors on u in [3/8,1/2], with owned one-sided cusp limits.
            assert!(73_i64.pow(3)<36*256_i64.pow(2));
            let left=CurveParameterRange2::new_validated(q(3,8).into(),q(1,2).into());
            let result=decided(first.parallel_intersections_on_regular_ranges(&second,&left,&CurveParameterRange2::unit(),&policy).unwrap());
            eprintln!("complete={} contacts={} overlaps={}",result.is_complete(),result.contacts().len(),result.overlaps().len());
            assert!(result.is_complete());assert!(result.overlaps().is_empty());assert_eq!(result.contacts().len(),1);
            let contact=&result.contacts()[0];
            assert_eq!(contact.first_parameter().polynomial_sign(&[-Real::one(),Real::from(2)],&policy).unwrap(),Classification::Decided(RealSign::Zero));
            assert_eq!(contact.second_parameter().polynomial_sign(&[Real::from(-2),Real::from(5)],&policy).unwrap(),Classification::Decided(RealSign::Zero));
            let center=decided(first.point_evidence_on_regular_range(contact.first_parameter(),&left,&policy).unwrap());
            assert_eq!(center.same_point(&p(0,-1).into(),&policy),Classification::Decided(true));
        }
    }
    #[test]
    fn regular_source_frame_keeps_contact_scale_across_a_center_cusp(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            let x=q(-3,16);
            let first=QuadraticBezier2::new(Point2::new(x.clone(),Real::from(-2)),Point2::new(x.clone(),Real::zero()),Point2::new(x,Real::from(2))).parallel_left(Real::zero()).unwrap();
            let second=RationalBezier2::try_new(vec![p(0,0),p(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),p(1,1)],vec![Real::one();5]).unwrap().parallel_left(q(15,16)).unwrap();
            // At u=sqrt(3/8), the source is (3/8,9/64), its unit
            // normal is (-3/5,4/5), and the offset is (-3/16,57/64).
            // The center/source derivative scale is 1/25 > 0 there.
            // At the unit range midpoint u=1/2 it is 1-3/sqrt(5) < 0.
            let unit=CurveParameterRange2::unit();
            let result=decided(first.parallel_intersections_on_regular_ranges(&second,&unit,&unit,&policy).unwrap());
            assert!(result.is_complete());
            let mut found=false;
            for contact in result.contacts(){
                if contact.second_parameter().polynomial_sign(&[Real::from(-3),Real::zero(),Real::from(8)],&policy).unwrap()==Classification::Decided(RealSign::Zero){
                    found=true;
                    assert_eq!(contact.first_parameter().polynomial_sign(&[Real::from(-185),Real::from(256)],&policy).unwrap(),Classification::Decided(RealSign::Zero));
                    assert_eq!(contact.tangent_cross_sign(),Some(RealSign::Negative));
                    assert_eq!(contact.tangent_dot_sign(),Some(RealSign::Positive));
                }
            }
            assert!(found,"the independently known contact was not enumerated");
        }
    }
}
