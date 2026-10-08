#[cfg(test)]
mod regular_pair_exterior_domain_probe {
    use super::*;
    fn q(n:i64,d:i64)->Real{(Real::from(n)/Real::from(d)).unwrap()}
    fn decided<T>(value:Classification<T>)->T{match value{Classification::Decided(value)=>value,Classification::Uncertain(reason)=>panic!("retained regular pair blocked: {reason:?}")}}
    #[test]
    fn exterior_regular_pair_keeps_contacts_outside_ancestral_bounds(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            let points=vec![Point2::from_values(0,0),Point2::from_values(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),Point2::from_values(1,1)];
            let other=points.iter().map(|point|Point2::new(Real::from(13)-point.y(),Real::from(21)+point.x())).collect();
            let distance=Real::from(65).sqrt().unwrap();
            let first=RationalBezier2::try_new(points,vec![Real::one();5]).unwrap().parallel_left(distance.clone()).unwrap();
            let second=RationalBezier2::try_new(other,vec![Real::one();5]).unwrap().parallel_left(distance).unwrap();
            let range=CurveParameterRange2::new_validated(q(3,2).into(),q(5,2).into());
            let parameter:CurveParameter2=Real::from(2).into();
            let center:CurvePoint2=Point2::from_values(-4,17).into();
            // B(2)=(4,16), its normal times sqrt(65) is (-8,1).
            // The second source is R90(B)+(13,21), so both parallels
            // pass through (-4,17). Their authored unit-span offset boxes
            // are disjoint: 1+sqrt(65) < 21-sqrt(65).
            for parallel in [&first,&second]{
                let point=decided(parallel.point_evidence_on_regular_range(&parameter,&range,&policy).unwrap());
                assert_eq!(point.same_point(&center,&policy),Classification::Decided(true));
            }
            let result=decided(first.parallel_intersections_on_regular_ranges(&second,&range,&range,&policy).unwrap());
            eprintln!("complete={} contacts={} overlaps={}",result.is_complete(),result.contacts().len(),result.overlaps().len());
            assert!(result.is_complete());
            let mut found=false;
            for contact in result.contacts(){
                found|=contact.first_parameter().polynomial_sign(&[Real::from(-2),Real::one()],&policy).unwrap()==Classification::Decided(RealSign::Zero)
                    &&contact.second_parameter().polynomial_sign(&[Real::from(-2),Real::one()],&policy).unwrap()==Classification::Decided(RealSign::Zero);
            }
            assert!(found,"the independently known exterior contact was omitted");
        }
    }
    #[test]
    fn general_regular_pair_replays_unequal_contact_scale_changes(){
        for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512]{
            let points=vec![Point2::from_values(0,0),Point2::from_values(0,0),Point2::new(q(1,6),Real::zero()),Point2::new(q(1,2),Real::zero()),Point2::from_values(1,1)];
            let other=points.iter().map(|point|Point2::new(q(-51,64)-point.y(),q(-3,64)+point.x())).collect();
            let first=RationalBezier2::try_new(points,vec![Real::one();5]).unwrap().parallel_left(q(15,16)).unwrap();
            let second=RationalBezier2::try_new(other,vec![Real::one();5]).unwrap().parallel_left(q(-15,16)).unwrap();
            let range=CurveParameterRange2::unit();
            let parameter:CurveParameter2=q(3,8).sqrt().unwrap().into();
            let center:CurvePoint2=Point2::new(q(-3,16),q(57,64)).into();
            // The first scale is 1/25 at the contact, but negative at the
            // range midpoint. The second scale is 49/25 at the contact
            // and positive everywhere. Neither carrier has a rational
            // parallel image; the general pair replay must keep this crossing.
            for parallel in [&first,&second]{
                let point=decided(parallel.point_evidence_on_regular_range(&parameter,&range,&policy).unwrap());
                assert_eq!(point.same_point(&center,&policy),Classification::Decided(true));
            }
            let result=decided(first.parallel_intersections_on_regular_ranges(&second,&range,&range,&policy).unwrap());
            eprintln!("general complete={} contacts={} overlaps={}",result.is_complete(),result.contacts().len(),result.overlaps().len());
            assert!(result.is_complete());
            let mut found=false;
            for contact in result.contacts(){
                if [contact.first_parameter(),contact.second_parameter()].into_iter().all(|parameter|parameter.polynomial_sign(&[Real::from(-3),Real::zero(),Real::from(8)],&policy).unwrap()==Classification::Decided(RealSign::Zero)){
                    found=true;
                    assert_eq!(contact.tangent_cross_sign(),Some(RealSign::Positive));
                    assert_eq!(contact.tangent_dot_sign(),Some(RealSign::Zero));
                }
            }
            assert!(found,"the independently known general pair contact was omitted");
        }
    }
}
