    #[test]
    fn selected_parallel_ray_vertices_use_spatial_endpoint_ownership() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let center_support = QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(p(-1, 0), p(-1, -1)).unwrap(),
            ).parallel_left(Real::zero()).unwrap();
            let Classification::Decided(Some(circle)) =
                crate::bezier_offset::BezierAlgebraicCuspSemicircle2::from_selected_parallel_normal(
                    center_support, BezierParameter2::Exact(Real::zero()), Real::one(), false, &policy,
                ).unwrap()
            else { panic!("the regular exact circle frame must construct") };
            let Classification::Decided(quarter) = crate::BezierAlgebraicCuspSemicircleFragment2::try_new(
                circle.complementary_half(),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(Real::zero()),
                crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(q(1,2)),
                false, &policy,
            ).unwrap() else { panic!("the lower-left quarter must construct") };
            let parallel = QuadraticBezier2::from_line_segment(
                LineSeg2::try_new(p(-1,-1),p(0,0)).unwrap(),
            ).parallel_left(Real::zero()).unwrap();
            let selected = BezierSplitFragment2::SelectedFiber(
                crate::bezier_split::BezierSelectedFiberFragment2::new(
                    crate::bezier_split::BezierSelectedFiberSource2::AnalyticParallel(parallel),
                    CurveParameterRange2::unit(), p(-1,-1).into(), p(0,0).into(),
                ),
            );
            // Three quarters of the unit circle centered at (-1,0), closed
            // by its diagonal chord. The entire boundary lies at x <= 0.
            let fragments = vec![
                BezierSplitFragment2::AlgebraicCuspSemicircle(
                    crate::BezierAlgebraicCuspSemicircleFragment2::full(circle,&policy)),
                BezierSplitFragment2::AlgebraicCuspSemicircle(quarter),
                selected,
            ];
            for reversed in [false,true] {
                let fragments = if reversed { fragments.iter().rev().map(|fragment| fragment.reversed().unwrap()).collect() } else { fragments.clone() };
                let boundary = CurveRegionBoundaryLoop2::new(fragments,&policy).unwrap();
                let origin=p(1,0);
                let ray=ray_candidates(&origin).remove(0);
                assert_eq!(classify_point_with_retained_ray_skipping_origin(&boundary,&origin,&ray,None,&policy).unwrap(),
                    Classification::Decided(RetainedRayWinding::Winding(0)),
                    "the left ray crosses both the arc and closing chord: reversed={reversed}, policy={policy:?}");
                for (point,expected) in [(origin,ContourPointLocation::Outside),(p(-1,0),ContourPointLocation::Inside),(p(0,0),ContourPointLocation::Boundary)] {
                    assert_eq!(boundary.classify_point_raw(&point,&policy).unwrap(),Classification::Decided(expected));
                }
            }
        }
    }

