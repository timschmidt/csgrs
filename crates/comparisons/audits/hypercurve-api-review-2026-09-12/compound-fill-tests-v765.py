from pathlib import Path
import json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'compound-fill-candidate-v765'
name='hypercurve/tests/hypercurve_svg.rs';s=(W/name).read_text()
s+=r'''

#[test]
fn compound_fill_uses_global_winding_before_nesting_and_overlap_selection() {
    use hypercurve::{CurveCertainty, RegionPointLocation::{Boundary, Inside, Outside}};
    for (layout, first, same, opposite) in [
        ("nested", "M0 0 H10 V10 H0 Z", "M2 2 H8 V8 H2 Z", "M2 2 V8 H8 V2 Z"),
        ("overlap", "M0 0 H4 V4 H0 Z", "M2 0 H6 V4 H2 Z", "M2 0 V4 H6 V0 Z"),
    ] {
        for (same_direction, second) in [(true, same), (false, opposite)] {
            for fill in ["nonzero", "evenodd"] {
                for reverse_order in [false, true] {
                    let data = if reverse_order {format!("{second} {first}")} else {format!("{first} {second}")};
                    let document = format!(r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill-rule="{fill}" d="{data}"/></svg>"#);
                    let geometry=import_svg_document(&document).unwrap_or_else(|_| panic!("compound import failed: {layout}, {fill}, same={same_direction}, order={reverse_order}"));
                    let filled = fill=="nonzero" && same_direction;
                    let queries = if layout=="nested" {
                        vec![(1,1,Inside),(5,5,if filled {Inside}else{Outside}),(2,5,if filled {Inside}else{Boundary}),(0,5,Boundary),(11,5,Outside)]
                    } else {
                        vec![(1,2,Inside),(3,2,if filled {Inside}else{Outside}),(5,2,Inside),(2,2,if filled {Inside}else{Boundary}),(4,2,if filled {Inside}else{Boundary}),(0,2,Boundary),(7,2,Outside)]
                    };
                    assert_eq!(geometry.region().len(),if filled {1}else{2});
                    for (x,y,expected) in queries {
                        let result=geometry.region().classify_point(&point(x,y).into(),&CurveContext::STRICT).unwrap();
                        assert_eq!(result.certainty,CurveCertainty::Certified);
                        assert_eq!(result.value,Classification::Decided(expected),"{layout}, {fill}, same={same_direction}, order={reverse_order}, point=({x},{y})");
                    }
                }
            }
        }
    }
}

#[test]
fn compound_fill_preserves_recursive_islands_and_cancels_opposed_traversals() {
    use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
    for fill in ["nonzero","evenodd"] {
        let document=format!(r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill-rule="{fill}" d="M0 0 H10 V10 H0 Z M2 2 H8 V8 H2 Z M4 4 H6 V6 H4 Z"/></svg>"#);
        let geometry=import_svg_document(&document).unwrap();
        for (x,y,expected) in [(1,1,Inside),(3,3,if fill=="nonzero"{Inside}else{Outside}),(5,5,Inside),(4,5,if fill=="nonzero"{Inside}else{Boundary}),(11,5,Outside)] {
            assert_eq!(geometry.region().classify_point(&point(x,y).into(),&CurveContext::STRICT).unwrap().into_value(),Classification::Decided(expected));
        }
        let canceled=format!(r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill-rule="{fill}" d="M0 0 H4 V4 H0 Z M0 0 V4 H4 V0 Z"/></svg>"#);
        let geometry=import_svg_document(&canceled).unwrap();
        assert!(geometry.region().is_empty());
        assert_eq!(geometry.region().classify_point(&point(0,2).into(),&CurveContext::STRICT).unwrap().into_value(),Classification::Decided(Outside));
    }
}
'''
p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
base=json.loads((A/'compound-fill-base-v765.json').read_text());base[name]=json.loads((A/'region-admission-v763-sources.json').read_text())[name];(A/'compound-fill-base-v765.json').write_text(json.dumps(base,indent=2)+'\n')
name='hypercurve/tests/hypercurve_curve_region_promotion.rs';s=(C/name).read_text()
s+=r'''

#[test]
fn compound_fill_retains_signed_multiplicity_and_reversal_identity() {
    use RegionPointLocation::{Boundary, Inside, Outside};
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let once=path_from_contour(&square(0,0,4,4));
        let twice=CurvePath2::try_new(once.curves().iter().chain(once.curves()).cloned().collect()).unwrap();
        let opposite=certified(once.reversed(&policy).unwrap());
        for reverse in [false,true] {
            let paths=[twice.clone(),opposite.clone()].map(|path| if reverse {certified(path.reversed(&policy).unwrap())}else{path});
            for reverse_order in [false,true] {
                let ordered=if reverse_order {vec![paths[1].clone(),paths[0].clone()]}else{paths.to_vec()};
                for fill_rule in [FillRule::NonZero,FillRule::EvenOdd] {
                    let region=certified(CurveRegion2::try_from_boundary_paths(&ordered,fill_rule,&policy).unwrap());
                    assert_eq!(region.len(),1);
                    for (point,expected) in [(p(2,2),Inside),(p(0,2),Boundary),(p(5,2),Outside)] {
                        assert_eq!(certified(region.classify_point(&point.into(),&policy).unwrap()),Classification::Decided(expected));
                    }
                    let area=decided(region.filled_area(&policy).unwrap()).unwrap();
                    assert_eq!(area.partial_cmp(&Real::from(16)),Some(std::cmp::Ordering::Equal));
                    let grown=certified(region.offset(Real::one(),&OffsetCornerStyle2::Round,&policy).unwrap());
                    for (point,expected) in [(p(-1,2),Boundary),(p(0,2),Inside),(p(-2,2),Outside)] {
                        assert_eq!(certified(grown.classify_point(&point.into(),&policy).unwrap()),Classification::Decided(expected));
                    }
                    let difference=certified(region.boolean_region(&region,hypercurve::BooleanOp::Difference,&policy).unwrap());
                    assert!(difference.is_empty());
                }
            }
        }
        for fill_rule in [FillRule::NonZero,FillRule::EvenOdd] {
            assert!(certified(CurveRegion2::try_from_boundary_paths(&[],fill_rule,&policy).unwrap()).is_empty());
        }
    }
}

#[test]
fn compound_circle_fill_selects_exact_algebraic_overlap_and_canceled_seams() {
    use RegionPointLocation::{Boundary, Inside, Outside};
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let first=path_from_contour(&circle(0,0,2));
        let second=path_from_contour(&circle(2,0,2));
        for opposite in [false,true] {
            let second=if opposite {certified(second.reversed(&policy).unwrap())}else{second.clone()};
            for fill_rule in [FillRule::NonZero,FillRule::EvenOdd] {
                let region=certified(CurveRegion2::try_from_boundary_paths(&[first.clone(),second.clone()],fill_rule,&policy).unwrap());
                let filled=fill_rule==FillRule::NonZero && !opposite;
                for (point,expected) in [(p(-1,0),Inside),(p(1,0),if filled{Inside}else{Outside}),(p(3,0),Inside),(p(0,0),if filled{Inside}else{Boundary}),(p(2,0),if filled{Inside}else{Boundary}),(p(5,0),Outside)] {
                    assert_eq!(certified(region.classify_point(&point.into(),&policy).unwrap()),Classification::Decided(expected));
                }
                let replay=certified(CurveRegion2::try_from_boundary_paths(&decided(region.boundary_paths(&policy).unwrap()),FillRule::NonZero,&policy).unwrap());
                assert_eq!(certified(replay.classify_point(&p(1,0).into(),&policy).unwrap()),Classification::Decided(if filled{Inside}else{Outside}));
            }
        }
    }
}

#[test]
fn compound_fill_reuses_retained_rational_and_generated_boundaries() {
    use RegionPointLocation::{Boundary,Inside,Outside};
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let generated=axis_aligned_algebraic_rectangle(&policy);
        let generated_paths=decided(generated.boundary_paths(&policy).unwrap());
        assert_eq!(generated_paths.len(),1);
        let fixtures=[(rational_cap_path(),p(0,0),p(2,-1),p(3,0)),(generated_paths[0].clone(),Point2::new(q(1,2),q(1,2)),p(0,0),p(2,0))];
        for (path,inside,boundary,outside) in fixtures {
            let opposite=certified(path.reversed(&policy).unwrap());
            for (second,opposed) in [(path.clone(),false),(opposite,true)] {
                for fill_rule in [FillRule::NonZero,FillRule::EvenOdd] {
                    let region=certified(CurveRegion2::try_from_boundary_paths(&[path.clone(),second.clone()],fill_rule,&policy).unwrap());
                    let survives=fill_rule==FillRule::NonZero && !opposed;
                    assert_eq!(region.is_empty(),!survives);
                    for (point,expected) in [(&inside,if survives{Inside}else{Outside}),(&boundary,if survives{Boundary}else{Outside}),(&outside,Outside)] {
                        assert_eq!(certified(region.classify_point(&point.clone().into(),&policy).unwrap()),Classification::Decided(expected));
                    }
                    let replay=certified(region.boolean_region(&CurveRegion2::empty(),hypercurve::BooleanOp::Union,&policy).unwrap());
                    assert_eq!(replay.is_empty(),!survives);
                }
            }
        }
    }
}
'''
(C/name).write_text(s)
print('Added five compound-fill regression cases;30 candidate paths')
