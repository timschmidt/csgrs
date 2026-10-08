from pathlib import Path
import re
p=Path('/home/tim/Documents/GitHub/workspace/hypercurve/tests/hypercurve_curve_region_promotion.rs')
s=p.read_text()

def once(s,a,b):
    assert s.count(a)==1, (a,s.count(a))
    return s.replace(a,b)

def function(name, change):
    global s
    start=s.index('fn '+name+'(')
    following=re.search(r'\nfn ',s[start+3:])
    end=start+3+following.start() if following else len(s)
    s=s[:start]+change(s[start:end])+s[end:]

def calls(text,name,rewrite):
    pos=0
    while (start:=text.find(name+'(',pos))>=0:
        opening=start+len(name)
        depth=1; end=opening+1
        while depth:
            if text[end]=='(': depth+=1
            elif text[end]==')': depth-=1
            end+=1
        args=[]; left=opening+1; nesting=0
        for i in range(left,end-1):
            if text[i] in '([{': nesting+=1
            elif text[i] in ')]}': nesting-=1
            elif text[i]==',' and nesting==0:
                args.append(text[left:i].strip()); left=i+1
        if text[left:end-1].strip(): args.append(text[left:end-1].strip())
        tail=re.match(r'\s*\.unwrap\(\)',text[end:])
        assert tail, text[start:end+70]
        end+=tail.end()
        replacement=rewrite(args,text[:start])
        text=text[:start]+replacement+text[end:]
        pos=start+len(replacement)
    return text

def admit(text,policy):
    def region(args,prefix):
        assert len(args)==4
        ctx=policy(prefix) if callable(policy) else policy
        values=[('&'+arg[4:]) if arg.startswith('vec![') else '&'+arg for arg in args]
        return 'CurveRegion2::try_from_boundary_paths_with_loop_topology(\n'+',\n'.join(values)+',\n'+ctx+',\n).unwrap().into_value()'
    return calls(text,'CurveRegion2::try_new_with_loop_topology',region)

def paths(text):
    def path(args,prefix):
        assert len(args)==2
        return 'CurvePath2::try_new_with_policy('+', '.join(args)+').unwrap().into_value()'
    return calls(text,'CurveRegionBoundaryLoop2::new',path)

def simple(text,policy):
    text=text.replace('BezierSplitFragment2::AlgebraicChord(decided(', 'Curve2::from(decided(')
    text=text.replace('.reversed().unwrap()', '.reversed('+policy+').unwrap().into_value()')
    return admit(paths(text),policy)

for name in ['axis_aligned_algebraic_rectangle','axis_aligned_algebraic_l_region','axis_aligned_algebraic_dumbbell_region','selected_endpoint_chord_pairs_share_the_linear_fillet_kernel']:
    function(name,lambda t:simple(t,'policy'))
function('shifted_algebraic_rectangle_boundary',lambda t:paths(t.replace('-> CurveRegionBoundaryLoop2','-> CurvePath2').replace('BezierSplitFragment2::AlgebraicChord(decided(', 'Curve2::from(decided(').replace('.reversed().unwrap()', '.reversed(policy).unwrap().into_value()')))
function('algebraic_material_hole_rectangle',lambda t:admit(t,'policy'))
function('selected_algebraic_round_join_retains_a_general_minor_cut',lambda t:simple(t,'&policy'))

def bezier_incidence(t):
    start=t.index('        let quadratic = BezierSplitFragment2::Materialized {')
    end=t.index('        let mut fragments',start)
    t=t[:start]+'        let quadratic = Curve2::from(QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)));\n'+t[end:]
    return simple(t,'policy')
function('selected_endpoint_chords_share_linear_bezier_fillet_incidence',bezier_incidence)

def arc_incidence(t):
    start=t.index('        let native_arc = ')
    end=t.index('        let mut fragments',start)
    t=t[:start]+'''        let arc = Curve2::from(
            CircularArc2::try_from_center(p(0, 0), p(1, 1), p(1, 0), true).unwrap(),
        );
'''+t[end:]
    return simple(t,'policy')
function('selected_endpoint_chords_share_linear_arc_fillet_incidence',arc_incidence)

def canonical(t):
    t=t.replace('BezierSplitFragment2::AlgebraicChord(chord)\n','Curve2::from(chord)\n')
    start=t.index('                let fragments = seam_source.boundary_loops()[0]')
    end=t.index('            let seam = seam_source',start)
    t=t[:start]+'''                let paths = decided(seam_source.boundary_paths(&policy).unwrap());
                let boundary = paths[0].reversed(&policy).unwrap().into_value();
                CurveRegion2::try_from_boundary_paths_with_loop_topology(
                    &[boundary],
                    &[CurveRegionLoopRole::Material],
                    &[FillRule::NonZero],
                    &[CurveBoundaryInteriorSide2::Right],
                    &policy,
                ).unwrap().into_value()
            } else {
                seam_source
            };
'''+t[end:]
    t=simple(t,'policy')
    t=once(t,'''        let first = source
            .fillet_loop_vertex_by_radius(0, 1, q(1, 2), CurveCornerMode2::TrimOnly, &policy)''','''        let (loop_index, vertex_index) = boundary_vertex_at(&source, &p(4, 0), &policy);
        let first = source
            .fillet_loop_vertex_by_radius(loop_index, vertex_index, q(1, 2), CurveCornerMode2::TrimOnly, &policy)''')
    return t
function('canonical_exact_chord_regions_fillet_without_line_demotion',canonical)

def cusp_seam(t):
    start=t.index('        let source = rounded();\n        let mut seam_fragments')
    t=t[:start]+'''        let source = rounded();
        let paths = decided(source.boundary_paths(&policy).unwrap());
        let mut curves = paths[0].curves().to_vec();
        let cusp_index = curves.iter().position(|curve| curve.family() == CurveFamily2::CircularArc)
            .expect("the round offset must retain a circular join");
        curves.rotate_left(cusp_index);
        let seam_corner = curves[0].start();
        let seam_curve_count = curves.len();
        let authored = CurvePath2::try_new_with_policy(curves, &policy).unwrap().into_value();
        for reverse in [false, true] {
            let path = if reverse {
                authored.reversed(&policy).unwrap().into_value()
            } else {
                authored.clone()
            };
            let region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
                &[path],
                &[CurveRegionLoopRole::Material],
                &[FillRule::NonZero],
                &[if reverse { CurveBoundaryInteriorSide2::Right } else { CurveBoundaryInteriorSide2::Left }],
                &policy,
            ).unwrap().into_value();
            let paths = decided(region.boundary_paths(&policy).unwrap());
            let vertex = paths[0].curves().iter().position(|curve| {
                decided(curve.start().coincides_with(&seam_corner, &policy))
            }).expect("the authored cusp seam survives normalization");
            let cut = region.chamfer_loop_vertex_by_setbacks(
                0, vertex, setback.clone(), setback.clone(), CurveCornerMode2::TrimOnly, &policy,
            ).expect("either authored traversal must retain the exact cusp seam chamfer");
            let CurveCornerSolutions2::Unique(cut) = certified(cut) else {
                panic!("the authored seam cusp must have one exact chamfer");
            };
            assert_eq!(cut.boundary_loops()[0].len(), seam_curve_count + 1);
            assert_eq!(
                decided(cut.classify_point(&Point2::new(q(1, 2), q(1, 2)), &policy).unwrap()),
                RegionPointLocation::Inside,
            );
        }
    }
}

#[test]
'''
    return t
function('selected_algebraic_cusp_chamfers_use_the_unified_retained_kernel',cusp_seam)

def cutter(t):
    start=t.index('        let cusp = match &fragments[cusp_index]')
    end=t.index('        let mapped_chamfer',start)
    t=t[:start]+t[end:]
    start=t.index('        let before_cusp_index = ')
    end=t.index('        let replay_points = ',start)
    t=t[:start]+'''        let before_cusp_index = (cusp_index + fragments.len() - 1) % fragments.len();
        let after_retained_index = (retained_index + 1) % fragments.len();
        let paths = decided(first.boundary_paths(&policy).unwrap());
        let curves = paths[0].curves();
        assert_eq!(curves.len(), fragments.len());
        let closure = decided(BezierAlgebraicChord2::try_new(
            curves[after_retained_index].end(), curves[before_cusp_index].start(), &policy,
        ).unwrap());
        let retained_boundary = CurvePath2::try_new_with_policy(
            vec![curves[retained_index].clone(), curves[after_retained_index].clone(),
                 Curve2::from(closure), curves[before_cusp_index].clone(), curves[cusp_index].clone()],
            &policy,
        ).unwrap().into_value();
        let retained_region = CurveRegion2::try_from_boundary_paths_with_loop_topology(
            &[retained_boundary], &[CurveRegionLoopRole::Material], &[FillRule::NonZero],
            &[CurveBoundaryInteriorSide2::Left], &policy,
        ).unwrap().into_value();

'''+t[end:]
    # Selection already proved this is a chord; rebuilding now uses the general curve directly.
    t=once(t,'''        let BezierSplitFragment2::AlgebraicChord(retained) = &fragments[retained_index] else {
            unreachable!("the retained fragment was selected as a chord")
        };
''','')
    return t
function('exact_support_cutter_reenters_correlated_chord_collinearly',cutter)

def coupled(t):
    start=t.index('        let mut boundaries = first')
    end=t.index('        let merged = source',start)
    t=t[:start]+'''        let mut boundaries = decided(first.boundary_paths(&policy).unwrap());
        boundaries.extend(decided(second.boundary_paths(&policy).unwrap()));
        if reverse {
            boundaries = boundaries.into_iter()
                .map(|path| path.reversed(&policy).unwrap().into_value()).collect();
        }
        let source = CurveRegion2::try_from_boundary_paths_with_loop_topology(
            &boundaries, &[CurveRegionLoopRole::Material; 2], &[fill_rule; 2],
            &[if reverse { CurveBoundaryInteriorSide2::Right } else { CurveBoundaryInteriorSide2::Left }; 2],
            &policy,
        ).unwrap().into_value();

'''+t[end:]
    return t
function('algebraic_chord_expansion_merges_coupled_material_loops_exactly',coupled)
assert s.count('CurveRegionBoundaryLoop2')==1, s.count('CurveRegionBoundaryLoop2')
s=s.replace(', CurveRegionBoundaryLoop2','')
assert 'CurveRegion2::try_new_with_loop_topology' not in s
assert 'CurveRegion2::new(' not in s
p.write_text(s)
print('Migrated all 14 raw region constructor calls in the promotion integration target.')
