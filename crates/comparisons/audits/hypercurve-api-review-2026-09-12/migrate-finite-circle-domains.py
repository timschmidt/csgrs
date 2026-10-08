from pathlib import Path
import re
root=Path('/home/tim/Documents/GitHub/workspace/hypercurve/src')
p=root/'bezier_offset.rs';s=p.read_text()
names=['rational_intersections','rational_intersections_with_parameter_map','rational_intersections_internal','selected_parallel_normal_rational_system','selected_parallel_normal_rational_intersections_internal','selected_parallel_normal_rational_selected_fiber_intersections','selected_parallel_normal_replay_rational_circle_component','selected_radial_rational_system','selected_radial_replay_rational_circle_component','recursive_selected_radial_rational_system','recursive_selected_radial_rational_intersections_internal','recursive_selected_radial_replay_rational_circle_component','chord_normal_projective_rational_system','chord_normal_projective_rational_intersections_internal','chord_normal_projective_replay_rational_circle_component','represented_rational_system','represented_rational_component_system','represented_rational_intersections_internal','replay_rational_circle_component_with_exact_frame','replay_rational_circle_component','rational_frame_selected_fiber_intersections','exact_line_image_selected_radial_rational_intersections']
for name in names:
    m=re.search(r'^    (?:pub\(crate\) )?fn '+name+r'\(',s,re.M);assert m,name
    end=s.index('\n    }\n',m.start())+7; chunk=s[m.start():end]
    assert '        other: &RationalBezier2,\n' in chunk,name
    chunk=chunk.replace('        other: &RationalBezier2,\n','        other: &RationalBezier2,\n        range: &CurveParameterRange2,\n',1)
    # These builders need a sign on this chart, not the authored unit interval.
    chunk=chunk.replace('other.denominator_sign(&crate::CurveParameterRange2::unit())','other.denominator_sign(range)')
    s=s[:m.start()]+chunk+s[end:]
p.write_text(s)
# Insert a unit range at existing callers of unambiguous private methods. Each
# finite query owner is then migrated to its actual range explicitly below.
for p in root.glob('*.rs'):
    s=p.read_text(); insert=[]
    for name in names:
        if name=='rational_intersections': continue
        for m in re.finditer(r'\.'+name+r'\s*\(',s):
            i=m.end(); depth=0
            while i<len(s):
                c=s[i]
                if c in '([{':depth+=1
                elif c in ')]}':depth-=1
                elif c==',' and depth==0:
                    insert.append(i+1);break
                i+=1
            else:raise RuntimeError(name)
    for i in sorted(insert,reverse=True):s=s[:i]+' &crate::CurveParameterRange2::unit(),'+s[i:]
    p.write_text(s)
# Inside finite circle functions, pass the domain onward and use it to schedule
# isolated candidates. Component endpoint/envelope publication follows below.
p=root/'bezier_offset.rs';s=p.read_text()
for name in names:
    m=re.search(r'^    (?:pub\(crate\) )?fn '+name+r'\(',s,re.M); end=s.index('\n    }\n',m.start())+7;chunk=s[m.start():end]
    chunk=chunk.replace('&crate::CurveParameterRange2::unit(),','range,').replace('SelectedThirdAxisDomain2::Finite(&CurveParameterRange2::unit())','SelectedThirdAxisDomain2::Finite(range)')
    # The already general selected-parameter projection takes its finite range
    # immediately before policy.
    chunk=chunk.replace('            &CurveParameterRange2::unit(),\n            policy,','            range,\n            policy,')
    s=s[:m.start()]+chunk+s[end:]
p.write_text(s)
print('Migrated explicit range signatures:',len(names))
