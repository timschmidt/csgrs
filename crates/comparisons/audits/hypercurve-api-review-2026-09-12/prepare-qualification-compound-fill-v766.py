from pathlib import Path
import json
A=Path(__file__).resolve().parent
old=json.loads((A/'common-point-classification-v744-terminal.json').read_text())
current=json.loads((A/'region-admission-cases-v763.json').read_text())
lib=set(current['hypercurve'])
for case in old['cases']:
 if case['target']=='hypercurve'and case['name'].startswith(('bezier_region::','curve_region_boolean::','curve::','finite_projection::')):lib.add(case['name'])
listing=(A/'region-admission-v763-hypercurve-list.log').read_text()
available={line.removesuffix(': test')for line in listing.splitlines()if line.endswith(': test')}
for name in available:
 if any(key in name for key in ['boundary_side_ray','algebraic_side_ray','reversed_source_charts','extended_algebraic']):lib.add(name)
assert lib<=available,lib-available
names={'hypercurve_svg':None,'hypercurve_curve_region_promotion':None,'hypercurve':sorted(lib)}
for target in ['hypercurve_analytic_parallel_region','hypercurve_bezier_region','hypercurve_bspline','hypercurve_curve','hypercurve_curve_region_boolean','hypercurve_path_closure']:names[target]=None
names['hypercurve_curve_region_boolean_fuzz']=['explicit_loop_topology_supports_reversed_nonuniform_rational_regions','deterministic_transverse_curve_family_pair_matrix_completes','deterministic_endpoint_curve_family_pair_matrix_completes','deterministic_tangent_curve_family_pair_matrix_completes','deterministic_coincident_curve_family_images_complete']
for target in ['hyperbrep','csgrs']:names[target]=[c['name']for c in old['cases']if c['target']==target]
(A/'compound-fill-cases-v766.json').write_text(json.dumps(names,indent=2)+'\n')
s=(A/'qualify-region-admission-v763.py').read_text().replace("prefix='region-admission-v763'","prefix='compound-fill-v766'").replace("prior_prefix='nesting-result-removal-v757'","prior_prefix='region-admission-v763'").replace('region-admission-promotion-v763','compound-fill-promotion-v766').replace('region-admission-cases-v763','compound-fill-cases-v766')
s=s.replace("names[target]=sorted(n for n in available if target!='csgrs' or n.startswith('curve::native::tests::'))","names[target]=sorted((n for n in available if target!='csgrs' or n.startswith('curve::native::tests::')),key=lambda n:(not n.startswith('compound_'),n))")
s=s.replace(" for target,selected in names.items():\n  for name in selected:"," for target,selected in names.items():\n  if target not in binaries:binaries.update(compile_tests(target,[target]))\n  for name in selected:")
s=s.replace("for n in sorted(changed)if n.endswith('.rs')","for n in sorted(changed)if n.endswith('.rs')and not n.startswith('csgrs/')")
needle="  ('fuzz-check',";pos=s.index(needle)
s=s[:pos]+"  ('csgrs-format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--config-path',str(W/'csgrs/.rustfmt.toml'),'--check',*[str(build/n)for n in sorted(changed)if n.endswith('.rs')and n.startswith('csgrs/')]],'csgrs'),\n"+s[pos:]
(A/'qualify-compound-fill-v766.py').write_text(s)
print('Prepared12target qualification;',len(lib),'selected library cases')
