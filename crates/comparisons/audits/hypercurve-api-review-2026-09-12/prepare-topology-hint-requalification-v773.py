from pathlib import Path
import json
A=Path(__file__).resolve().parent
old=json.loads((A/'topology-hint-removal-v772-terminal.json').read_text());names=old['expected_cases'];target='hypercurve_curve_region_boolean_fuzz';names[target]=['authored_loop_semantics_support_reversed_nonuniform_rational_regions'if n=='explicit_loop_topology_supports_reversed_nonuniform_rational_regions'else n for n in names[target]]
(A/'topology-hint-removal-cases-v773.json').write_text(json.dumps(names,indent=2)+'\n')
s=(A/'qualify-topology-hint-removal-v772.py').read_text().replace("prefix='topology-hint-removal-v772'","prefix='topology-hint-removal-v773'").replace('topology-hint-removal-promotion-v772','topology-hint-removal-promotion-v773').replace('topology-hint-removal-cases-v771','topology-hint-removal-cases-v773')
s=s.replace("[cargo,'test','--lib',","[cargo,'test',")
marker='verify();save();code=0\n';assert s.count(marker)==1
proof='''previous=json.loads((A/'topology-hint-removal-v772-terminal.json').read_text())
previous_reaped=json.loads((A/'topology-hint-removal-v772-reaped.json').read_text());assert previous_reaped['outer_exit_code']==1 and previous['all_processes_reaped']
previous_manifest=json.loads((A/previous['source_manifest']).read_text());test_file='hypercurve/tests/hypercurve_curve_region_boolean_fuzz.rs';target='hypercurve_curve_region_boolean_fuzz'
assert {n for n in manifest if manifest[n]!=previous_manifest[n]}=={test_file}
correction=promotion['test_only_correction'];before=(Path(previous['source_directory'])/test_file).read_text();assert before.count(correction['removed_assertion'])==1
expected=before.replace(correction['removed_assertion'],'',1).replace('fn '+correction['renamed_test'][0]+'()','fn '+correction['renamed_test'][1]+'()',1)
assert expected==(W/test_file).read_text()
# No other source references or embeds the modified integration-test file.
for name in manifest:
 if name.endswith('.rs')and name!=test_file:assert 'hypercurve_curve_region_boolean_fuzz.rs'not in(W/name).read_text(),name
assert len(previous['cases'])==378 and sum(c['passed']for c in previous['cases'])==377
assert len(previous['builds'])==1 and previous['builds'][0]['returncode']==0
retained_build=dict(previous['builds'][0]);retained_build['binaries']={k:v for k,v in retained_build['binaries'].items()if k!=target};assert len(retained_build['binaries'])==7
report['builds']=[retained_build];report['test_listings']={k:v for k,v in previous['test_listings'].items()if k!=target};report['cases']=[c for c in previous['cases']if c['passed']];assert all(c['target']!=target for c in report['cases'])
report['reused_builds_from']='topology-hint-removal-v772-terminal.json';report['source_change_proof']=dict(only_changed_source=test_file,removed_obsolete_assertion=correction['removed_assertion'],renamed_test=correction['renamed_test'],reused_unchanged_cases=377)
reused={(c['target'],c['name'])for c in report['cases']}
verify();save();code=0
'''
s=s.replace(marker,proof)
a=" binaries=compile_tests('hypercurve',['hypercurve',*[n for n in names if n not in {'hypercurve','hyperbrep','csgrs'}]])";assert a in s
s=s.replace(a," binaries=dict(retained_build['binaries']);binaries.update(compile_tests('hypercurve',[target]));print('Reused377 unchanged passing cases; rebuilt only the corrected integration target',flush=True)")
s=s.replace('  for name in selected:\n   log=',"  for name in selected:\n   if (target,name)in reused:continue\n   log=")
(A/'qualify-topology-hint-removal-v773.py').write_text(s)
print('Prepared isolated integration-target requalification plus all7 final checks')
