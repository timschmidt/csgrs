from pathlib import Path
import ast,difflib,gzip,hashlib,json,shutil,subprocess,tarfile
root=Path.cwd(); e=root/'docs/evidence/2026-10-13-retained-polynomial-projection'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
alg='crates/hyperreal/src/computable/node/algebra.rs'; struct='crates/hyperreal/src/computable/node/structural_analysis.rs'; rep='crates/hyperreal/src/computable/node/representation.rs'; helper='crates/hyperreal/src/computable/node/retained_polynomial.rs'
prefix='#[cfg(test)]\nmod projection_tests;\n\n'
rewrites=[
('''.pi_laurent_value_retained::<true>(
                budget,
                &mut remaining_work,
                &mut memo,
                &mut reused,
            )''','''.pi_laurent_value_retained::<true>(budget, &mut remaining_work, &mut memo, &mut reused)'''),
('''let rational = self.project_pi_laurent_polynomial(budget, |polynomial| {
            polynomial.rational_multiple(0)
        })?;''','''let rational = self
            .project_pi_laurent_polynomial(budget, |polynomial| polynomial.rational_multiple(0))?;''')]
old=(e/'algebra_projected.rs').read_text(); new=old
for before,after in rewrites:
 assert new.count(before)==1
 new=new.replace(before,after)
assert prefix+new==(root/'target/retained-polynomial-projection/projected/source'/alg).read_text()
assert not (e/'preparation-v1/protocol.json').exists()
assert not (e/'qualification-v1').exists() and not (e/'cost-v1').exists()
prior=json.loads((root/'docs/evidence/2026-10-13-retained-polynomial-views/preparation-v2/protocol.json').read_text())
parent=json.loads((root/'docs/evidence/2026-10-13-native-tower-sign-retention/preparation-v1/protocol.json').read_text())
parent_map=json.loads(gzip.decompress((root/'docs/evidence/2026-10-13-native-tower-sign-retention/preparation-v1/source-hashes.json.gz').read_bytes()))
assert all(sha(Path(parent['source'])/p)==h for p,h in parent_map.items())
archive=e/'failed-preparation-v1'; archive.mkdir(exist_ok=False)
variants={}
for name in ['baseline','recursive','projected']:
 source=root/'target/retained-polynomial-projection'/name/'source'
 origin=Path(prior['variants']['baseline' if name=='baseline' else 'candidate']['source'])
 expected=dict(parent_map)
 expected[alg]=hashlib.sha256((prefix+(new if name=='projected' else (origin/alg).read_text())).encode()).hexdigest()
 expected[rep]=sha(origin/rep)
 if name=='projected':expected[struct]=sha(e/'structural_analysis_projected.rs')
 if name!='baseline':expected[helper]=sha(e/f'retained_polynomial_{name}.rs')
 for test in ['temporary_storage_tests','memo_publication_tests','shared_radical_tests','projection_tests']:
  expected[f'crates/hyperreal/src/computable/node/{test}.rs']=sha(e/(test+'.rs'))
 actual={str(p.relative_to(source)):sha(p) for p in source.rglob('*') if p.is_file()}
 assert actual==expected,name
 changes={p:h for p,h in actual.items() if parent_map.get(p)!=h}
 maps=archive/(name+'-source-hashes.json.gz'); maps.write_bytes(gzip.compress((json.dumps(actual,sort_keys=True)+'\n').encode(),mtime=0))
 tar=archive/(name+'-delta.tar.gz')
 with tarfile.open(tar,'w:gz') as a:
  for p in sorted(changes): a.add(source/p,arcname=p)
 variants[name]={'source':str(source),'source_files':len(actual),'source_hashes_sha256':sha(maps),'delta_archive_sha256':sha(tar),'changes':changes}
fixes=[{'before':a,'after':b} for a,b in rewrites]
assessment={'status':'preparation-source-recovery-failed-on-two-rustfmt-rewrites-before-compilation','failure_sha256':sha(e/'preparation-failure-v1.json'),'executed_runner_sha256':sha(e/'prepare_v1.py'),'assessment_script_sha256':sha(Path(__file__)),'formatting_rewrites':fixes,'source_variants':variants,'all_other_source_bytes_match_declared_inputs':True,'mathematical_code_changed':False,'tests_changed':False,'qualification_executed':False,'costs_executed':False,'runtime_admitted':False,'plan_complete':False}
(e/'preparation-failure-assessment-v1.json').write_text(json.dumps(assessment,indent=2)+'\n')
# Correct only formatting expectations and freeze all other v1 inputs.
(e/'algebra_projected_v2.rs').write_text(new)
edits=json.loads((e/'projection-edits-v1.json').read_text())
for item,(before,after) in zip(edits['runtime'][alg][:2],rewrites):
 assert item['after'].count(before)==1
 item['after']=item['after'].replace(before,after)
(e/'projection-edits-v2.json').write_text(json.dumps(edits,indent=2)+'\n')
scope=json.loads((e/'scope-v1.json').read_text()); scope['status']='v1-format-recovery-failure-preserved-v2-inputs-recorded'
scope['preserved_failure']={'path':'preparation-failure-assessment-v1.json','sha256':sha(e/'preparation-failure-assessment-v1.json'),'correction':'Match only two rustfmt rewrites in recorded expected method fragments; all mathematical and test tokens otherwise unchanged. Fresh v2 source maps must equal all three frozen formatted v1 maps.'}
(e/'scope-v2.json').write_text(json.dumps(scope,indent=2)+'\n')
for name in ['prepare','qualify','assess_qualification','measure','assess_cost']:
 s=(e/f'{name}_v1.py').read_text()
 for runner in ['prepare','qualify','assess_qualification','measure','assess_cost']:
  s=s.replace(runner+'_v1',runner+'_v2')
 for oldpath,newpath in [('scope-v1.json','scope-v2.json'),('projection-edits-v1.json','projection-edits-v2.json'),('algebra_projected.rs','algebra_projected_v2.rs'),('qualification-v1','qualification-v2'),('correctness-v1.json','correctness-v2.json'),('cost-assessment-v1.json','cost-assessment-v2.json'),("E / 'cost-v1'","E / 'cost-v2'"),("E / 'preparation-v1'","E / 'preparation-v2'"),("E / 'preparation-v1/protocol.json'","E / 'preparation-v2/protocol.json'"),('target/retained-polynomial-projection','target/retained-polynomial-projection-v2'),('cost-v1-binaries','cost-v2-binaries'),('preparation-failure-v1.json','preparation-failure-v2.json')]:
  s=s.replace(oldpath,newpath)
 if name=='prepare':
  s=s.replace("    WORK.mkdir(exist_ok=False)","""    failure = json.loads((E / 'preparation-failure-assessment-v1.json').read_text())
    assert sha(E / 'preparation-failure-assessment-v1.json') == review['preserved_failure']['sha256']
    assert failure['all_other_source_bytes_match_declared_inputs']
    assert not failure['qualification_executed'] and not failure['costs_executed']
    for name, frozen in failure['source_variants'].items():
        path = E / 'failed-preparation-v1' / (name + '-source-hashes.json.gz')
        assert sha(path) == frozen['source_hashes_sha256']
        assert all(sha(Path(frozen['source']) / p) == h for p, h in json.loads(gzip.decompress(path.read_bytes())).items())
    WORK.mkdir(exist_ok=False)""")
  s=s.replace("        assert len(current) == (10135 if variant == 'baseline' else 10136)","""        assert len(current) == (10135 if variant == 'baseline' else 10136)
        assert current == json.loads(gzip.decompress((E / 'failed-preparation-v1' / (variant + '-source-hashes.json.gz')).read_bytes()))""")
  s=s.replace("        'main_source_sha256': guard,", "        'failed_v1_assessment_sha256': sha(E / 'preparation-failure-assessment-v1.json'),\n        'all_source_maps_equal_formatted_v1': True,\n        'main_source_sha256': guard,")
 if name=='qualify':
  s=s.replace("    for name, digest in preparation['input_sha256'].items():", """    assert sha(E / 'preparation-failure-assessment-v1.json') == preparation['failed_v1_assessment_sha256']
    assert preparation['all_source_maps_equal_formatted_v1']
    failure = json.loads((E / 'preparation-failure-assessment-v1.json').read_text())
    for variant, frozen in failure['source_variants'].items():
        path = E / 'failed-preparation-v1' / (variant + '-source-hashes.json.gz')
        assert sha(path) == frozen['source_hashes_sha256']
        hashes = json.loads(gzip.decompress(path.read_bytes()))
        assert all(sha(Path(frozen['source']) / p) == h for p, h in hashes.items())
        assert hashes == json.loads(gzip.decompress((PREP / (variant + '-source-hashes.json.gz')).read_bytes()))
    for name, digest in preparation['input_sha256'].items():""")
 ast.parse(s)
 (e/f'{name}_v2.py').write_text(s)
print({'failure_frozen':True,'source_counts':{k:v['source_files'] for k,v in variants.items()},'format_rewrites':2,'new_runners_unexecuted':True})
