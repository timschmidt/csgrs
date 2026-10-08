from pathlib import Path
import hashlib,json,os,re,subprocess,sys,time
A=Path(__file__).resolve().parent
W=A.parent
prefix='corner-native-field-replay-20260926-v147'
report=json.loads((A/f'{prefix}-terminal.json').read_text())
guard=json.loads((A/report['workspace_guard']).read_text())
manifest=json.loads((A/report['source_manifest']).read_text())
archive=Path(report['source_directory'])
build=Path(report['build_source_directory'])
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report['pre_resume_failures']=[dict(outer_session=6047,exit_code=1,reason='Five test names used tests instead of conversion_tests; no tests executed before selection assertion. All four checks and the fresh release build passed.')]
def verify():
    for name,sha in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == guard[name], name
        for root in [archive,build]:
            assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha,name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']

def save():
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')

def run(repo,label,command,limit,group):
    report['active'] = label
    save()
    log = A/f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command,cwd=build/repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo=repo,label=label,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name)
    report[group].append(row)
    report.pop('active')
    save()
    print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-2400:] if code else '',flush=True)
    return row

def stop():
    verify()
    report['all_sources_unchanged'] = True
    report['all_processes_reaped'] = True
    save()
    raise SystemExit(1)

def compile(repo):
    command = [cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    report['active'] = repo+'-build'
    save()
    with (A/f'{prefix}-{repo}-build.jsonl').open('w') as out, (A/f'{prefix}-{repo}-build.log').open('w') as err:
        code = subprocess.run(command,cwd=build/repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
    report.pop('active')
    report[repo+'_build_returncode'] = code
    if code:
        print((A/f'{prefix}-{repo}-build.log').read_text()[-3000:],flush=True)
        stop()
    rows = [json.loads(line) for line in (A/f'{prefix}-{repo}-build.jsonl').read_text().splitlines()]
    artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
    assert not artifact['fresh'],repo
    binary = A/f'{prefix}-{repo}'
    shutil.copy2(artifact['executable'],binary)
    report['binaries'][repo] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    save()
    return binary

verify()
binary=Path(report['binaries']['hypercurve']['path'])
jobs = [
    'curve_support::tests::parallel_injectivity_requires_regularity_on_the_requested_range',
    'bezier_offset::conversion_tests::formula_roots_compare_in_their_existing_native_coefficient_field',
    'bezier_region::tests::one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once',
    'bezier_region::tests::one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier',
    'bezier_offset::conversion_tests::local_parallel_endpoint_clipping_reuses_a_coefficient_root',
    'bezier_offset::conversion_tests::split_chord_linear_tangents_reuse_the_oriented_support',
    'bezier_offset::conversion_tests::recursive_chord_parallel_retains_owned_roots_across_exterior_ranges',
    'bezier_offset::conversion_tests::translated_parallel_endpoint_does_not_certify_source_incidence',
]
names = [line[:-6] for line in subprocess.check_output([str(binary),'--list'], text=True).splitlines() if line.endswith(': test')]
assert set(jobs) <= set(names), set(jobs)-set(names)
for index,name in enumerate(jobs):
    row=run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],90,'cases')
    row['name']=name
    if not row['returncode']:
        assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text()),name
    save()
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
report['qualification_complete']=not any(row['returncode'] for row in report['cases']+report['checks'])
save()
print('Every owned process reaped; success=',report['qualification_complete'],flush=True)
sys.exit(0 if report['qualification_complete'] else 1)
