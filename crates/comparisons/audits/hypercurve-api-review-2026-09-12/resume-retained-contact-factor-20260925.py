from pathlib import Path
import hashlib, json, re, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-contact-factor-20260925-v98'
path = A/f'{prefix}-terminal.json'
report = json.loads(path.read_text())
assert report['all_processes_reaped'] and report['driver_failure']
assert report['hypersolve_passed'] == 514 and len(report['cases']) == 1
guard = json.loads((A/report['workspace_guard']).read_text())
manifest = json.loads((A/report['source_manifest']).read_text())
archive = Path(report['source_directory'])
build = Path(report['build_source_directory'])
def verify():
    for repo, head in report['parents'].items():
        assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip() == head
    for name, sha in guard.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((archive/name).read_bytes()).hexdigest() == manifest[name], name
        assert hashlib.sha256((build/name).read_bytes()).hexdigest() == manifest[name], name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
def save():
    verify()
    path.write_text(json.dumps(report,indent=2)+'\n')
verify()
names = {target: [line[:-6] for line in subprocess.check_output([binary['path'],'--list'],text=True).splitlines()
                  if line.endswith(': test')] for target,binary in report['binaries'].items() if target != 'hypersolve'}
prior = json.loads((A/'selected-frame-import-20260925-v94-candidate-terminal.json').read_text())
jobs = [(row['target'],row['name']) for row in prior['cases']]
extra = [
    'retained_parameter_import_keeps_original_axes_and_tower',
    'exterior_chord_contact_deflation_preserves_other_contacts',
    'recursive_quadratic_endpoint_roots_keep_the_original_field',
    'certified_circle_chord_endpoint_factor_preserves_roots_and_field',
    'algebraic_chord_source_incidence_deflates_only_the_retained_contact',
    'recursive_polynomial_isolators_keep_the_selected_root_after_endpoint_deflation',
    'recursive_ordered_field_isolation_does_not_guess_polynomial_degree',
]
for suffix in extra:
    matching = [name for name in names['hypercurve'] if name.rsplit('::',1)[-1] == suffix]
    assert len(matching) == 1, suffix
    if ('hypercurve',matching[0]) not in jobs:
        jobs.append(('hypercurve',matching[0]))
jobs.sort(key=lambda job: (job[1].rsplit('::',1)[-1] not in extra,job))
report.update(selection=jobs,all_processes_reaped=False,
              resumed_after=f'{prefix}-pre-resume-driver-failure.json',
              skipped_pending_migration_control='local_parallel_endpoint_clipping_reuses_a_coefficient_root')
report.pop('driver_failure')
save()
print('Running',len(jobs),'Hypercurve cases from freshly built, source-bound executables.',flush=True)
for index,(target,name) in enumerate(jobs):
    assert name in names[target],name
    verify()
    report['active'] = [target,name]
    save()
    log = A/f'{prefix}-case-{index:03}.log'
    assert not log.exists()
    command = [report['binaries'][target]['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never']
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command,cwd=build/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo='hypercurve',label=f'case-{index:03}',target=target,name=name,command=command,
               returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name)
    report['cases'].append(row)
    report.pop('active')
    save()
    print(index,name.rsplit('::',1)[-1],code,log.read_text()[-2000:] if code else '',flush=True)
    if code or not re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()):
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
report.update(hypercurve_passed=len(jobs),all_processes_reaped=True,qualification_complete=True)
save()
print('Passed:',report['hypersolve_passed'],'Hypersolve;',len(jobs),'Hypercurve cases.',flush=True)
