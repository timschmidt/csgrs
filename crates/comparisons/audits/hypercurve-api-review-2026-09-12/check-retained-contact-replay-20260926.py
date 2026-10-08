from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
prefix = f'retained-contact-replay-20260926-{version}'
manifest_name = f'local-chord-complete-replay-20260924-{version}-sources.json'
manifest = json.loads((A/manifest_name).read_text())
archive = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924').resolve()
build = A/'build-workspace-20260925'
for name in manifest:
    source, destination = archive/name, build/name
    if not destination.exists() or source.read_bytes() != destination.read_bytes():
        shutil.copy2(source,destination)
        os.utime(destination,None)
    assert (source.stat().st_dev,source.stat().st_ino) != (destination.stat().st_dev,destination.stat().st_ino)
env = dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain/'cargo')
report = dict(parents={repo:subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip() for repo in ['hypercurve','hypersolve','hyperreal']},
              source_manifest=manifest_name,source_directory=str(archive),build_source_directory=str(build),
              checks=[],cases=[],binaries={},all_processes_reaped=False,diagnostic_only=True)

def verify():
    for name,sha in manifest.items():
        for root in [W,archive,build]:
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
    os.utime(build/repo/'src/lib.rs',None)
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
    artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == repo and row.get('executable'))
    assert not artifact['fresh'],repo
    binary = A/f'{prefix}-{repo}'
    shutil.copy2(artifact['executable'],binary)
    report['binaries'][repo] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    save()
    return binary

verify()
for repo,files in [('hypercurve',['src/bezier_offset.rs','src/curve_region_boolean.rs','src/rational_bezier.rs'])]:
    checks = [('fmt',[str(toolchain/'rustfmt'),'--edition','2024','--check',*files])]
    checks += [(f'clippy-{index}',[cargo,'clippy','--all-targets',feature,'--locked','--offline','--','-D','warnings']) for index,feature in enumerate(['--all-features','--no-default-features'])]
    for label,command in checks:
        if run(repo,repo+'-'+label,command,1200,'checks')['returncode']:
            stop()
if run('hyperbrep','hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],1200,'checks')['returncode']:
    stop()
binary = compile('hypercurve')
names = [line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
suffixes = {
    'certified_circle_tangent_remains_nontransverse_after_residual_deflation',
    'circle_incidence_retains_oriented_transverse_signs',
    'extended_conic_retains_certified_tangency_in_its_interior',
    'parallel_arc_contacts_retain_conic_parameters_across_elevation_and_reversal',
    'projective_quadratic_conic_inverse_survives_elevation_and_reversal',
    'quadratic_conic_inverse_reuses_retained_point_field',
    'bivariate_bernstein_sign_excludes_only_strict_unit_square_misses',
    'poincare_miranda_preconditions_rotated_transverse_system_exactly',
    'analytic_parallel_self_intersection_removes_the_parameter_diagonal',
    'closed_source_diagonal_is_removed_before_residual_component_scan',
    'selected_structural_parallel_overlap_replays_off_diagonal_contacts',
    'non_source_parallel_overlap_replays_the_radical_component',
    'ordered_self_contact_domains_retain_axis_roles_after_restriction',
    'ordered_zero_parallel_domains_retain_exterior_loop_contacts',
    'ordered_ph_parallel_domains_retain_same_sheet_exterior_contacts',
    'ordered_ph_parallel_domains_reject_a_cached_opposite_normal_contact',
    'bivariate_parameter_box_sign_keeps_boundary_zeros_and_bernstein_fallback',
    'one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope',
    'selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber',
    'pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet',
    'resource_blocked_selected_corner_chamfers_in_its_affine_fiber',
}
jobs = [name for name in names if name.rsplit('::',1)[-1] in suffixes]
assert len(jobs) == len(suffixes), sorted(suffixes-{name.rsplit('::',1)[-1] for name in jobs})
jobs.sort(key=lambda name:('parallel_arc_contacts_retain' not in name,'one_fragment_nonzero' in name or 'selected_parallel_normal_circle_intersects' in name,name))
verify()
report['selection'] = jobs
save()
for index,name in enumerate(jobs):
    limit = 90
    row = run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],limit,'cases')
    row['name'] = name
    save()
    if row['returncode'] == 0:
        assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',(A/row['log']).read_text())
verify()
report['all_sources_unchanged'] = True
report['all_processes_reaped'] = True
save()
print('Complete; every owned process reaped.',flush=True)
raise SystemExit(int(any(row['returncode'] != 0 for row in report['cases'])))
