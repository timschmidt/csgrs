from pathlib import Path
import hashlib,json,os,subprocess,time,sys
A=Path(__file__).resolve().parent
W=A.parent
version=sys.argv[1]
prefix=f'local-chord-complete-replay-20260924-{version}'
root=Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
manifest=json.loads((A/f'{prefix}-sources.json').read_text())
terminal=json.loads((A/f'{prefix}-terminal.json').read_text())
assert terminal['all_processes_reaped']
binary=A/f'{prefix}-libtest'
def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==terminal['binary_sha256']
    for name,sha in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
suffixes=['one_fragment_selected_projective_extensions_keep_the_local_fiber','one_fragment_selected_native_extensions_keep_the_local_fiber','retained_polynomial_chamfer_extends_exact_and_algebraic_incident_roots','retained_rational_chamfer_extends_exact_and_algebraic_pre_pole_roots','one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once','one_fragment_selected_loop_extends_chamfer_cuts_on_its_analytic_carrier','one_fragment_selected_loop_one_sided_chamfers_do_not_duplicate_the_source','selected_parallel_companion_fillets_without_range_promotion','one_fragment_retained_ph_loop_extends_fillet_on_one_analytic_carrier']
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
rows=[]
for suffix in suffixes:
    found=[name for name in names if name.endswith('::'+suffix)]
    assert len(found)==1
    log=A/f'{version}-extension-diagnostic-{suffix}.log'
    started=time.monotonic()
    with log.open('w') as out:
        try:
            code=subprocess.run([str(binary),'--exact',found[0],'--nocapture','--test-threads=1','--color','never'],cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:
            code='timeout'
    rows.append(dict(name=found[0],returncode=code,elapsed_seconds=time.monotonic()-started,limit_seconds=75,log=log.name))
    print(suffix,code,rows[-1]['elapsed_seconds'],log.read_text()[-1800:] if code else '',flush=True)
verify()
report=dict(cases=rows,all_owned_processes_reaped=True,all_sources_unchanged=True,binary_sha256=terminal['binary_sha256'],qualification=False)
(A/f'{version}-extension-diagnostic-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(int(any(row['returncode'] for row in rows)))
