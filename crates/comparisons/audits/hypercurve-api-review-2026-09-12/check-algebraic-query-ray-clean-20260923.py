from pathlib import Path
import hashlib,json,os,subprocess
p=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-algebraic-query-ray-clean-2026-09-23')
base=Path('/tmp/hypercurve-algebraic-query-ray-ownership-2026-09-23')
prefix='algebraic-query-ray-ownership-20260923-clean1'
old=json.loads((p/'algebraic-query-ray-ownership-20260923-focused1-sources.json').read_text())
new=json.loads((p/f'{prefix}-sources.json').read_text())
report=json.loads((p/'algebraic-query-ray-ownership-20260923-broad1-terminal.json').read_text())
assert report['all_processes_reaped'] and not report['new_failures'] and not report['changed_existing_nonpasses']
line='    retained_point_linear_difference_to_algebraic_sign,\n'
assert (base/'hypercurve/src/bezier_region.rs').read_text().replace(line,'')==(root/'hypercurve/src/bezier_region.rs').read_text()
def verify():
    for r,b in [(base,old),(root,new)]:
        for name,sha in b.items():assert hashlib.sha256((r/name).read_bytes()).hexdigest()==sha,name
verify()
env=dict(os.environ,**json.loads((p/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
checks=[]
for feature in ['--all-features','--no-default-features']:
    cmd=[cargo,'check','--all-targets',feature,'--locked','--offline']
    log=p/f'{prefix}-check-{len(checks)}.log'
    with log.open('w') as out:
        code=subprocess.run(cmd,cwd=root/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
    verify()
    checks.append(dict(command=cmd,returncode=code,log=log.name))
    assert code==0
    assert 'warning:' not in log.read_text()
    print('Passed without warnings',feature,flush=True)
terminal=dict(checks=checks,only_change_from_broad_is_unused_import_removal=True,all_sources_unchanged=True,all_processes_reaped=True,region_sha256=new['hypercurve/src/bezier_region.rs'])
(p/f'{prefix}-terminal.json').write_text(json.dumps(terminal,indent=2)+'\n')
print('Terminal; final compilation checks reaped.',flush=True)
