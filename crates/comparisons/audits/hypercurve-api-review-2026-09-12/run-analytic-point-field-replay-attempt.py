import subprocess,pathlib,hashlib,json,time,sys
root=pathlib.Path('/home/tim/Documents/GitHub/workspace/hypercurve'); audit=root.parent/'hypercurve-api-review-2026-09-12'; prefix='analytic-point-field-replay-attempt'+sys.argv[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
frozen={str(p.relative_to(root)):sha(p) for p in [root/'src/curve_region_boolean.rs',root/'src/bezier_offset.rs']}
cmd=['cargo','test','--release','--all-features','--lib','--no-run','--offline','--message-format=json','-j','2']
t0=time.monotonic()
with (audit/(prefix+'-build.jsonl')).open('w') as out, (audit/(prefix+'-build.log')).open('w') as err:
    r=subprocess.run(cmd,cwd=root,stdout=out,stderr=err,timeout=600)
record={'build_command':cmd,'build_returncode':r.returncode,'build_elapsed_seconds':time.monotonic()-t0,'source_sha256':frozen,'tests':[]}
print('build',r.returncode,record['build_elapsed_seconds'],flush=True)
if r.returncode==0:
    binary=root/'target/release/deps/hypercurve-6e6572a45c5b177b';record['binary_sha256']=sha(binary)
    for name in ['algebraic_chord_analytic_parallel_pair_replays_contacts_and_overlap','analytic_point_equality_replays_algebraic_source_and_normal_sheet','analytic_axis_order_reuses_polynomial_speed_across_reduced_tangent_fields']:
        log=audit/(prefix+'-'+name+'.log'); t0=time.monotonic()
        with log.open('w') as out:
            try:
                r=subprocess.run([str(binary),name,'--nocapture','--test-threads=1'],stdout=out,stderr=subprocess.STDOUT,timeout=120)
                code=r.returncode
            except subprocess.TimeoutExpired:
                code=124
        row={'test':name,'returncode':code,'elapsed_seconds':time.monotonic()-t0,'timeout_seconds':120,'log':log.name};record['tests'].append(row)
        print(json.dumps(row),flush=True);print(log.read_text()[-5000:],flush=True)
else:
    for line in (audit/(prefix+'-build.jsonl')).read_text().splitlines():
        item=json.loads(line)
        if item.get('reason')=='compiler-message': print(item['message'].get('rendered',item['message']['message']),flush=True)
assert all(sha(root/p)==h for p,h in frozen.items())
(audit/(prefix+'.json')).write_text(json.dumps(record,indent=2)+'\n')
