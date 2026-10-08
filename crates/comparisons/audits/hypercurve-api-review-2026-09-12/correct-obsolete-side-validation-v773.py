from pathlib import Path
import hashlib,json,shutil,subprocess
A=Path(__file__).resolve().parent;W=A.parent;C=A/'topology-hint-removal-candidate-v773';assert not C.exists()
prior=json.loads((A/'topology-hint-removal-promotion-v772.json').read_text());r=json.loads((A/'topology-hint-removal-v772-terminal.json').read_text());receipt=json.loads((A/'topology-hint-removal-v772-reaped.json').read_text());assert receipt['outer_exit_code']==1 and r['all_processes_reaped']and len(r['cases'])==378 and sum(c['passed']for c in r['cases'])==377
manifest=json.loads((A/r['source_manifest']).read_text());name='hypercurve/tests/hypercurve_curve_region_boolean_fuzz.rs'
for n,sha in manifest.items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==sha,n
for n,path in prior['candidates'].items():p=C/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,p)
s=(C/name).read_text();old_name='explicit_loop_topology_supports_reversed_nonuniform_rational_regions';new_name='authored_loop_semantics_support_reversed_nonuniform_rational_regions';start=s.index('fn '+old_name+'()');a=s.index('    assert!(\n        CurveRegion2::try_from_boundary_paths_with_loop_semantics(',start);b=s.index('    let forward = ',a);removed=s[a:b];assert 'interior-side evidence count must match the authored loops'in removed and removed.count('assert!(')==1
s=(s[:a]+s[b:]).replace('fn '+old_name+'()','fn '+new_name+'()',1);(C/name).write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',str(C/name)],cwd=W/'hypercurve',check=True)
for n,sha in manifest.items():assert hashlib.sha256((W/n).read_bytes()).hexdigest()==sha,n
(W/name).write_bytes((C/name).read_bytes());prior['candidates']={n:str(C/n)for n in prior['candidates']};prior['promoted']={n:hashlib.sha256((W/n).read_bytes()).hexdigest()for n in prior['candidates']};prior['test_only_correction']=dict(path=name,before=manifest[name],after=prior['promoted'][name],removed_assertion=removed,renamed_test=[old_name,new_name]);(A/'topology-hint-removal-promotion-v773.json').write_text(json.dumps(prior,indent=2)+'\n')
print('Removed only obsolete side-count assertion and renamed its retained geometry test')
