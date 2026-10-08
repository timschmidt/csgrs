from pathlib import Path
import hashlib,io,json,os,shutil,subprocess,tarfile,time
workspace=Path('/home/tim/Documents/GitHub/workspace');audit=Path(__file__).resolve().parent
candidate=Path('/tmp/hypercurve-nonzero-sign-reuse-2026-09-23');root=Path('/tmp/hypercurve-nonzero-sign-parent-2026-09-23');root.mkdir()
for p in candidate.iterdir():
 if p.is_dir() and p.name!='hypersolve':(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
repo=root/'hypersolve';repo.mkdir();archive=subprocess.check_output(['git','archive','74ad6b857e042c4f8d18f67a89f08b6668a8ad41'],cwd=workspace/'hypersolve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream:stream.extractall(repo,filter='data')
p=repo/'src/algebraic_fiber.rs';s=p.read_text();n=(candidate/'hypersolve/src/algebraic_fiber.rs').read_text()
a=n.index('    #[test]\n    fn certified_nonzero_signs_');b=n.index('    #[test]\n    fn rational_subresultants_replay_degree_41',a)
i=s.index('    #[test]\n    fn rational_subresultants_replay_degree_41');p.write_text(s[:i]+n[a:b]+s[i:])
shutil.copy2(candidate/'hypersolve/tests/data/nonph_fillet_endpoint_sign.json',repo/'tests/data/nonph_fillet_endpoint_sign.json')
bindings={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*') if p.is_file()}
qualified=json.loads((audit/'nonzero-sign-reuse-20260923-final1-sources.json').read_text())
for name,sha in qualified.items():
 if not name.startswith('hypersolve/'):bindings[name]=sha
prefix='nonzero-sign-reuse-20260923-parent';(audit/(prefix+'-sources.json')).write_text(json.dumps(bindings,indent=2)+'\n')
def verify():
 for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify();env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
with (audit/(prefix+'-build.jsonl')).open('w') as out,(audit/(prefix+'-build.log')).open('w') as err:
 code=subprocess.run([cargo,'test','--release','--lib','--all-features','--no-run','--message-format=json','--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=err,timeout=900).returncode
verify();assert code==0
binary=audit/(prefix+'-libtest')
for line in (audit/(prefix+'-build.jsonl')).read_text().splitlines():
 row=json.loads(line)
 if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']=='hypersolve':shutil.copy2(row['executable'],binary)
print('Fresh parent with candidate regressions built',hashlib.sha256(binary.read_bytes()).hexdigest(),flush=True)
names=['algebraic_fiber::tests::certified_nonzero_signs_distinguish_small_values_from_selected_zeros','algebraic_fiber::tests::certified_nonzero_signs_share_the_refined_degree_41_endpoint'];results=[]
for index,name in enumerate(names):
 log=audit/(prefix+f'-case-{index}.log');start=time.monotonic()
 with log.open('w') as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
  except subprocess.TimeoutExpired:code='timeout'
 row=dict(name=name,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name);results.append(row);print(row,log.read_text()[-1200:],flush=True)
verify();result=dict(results=results,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-terminal.json')).write_text(json.dumps(result,indent=2)+'\n');print('Terminal; parent probes reaped.',flush=True)
