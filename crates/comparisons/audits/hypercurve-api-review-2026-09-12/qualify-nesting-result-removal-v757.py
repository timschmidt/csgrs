from pathlib import Path
import hashlib,json,os,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='nesting-result-removal-v757';prior_prefix='nesting-result-removal-v755';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
reaped=json.loads((A/f'{prior_prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text());assert prior['qualification_complete']and prior['all_processes_reaped']and all(c['passed']for c in prior['cases'])
old=json.loads((A/prior['source_manifest']).read_text());fix=json.loads((A/'nesting-helper-comment-fix-v756.json').read_text());name=fix['path']
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for n,sha in old.items():
 for root in [W,Path(prior['source_directory']),build]:assert digest(root/n)==sha,(root,n)
assert old[name]==fix['base_sha256']
for block in [fix['old'],fix['new']]:assert all(line.strip().startswith('///')for line in block.splitlines())
before=(W/name).read_text();assert before.count(fix['old'])==1;after=before.replace(fix['old'],fix['new']);assert before!=after
candidate=A/'nesting-helper-comment-candidate-v756'/name;candidate.parent.mkdir(parents=True,exist_ok=True);candidate.write_text(after);(W/name).write_text(after)
manifest={};assert not archive.exists()
for n in old:
 data=(W/n).read_bytes();manifest[n]=hashlib.sha256(data).hexdigest();p=archive/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
 if (build/n).read_bytes()!=data:(build/n).write_bytes(data);os.utime(build/n,None)
assert {n for n in manifest if manifest[n]!=old[n]}=={name}
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
promotion=json.loads((A/'nesting-result-removal-promotion-v755.json').read_text());promotion['candidates'][name]=str(candidate);promotion['promoted'][name]=manifest[name];promotion['comment_fix']='nesting-helper-comment-fix-v756.json';(A/'nesting-result-removal-promotion-v757.json').write_text(json.dumps(promotion,indent=2)+'\n')
report=json.loads(json.dumps(prior));report.update(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),reused_builds_from=prior_prefix,source_change_proof=dict(fix='nesting-helper-comment-fix-v756.json',before=old[name],after=manifest[name],only_comments_changed=True),all_processes_reaped=False,qualification_complete=False)
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',str(W/name)];log=A/f'{prefix}-comment-format.log'
with log.open('w')as output:result=subprocess.run(command,stdout=output,stderr=subprocess.STDOUT,timeout=30)
report['checks'].append(dict(label='comment-format',command=command,returncode=result.returncode,log=log.name))
for n,sha in manifest.items():
 for root in [W,archive,build]:assert digest(root/n)==sha,(root,n)
report['all_processes_reaped']=True;report['qualification_complete']=result.returncode==0;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Verified exact private-comment-only delta; reused73passing release cases/sevenchecks, final formatter exit',result.returncode)
raise SystemExit(result.returncode)
