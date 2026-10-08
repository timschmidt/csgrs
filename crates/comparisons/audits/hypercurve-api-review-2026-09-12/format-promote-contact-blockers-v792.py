from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;C=A/'contact-blockers-candidate-v792'
base=json.loads((A/'contact-blockers-base-v792.json').read_text());baseline=json.loads((A/'contact-blockers-v791-sources.json').read_text())
parents=json.loads((A/'point-definition-v790-repositories-after.json').read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,old in parents.items():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in base if p.startswith(name+'/'))
 assert subprocess.check_output(['git','status','--porcelain=v1'],cwd=W/name,text=True)==''.join(' M '+p+'\n'for p in paths),name
for name,sha in baseline.items():assert digest(W/name)==sha,name
assert json.loads((A/'contact-blockers-v791-reaped.json').read_text())['outer_exit_code']==1
rustfmt='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
for repo in sorted({n.split('/')[0]for n in base}):
 paths=[str(C/n)for n in sorted(base)if n.startswith(repo+'/')and n.endswith('.rs')]
 args=[rustfmt,'--edition','2024','--config','skip_children=true']
 if repo=='csgrs':args+=['--config-path',str(W/repo/'.rustfmt.toml')]
 subprocess.run([*args,*paths],cwd=W/repo,check=True)
for name,sha in baseline.items():assert digest(W/name)==sha,name
promoted={}
for name in base:
 data=(C/name).read_bytes();assert digest(C/name)!=base[name],name
 (W/name).write_bytes(data);promoted[name]=digest(W/name)
(A/'contact-blockers-promotion-v792.json').write_text(json.dumps(dict(candidates={n:str(C/n)for n in base},base=base,promoted=promoted,parents=parents),indent=2)+'\n')
print('Formatted and promoted',len(promoted),'paths across',len({n.split('/')[0]for n in base}),'repos')
