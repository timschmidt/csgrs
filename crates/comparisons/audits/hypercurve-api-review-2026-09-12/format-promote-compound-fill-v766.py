from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;C=A/'compound-fill-candidate-v765'
base=json.loads((A/'compound-fill-base-v765.json').read_text());baseline=json.loads((A/'region-admission-v763-sources.json').read_text())
parents=json.loads((A/'region-admission-v763-repositories-after.json').read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,old in parents.items():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()==old['head'],name
 assert not subprocess.check_output(['git','status','--porcelain=v1'],cwd=W/name,text=True).strip(),name
for name,sha in baseline.items():assert digest(W/name)==sha,name
rustfmt='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
for repo in sorted({n.split('/')[0]for n in base}):
 paths=[str(C/n)for n in sorted(base)if n.startswith(repo+'/')and n.endswith('.rs')]
 args=[rustfmt,'--edition','2024','--config','skip_children=true']
 if repo=='csgrs':args+=['--config-path',str(W/repo/'.rustfmt.toml')]
 subprocess.run([*args,*paths],cwd=W/repo,check=True)
for name,sha in base.items():assert digest(W/name)==sha,name
promoted={}
for name in base:
 data=(C/name).read_bytes();assert digest(C/name)!=base[name],name
 (W/name).write_bytes(data);promoted[name]=digest(W/name)
(A/'compound-fill-promotion-v766.json').write_text(json.dumps(dict(candidates={n:str(C/n)for n in base},base=base,promoted=promoted,parents=parents),indent=2)+'\n')
print('Formatted and promoted',len(promoted),'paths across',len({n.split('/')[0]for n in base}),'repos')
