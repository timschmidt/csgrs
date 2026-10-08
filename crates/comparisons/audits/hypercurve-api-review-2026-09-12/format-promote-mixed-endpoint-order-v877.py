from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;C=A/'mixed-endpoint-order-candidate-v877'
record=json.loads((A/'mixed-endpoint-order-candidate-base-v877.json').read_text());base=record['base'];baseline=json.loads((A/'polynomial-zero-jet-v874-sources.json').read_text())
parents=record['parents']
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,old in parents.items():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in base if p.startswith(name+'/'))
 assert subprocess.check_output(['git','status','--porcelain=v1'],cwd=W/name,text=True)=='',name
for name,sha in baseline.items():assert digest(W/name)==sha,name
rustfmt='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
for repo in sorted({n.split('/')[0]for n in base}):
 paths=[str(C/n)for n in sorted(base)if n.startswith(repo+'/')and n.endswith('.rs')]
 args=[rustfmt,'--edition','2024','--config','skip_children=true']
 if repo=='csgrs':args+=['--config-path',str(W/repo/'.rustfmt.toml')]
 subprocess.run([*args,*paths],cwd=W/repo,check=True)
for name in base:assert digest(W/name)==baseline[name],name
promoted={}
for name in base:
 data=(C/name).read_bytes();assert digest(C/name)!=base[name],name
 (W/name).write_bytes(data);promoted[name]=digest(W/name)
(A/'mixed-endpoint-order-promotion-v877.json').write_text(json.dumps(dict(candidates={n:str(C/n)for n in base},base=base,promoted=promoted,parents=parents),indent=2)+'\n')
print('Formatted and promoted',len(promoted),'paths across',len({n.split('/')[0]for n in base}),'repos')
