from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;C=A/'image-ownership-candidate-v917'
r=json.loads((A/'image-ownership-candidate-base-v917.json').read_text());baseline=json.loads((A/'affine-field-v910-sources.json').read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for n,old in r['parents'].items():
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/n,text=True).strip()==old['head'],n
 assert not subprocess.check_output(['git','status','--porcelain=v1'],cwd=W/n,text=True),n
for n,sha in baseline.items():assert digest(W/n)==sha,n
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',*[str(C/n)for n in sorted(r['base'])]],check=True)
promoted={}
for n in r['base']:
 assert digest(W/n)==r['base'][n] and digest(C/n)!=r['base'][n],n
 (W/n).write_bytes((C/n).read_bytes());promoted[n]=digest(W/n)
(A/'image-ownership-promotion-v917.json').write_text(json.dumps(dict(candidates=r['candidates'],base=r['base'],promoted=promoted,parents=r['parents']),indent=2)+'\n')
print('Formatted and promoted',len(promoted),'Hypersolve paths')
