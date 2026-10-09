import hashlib,json,subprocess,sys,tarfile,time
from pathlib import Path
out=Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=False)
paths=sorted([p for name in ['hyperreal','hyperlimit','hypercurve','hyperdrc'] for p in Path('crates',name).rglob('*') if p.is_file() and 'target' not in p.parts and (p.suffix=='.rs' or p.name in ['Cargo.toml','Cargo.lock'])]+[Path('Cargo.toml'),Path('Cargo.lock')])
def hashes(): return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
initial=hashes()
with tarfile.open(out/'sources.tar.gz','w:gz') as archive:
    for p in paths: archive.add(p,arcname=str(p))
(out/'sources.json').write_text(json.dumps({'source_hashes':initial,'unchanged':True},indent=2)+'\n')
commands=[('focused-circle-contact-oracle', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'retained_circle_line_contacts', '--', '--test-threads=1'])]
status=[]
for name,command in commands:
    assert hashes()==initial
    started=time.monotonic()
    with (out/(name+'.log')).open('w') as log:
        code=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT).returncode
    status.append({'name':name,'command':command,'exit_code':code,'elapsed_seconds':time.monotonic()-started})
    (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
    (out/'sources.json').write_text(json.dumps({'source_hashes':initial,'unchanged':initial==hashes()},indent=2)+'\n')
    assert initial==hashes()
    print(out,name,code,flush=True)
    if code: break
sys.exit(code)
