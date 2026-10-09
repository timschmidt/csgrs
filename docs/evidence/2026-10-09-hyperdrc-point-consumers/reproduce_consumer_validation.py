import hashlib,json,os,pathlib,subprocess,time,sys
out=pathlib.Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
paths=[p for crate in ['hyperdrc','hypercircuit','hyperreal'] for p in pathlib.Path('crates',crate).rglob('*.rs')]
sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
commands=[('fmt',['cargo','fmt','--all','--','--check']),('consumer-clippy',['cargo','clippy','--locked','-p','hypercircuit','--all-features','--all-targets','--','-D','warnings']),('consumer-drc',['cargo','test','--locked','-p','hypercircuit','--all-features','--release','hyperdrc']),('consumer-curves',['cargo','test','--locked','-p','hypercircuit','--all-features','--release','--test','curved_outline'])]
status=[]
for name,cmd in commands:
 started=time.time()
 with (out/(name+'.log')).open('w') as log:code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT).returncode
 status.append({'name':name,'command':cmd,'exit_code':code,'elapsed_seconds':time.time()-started})
 (out/'status.json').write_text(json.dumps(status,indent=2)+'\n');print(name,code,flush=True)
 if code:break
current={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(out/'sources.json').write_text(json.dumps({'source_hashes':sources,'unchanged':sources==current},indent=2)+'\n')
raise SystemExit(status[-1]['exit_code'])
