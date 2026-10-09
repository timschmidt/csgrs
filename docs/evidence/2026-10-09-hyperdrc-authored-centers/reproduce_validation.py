import hashlib,json,os,pathlib,subprocess,time,sys
out=pathlib.Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for crate in ['hyperdrc'] for p in pathlib.Path('crates',crate).rglob('*.rs') if 'target' not in p.parts}
commands=[('fmt',['cargo','fmt','--all','--','--check'],{}),('clippy',['cargo','clippy','--locked','-p','hyperdrc','--all-features','--all-targets','--','-D','warnings'],{}),('test',['cargo','test','--locked','-p','hyperdrc','--release'],{}),('docs',['cargo','doc','--locked','-p','hyperdrc','--all-features','--no-deps'],{'RUSTDOCFLAGS':'-D warnings'}),('fixture-build',['cargo','bench','--locked','-p','hyperdrc','--bench','fixture_smoke','--no-run'],{})]
status=[]
for name,cmd,env in commands:
 started=time.time()
 with (out/(name+'.log')).open('w') as log:
  code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env={**os.environ,**env}).returncode
 status.append({'name':name,'command':cmd,'environment':env,'exit_code':code,'elapsed_seconds':time.time()-started})
 (out/'status.json').write_text(json.dumps(status,indent=2)+'\n')
 print(name,code,flush=True)
 if code:break
current={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for crate in ['hyperdrc'] for p in pathlib.Path('crates',crate).rglob('*.rs') if 'target' not in p.parts}
(out/'sources.json').write_text(json.dumps({'source_hashes':sources,'unchanged':sources==current},indent=2)+'\n')
raise SystemExit(status[-1]['exit_code'])
