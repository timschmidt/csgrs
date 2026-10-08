from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent
r=json.loads((A/'rational-resultant-samples-20260928-v680-terminal.json').read_text())
binary=next(b for b in r['builds'] if 'hypercurve' in b.get('binaries',{}))['binaries']['hypercurve']
assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
pids=subprocess.check_output(['pgrep','-f','^'+binary['path']+' '],text=True).split()
assert len(pids)==1,pids
pid=int(pids[0]);proc=Path('/proc')/str(pid)
argv=proc.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
assert argv[0]==binary['path'] and argv[1]=='--exact'
assert argv[2] in r['expected_cases']['hypercurve'] and 'structural_overlap_trace_regression' in argv[2]
assert proc.joinpath('cwd').resolve()==Path(r['source_directory'])/'hypercurve'
original=None
for line in (A/next(b for b in r['builds'] if 'hypercurve' in b.get('binaries',{}))['log']).read_text().splitlines():
 try:message=json.loads(line)
 except ValueError:continue
 if message.get('reason')=='compiler-artifact' and message.get('target',{}).get('name')=='hypercurve' and message.get('executable'):original=message['executable']
assert original and hashlib.sha256(Path(original).read_bytes()).hexdigest()==binary['sha256']
command=['gdb','-q','-batch','-ex','set debuginfod enabled off','-ex','set pagination off','-ex','set sysroot /proc/'+str(pid)+'/root','-ex','set print frame-arguments none','-ex','file '+original,'-ex','attach '+str(pid),'-ex','thread apply all bt 28','-ex','detach','-ex','quit']
log=A/'rational-resultant-samples-v683-gdb.log'
with log.open('w')as out:code=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=25).returncode
(A/'rational-resultant-samples-v683-gdb.json').write_text(json.dumps(dict(pid=pid,argv=argv,cwd=str(proc.joinpath('cwd').resolve()),binary_sha256=binary['sha256'],original_executable=original,returncode=code,log=log.name),indent=2)+'\n')
print(log.read_text()[-22000:])
