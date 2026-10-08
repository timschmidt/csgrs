from pathlib import Path
import json,os,signal,sys
root=Path('/home/tim/Documents/GitHub/workspace/hypercurve')
expected=['test','--release','--all-features','--no-fail-fast','--lib','--tests','--']
clock_ticks=os.sysconf('SC_CLK_TCK')
uptime=float(Path('/proc/uptime').read_text().split()[0])
records=[]
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit():continue
 try:
  args=entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
  if not args or Path(args[0]).name!='cargo' or args[1:8]!=expected:continue
  if Path(os.readlink(entry/'cwd'))!=root or entry.stat().st_uid!=os.getuid():continue
  stat=(entry/'stat').read_text().rsplit(')',1)[1].split()
  children=[]
  for child in (entry/'task'/entry.name/'children').read_text().split():
   cp=Path('/proc')/child
   try:
    ca=(cp/'cmdline').read_bytes().decode().strip('\0').split('\0')
    cs=(cp/'stat').read_text().rsplit(')',1)[1].split()
    children.append({'pid':int(child),'args':ca,'start_ticks':cs[19],'elapsed_seconds':uptime-int(cs[19])/clock_ticks})
   except (FileNotFoundError,ProcessLookupError):pass
  records.append({'pid':int(entry.name),'args':args,'start_ticks':stat[19],'children':children})
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
assert len(records)==1,records
record=records[0]
if len(sys.argv)==2:
 limit=float(sys.argv[1]); assert limit>=300
 for child in record['children']:
  if child['elapsed_seconds']<limit:continue
  assert Path(child['args'][0]).is_relative_to(root/'target/release/deps'),child
  assert Path(child['args'][0]).name.startswith(('hypercurve_analytic_parallel_region-','hypercurve_pcb_boolean_regressions-')),child
  cp=Path('/proc')/str(child['pid'])
  current=(cp/'stat').read_text().rsplit(')',1)[1].split()
  assert current[19]==child['start_ticks'] and int(current[1])==record['pid']
  os.kill(child['pid'],signal.SIGTERM)
  child['termination']='SIGTERM after verified 300-second target budget; parent cargo retains --no-fail-fast'
with Path('box-policy-full-test-process-history.jsonl').open('a') as history:history.write(json.dumps(record)+'\n')
Path('box-policy-full-test-process.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
