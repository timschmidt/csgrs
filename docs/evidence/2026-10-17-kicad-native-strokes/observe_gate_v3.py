"""Read only this candidate's active test worker CPU counters."""
import hashlib,json,os,time
from pathlib import Path
W=Path(__file__).resolve().parent
PREFIX='/tmp/csgrs-kicad-native-target-v4/release/deps/hyperdrc-'
O=W/'gate-observation-v3'; O.mkdir(exist_ok=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
rows=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=(p/'cmdline').read_bytes().decode().split('\0')
        if not args or not args[0].startswith(PREFIX):continue
        tasks=[]
        for task in (p/'task').iterdir():
            stat=(task/'stat').read_text();tail=stat[stat.rindex(')')+2:].split()
            tasks.append({'tid':int(task.name),'comm':(task/'comm').read_text().strip(),
                'raw_stat':stat,'utime_ticks':int(tail[11]),'stime_ticks':int(tail[12]),
                'cpu_seconds':(int(tail[11])+int(tail[12]))/os.sysconf('SC_CLK_TCK')})
        rows.append({'pid':int(p.name),'args':args,'pgid':os.getpgid(int(p.name)),
            'tasks':tasks,'exe_sha256':sha(args[0]),'status':(p/'status').read_text()})
    except (FileNotFoundError,PermissionError):continue
record={'observed_unix_seconds':time.time(),'runner_sha256':sha(__file__),
    'clock_ticks_per_second':os.sysconf('SC_CLK_TCK'),'read_only':True,'processes':rows,
    'scope':'OS task counters; no assertion about the internal timer start or successful completion.'}
(O/'result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'processes':[{'pid':r['pid'],'tasks':[{k:t[k] for k in ['tid','comm','cpu_seconds']} for t in r['tasks']]} for r in rows]}))
