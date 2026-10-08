import {readFileSync,openSync,writeSync,closeSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
const root=import.meta.dirname,workspace=resolve(root,'../..');
const bins={native:resolve(workspace,'.audit-constructible-build.l4UDoe/field-bench-O2'),hyper:resolve(workspace,'.audit-constructible-build.l4UDoe/hyper-field-bench-release')};
const hashes={native:'dec74b91ccfd8725302f57367ea52414f20550c8a860c16b0cac746ca1eabc20',hyper:'25ed91a0df3f22d1f4b363b72381d4774f7c8c331c3aa3b0acc0e4d695fbb00c'};
for(const mode of Object.keys(bins))if(createHash('sha256').update(readFileSync(bins[mode])).digest('hex')!==hashes[mode])throw Error('binary changed '+mode);
const families=[['quadratic',16,1168],['tower-1',400,28],['tower-2',300,28],['tower-3',200,28],['tower-4',100,28],['tower-5',64,28],['independent',800,16]];
const run=process.argv[2]??'field-bench';
if(!/^[a-z0-9-]+$/.test(run))throw Error('run name');
const log=openSync(root+'/'+run+'-observations.jsonl','wx');
try{
  // Round zero is a complete discarded warm-up. Eight measured pairs alternate
  // order on one fixed CPU; each process constructs fresh inputs every round.
  for(let sample=0;sample<=8;sample++)for(const [group,rounds,rows] of families){
    for(const mode of sample%2===0?['native','hyper']:['hyper','native']){
      const path=root+'/field-'+group+'.tsv';
      const args=[bins[mode],path,String(rounds),...(mode==='hyper'?['-16384']:['+RTS','-s'])];
      const result=spawnSync('timeout',['60s','taskset','-c','6','/usr/bin/time','-f','AUDIT_RSS_KIB %M',...args],{cwd:workspace,encoding:'utf8',timeout:65000,maxBuffer:1048576});
      const prefix=`${run}-${sample}-${group}-${mode}`;
      writeFileSync(root+'/'+prefix+'.stdout',result.stdout??'',{flag:'wx'});
      writeFileSync(root+'/'+prefix+'.stderr',result.stderr??'',{flag:'wx'});
      if(result.error||result.signal||result.status!==0)throw Error(JSON.stringify({prefix,status:result.status,signal:result.signal,error:String(result.error),stderr:result.stderr}));
      const match=/^BENCH\t(\d+)\t(\d+)\t(\d+)\t(\d+)\n$/.exec(result.stdout);
      if(!match||+match[1]!==rounds||+match[2]!==rows*rounds)throw Error('incorrect benchmark coverage');
      const rss=/AUDIT_RSS_KIB (\d+)/.exec(result.stderr);if(!rss)throw Error('RSS absent');
      const allocated=/([\d,]+) bytes allocated in the heap/.exec(result.stderr);
      const record={sample,group,mode,rounds,checks:rows*rounds,cpu_ns:+match[3],wall_ns:+match[4],rss_kib:+rss[1],native_heap_bytes:allocated?Number(allocated[1].replaceAll(',','')):null,binary_sha256:hashes[mode],status:0};
      writeSync(log,JSON.stringify(record)+'\n');
    }
    console.log('finished',sample,group);
  }
}finally{closeSync(log);}
