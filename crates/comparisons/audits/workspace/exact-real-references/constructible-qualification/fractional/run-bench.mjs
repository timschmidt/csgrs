import {readFileSync,openSync,writeSync,closeSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {spawnSync} from 'node:child_process';
import {resolve} from 'node:path';
const root=import.meta.dirname,workspace=resolve(root,'../../..');
const stage=process.argv[3]??'prototype';if(!['prototype','retained'].includes(stage))throw Error('stage');
const bins={before:resolve(workspace,'.audit-constructible-build.l4UDoe/hyper-field-bench-release'),after:resolve(workspace,`.audit-constructible-build.l4UDoe/fractional-bench-${stage}-release`)};
const hashes={before:'25ed91a0df3f22d1f4b363b72381d4774f7c8c331c3aa3b0acc0e4d695fbb00c',after:stage==='prototype'?'939975fa628c1075ba3314b35d62a205442754a3d56474d8f8185eb63a94fc3a':'be460e417fd5b44c013c5e8198415c3dc84dd2e9e9a1e7dffb96aa418c12ed04'};
for(const mode of Object.keys(bins))if(createHash('sha256').update(readFileSync(bins[mode])).digest('hex')!==hashes[mode])throw Error('binary changed '+mode);
const families=[['quadratic',64,1168],['tower-1',1600,28],['tower-2',600,28],['tower-3',300,28],['tower-4',160,28],['tower-5',96,28],['independent',2400,16],['transverse-20',32,940],['transverse-60',32,940]];
const run=process.argv[2];if(!run||!/^[a-z0-9-]+$/.test(run))throw Error('new run name required');
const log=openSync(root+'/'+run+'-observations.jsonl','wx');
try{
 for(let sample=0;sample<=8;sample++)for(const [group,rounds,rows] of families){
  for(const mode of sample%2===0?['before','after']:['after','before']){
   const path=group.startsWith('transverse')?root+'/'+group+'.tsv':root+'/../field-'+group+'.tsv';
   const result=spawnSync('timeout',['60s','taskset','-c','6','/usr/bin/time','-f','AUDIT_RSS_KIB %M',bins[mode],path,String(rounds),'-16384'],{cwd:workspace,encoding:'utf8',timeout:65000,maxBuffer:1048576});
   const prefix=`${run}-${sample}-${group}-${mode}`;
   writeFileSync(root+'/'+prefix+'.stdout',result.stdout??'',{flag:'wx'});writeFileSync(root+'/'+prefix+'.stderr',result.stderr??'',{flag:'wx'});
   if(result.error||result.signal||result.status!==0)throw Error(JSON.stringify({prefix,status:result.status,signal:result.signal,error:String(result.error),stderr:result.stderr}));
   const m=/^BENCH\t(\d+)\t(\d+)\t(\d+)\t(\d+)\n$/.exec(result.stdout),rss=/AUDIT_RSS_KIB (\d+)/.exec(result.stderr);
   if(!m||+m[1]!==rounds||+m[2]!==rows*rounds||!rss)throw Error('invalid coverage or RSS');
   writeSync(log,JSON.stringify({sample,group,mode,rounds,checks:rows*rounds,cpu_ns:+m[3],wall_ns:+m[4],rss_kib:+rss[1],binary_sha256:hashes[mode],status:0})+'\n');
  }
  console.log('finished',sample,group);
 }
}finally{closeSync(log);}
