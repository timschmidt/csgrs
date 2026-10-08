import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,workspace=resolve(root,'../../..');
const assert=(ok,message)=>{if(!ok)throw Error(message);};
const run=process.argv[2]??'paired-v1',stage=process.argv[3]??'prototype';assert(/^[a-z0-9-]+$/.test(run)&&['prototype','retained'].includes(stage),'run or stage');
const binaries={before:['hyper-field-bench-release','25ed91a0df3f22d1f4b363b72381d4774f7c8c331c3aa3b0acc0e4d695fbb00c'],after:[`fractional-bench-${stage}-release`,stage==='prototype'?'939975fa628c1075ba3314b35d62a205442754a3d56474d8f8185eb63a94fc3a':'be460e417fd5b44c013c5e8198415c3dc84dd2e9e9a1e7dffb96aa418c12ed04']};
for(const [path,sha] of Object.values(binaries))assert(createHash('sha256').update(readFileSync(workspace+'/.audit-constructible-build.l4UDoe/'+path)).digest('hex')===sha,'binary changed');
const rows=readFileSync(root+'/'+run+'-observations.jsonl','utf8').trimEnd().split('\n').map(JSON.parse);
assert(rows.length===162,'observation count');
const families=[['quadratic',64,1168],['tower-1',1600,28],['tower-2',600,28],['tower-3',300,28],['tower-4',160,28],['tower-5',96,28],['independent',2400,16],['transverse-20',32,940],['transverse-60',32,940]];
const median=xs=>{xs=xs.slice().sort((a,b)=>a-b);const n=xs.length;return n%2?xs[n>>1]:(xs[n/2-1]+xs[n/2])/2;};
let seed=0x5271bc39;const rnd=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return(seed>>>0)/2**32;};
function ci(xs){const bs=Array.from({length:20000},()=>median(xs.map(()=>xs[Math.floor(rnd()*xs.length)]))).sort((a,b)=>a-b);return[bs[499],bs[19499]];}
const result=[];
for(const [group,rounds,count] of families){
 const pairs=[],samples={before:[],after:[]};
 for(let sample=0;sample<=8;sample++){
  const pair={};
  for(const mode of ['before','after']){
   const found=rows.filter(x=>x.group===group&&x.sample===sample&&x.mode===mode);assert(found.length===1,'missing/duplicate observation');const x=found[0];
   assert(x.status===0&&x.rounds===rounds&&x.checks===rounds*count&&x.cpu_ns>0&&x.wall_ns>0&&x.rss_kib>0&&x.binary_sha256===binaries[mode][1],'observation contract');
   const prefix=`${run}-${sample}-${group}-${mode}`;
   assert(readFileSync(root+'/'+prefix+'.stdout','utf8')===`BENCH\t${rounds}\t${rounds*count}\t${x.cpu_ns}\t${x.wall_ns}\n`,'output mismatch');
   assert(readFileSync(root+'/'+prefix+'.stderr','utf8').includes('AUDIT_RSS_KIB '+x.rss_kib),'RSS mismatch');
   pair[mode]=x;if(sample>0)samples[mode].push(x);
  }
  if(sample>0)pairs.push(pair);
 }
 const cpu=pairs.map(x=>x.after.cpu_ns/x.before.cpu_ns),wall=pairs.map(x=>x.after.wall_ns/x.before.wall_ns);
 result.push({group,pairs:8,checksPerObservation:rounds*count,afterOverBeforeCpu:median(cpu),cpuBootstrap95:ci(cpu),afterOverBeforeWall:median(wall),wallBootstrap95:ci(wall),beforeCpuMs:median(samples.before.map(x=>x.cpu_ns/1e6)),afterCpuMs:median(samples.after.map(x=>x.cpu_ns/1e6)),beforeRssKiB:median(samples.before.map(x=>x.rss_kib)),afterRssKiB:median(samples.after.map(x=>x.rss_kib))});
}
const report={observations:rows.length,postWarmup:rows.filter(x=>x.sample>0).length,verifiedComparisons:rows.reduce((n,x)=>n+x.checks,0),result,binaryBytes:Object.fromEntries(Object.entries(binaries).map(([mode,[path]])=>[mode,statSync(workspace+'/.audit-constructible-build.l4UDoe/'+path).size]))};
writeFileSync(root+'/'+(run==='paired-v1'?'bench-analysis':run+'-analysis')+'.json',JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report,null,2));
