import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname;
const read=n=>readFileSync(resolve(dir,n),'utf8');
const inv=read('../PLUME_FILE_INVENTORY.tsv').trim().split('\n').slice(1).map(x=>x.split('\t'));
for(const [path,kind,bytes,lines,hash] of inv) {
  const data=readFileSync(resolve(dir,'../Plume',path));
  if(data.length!==Number(bytes)||createHash('sha256').update(data).digest('hex')!==hash) throw Error(`source changed: ${path}`);
}
const runs=JSON.parse(read('bench-runs.json'));
if(runs.length!==216||runs.some(r=>r.status!==0||r.error)) throw Error('incomplete/error benchmark run');
const rows=read('bench-results.tsv').trim().split('\n').slice(1).map(x=>x.split('\t'));
if(rows.length!==216||new Set(rows.map(r=>r.slice(0,4).join('/'))).size!==216) throw Error('duplicate/missing benchmark rows');
if(read('KernelProbe.hs')!==read('KernelProbe.bench-final.hs')) throw Error('benchmark source changed');
const median=a=>{a.sort((x,y)=>x-y);return (a[3]+a[4])/2};
const summary=[];
for(const family of ['dense','sparse','terminating']) for(const bits of [32,64,128,256]) {
  const stats=['signed','dyadic'].map(op=>{
    const r=rows.filter(x=>x[0]!=='0'&&x[1]===op&&x[2]===family&&+x[3]===bits);
    if(r.length!==8||r.some(x=>x[8]!=='PASS')) throw Error('invalid final benchmark group');
    const cpu=r.map(x=>Number(x[5])/1e9),alloc=r.map(x=>Number(x[6]));
    return {op,cpu_ms:median(cpu),cpu_min_ms:Math.min(...cpu),cpu_max_ms:Math.max(...cpu),allocated_bytes:median(alloc)};
  });
  summary.push({family,bits,signed:stats[0],dyadic:stats[1],time_ratio:stats[1].cpu_ms/stats[0].cpu_ms,allocation_ratio:stats[1].allocated_bytes/stats[0].allocated_bytes});
}
const requests=JSON.parse(read('elementary-runs.json'));
if(requests.length!==129||requests.some(x=>x.error==='EPERM')) throw Error('invalid final elementary run');
const functional=read('functional-results.tsv').trim().split('\n');
if(functional.length!==270) throw Error('incomplete functional run');
const report={
  source_files_verified:inv.length,
  benchmark_observations:216,warmup_observations:24,final_observations:192,
  benchmark:summary,
  elementary:{requests:requests.length,finite:requests.filter(x=>x.status===0).length,heap_limits:requests.filter(x=>x.stderr.includes('Heap exhausted')).length,timeouts:requests.filter(x=>x.error==='ETIMEDOUT').length},
  functional:{requests:270,passed:functional.filter(x=>x.includes('Just True')).length,failed:functional.filter(x=>x.includes('Just False')).length,timeouts:functional.filter(x=>x.includes('Right Nothing')).length},
};
writeFileSync(resolve(dir,'qualification-summary.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
