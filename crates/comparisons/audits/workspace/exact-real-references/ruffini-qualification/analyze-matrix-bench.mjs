import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const median=a=>{const b=[...a].sort((x,y)=>x-y),n=b.length;return n%2?b[n>>1]:(b[n/2-1]+b[n/2])/2;};
let state=0x2f790301;const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return (state>>>0)/4294967296;};
function interval(a) {const samples=[];for(let i=0;i<20000;i++)samples.push(median(a.map(()=>a[Math.floor(random()*a.length)])));samples.sort((x,y)=>x-y);return[samples[500],samples[19499]];}
const runs=JSON.parse(readFileSync(root+'/matrix-bench-runs.json','utf8'));assert.equal(runs.length,6);
let observations=0,measured=0,verifiedMeasuredProducts=0;const families=[];
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/MatrixBench.java'),r.sourceSha256);assert.equal(hash(build+'/matrix/MatrixBench.class'),r.classSha256);
 assert.deepEqual(r.args.slice(0,3),['-c','6','java']);
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
 const calibration=rows.filter(r=>r[0]==='CALIBRATE');assert.equal(calibration.length,3);const data=rows.filter(r=>r[0]==='BENCH');assert.equal(data.length,36);observations+=36;
 const byRound=new Map();for(const [kind,n,bits,round,position,name,loops,cpu,wall]of data) {
  assert.equal(+n,r.n);assert.equal(+bits,r.bits);assert(+cpu>0&&+wall>0);assert.equal(+position,( ['naive','strassen1','strassen8'].indexOf(name)-Number(round)+30)%3);
  if(+round<0)continue;measured++;verifiedMeasuredProducts+=+loops;
  if(!byRound.has(+round))byRound.set(+round,{});assert(!byRound.get(+round)[name]);byRound.get(+round)[name]={cpu:Number(cpu)/Number(loops),wall:Number(wall)/Number(loops),loops:+loops};
 }
 assert.equal(byRound.size,9);
 const row={n:r.n,bits:r.bits,cpuNsPerProduct:{},comparisons:{}};
 for(const name of ['naive','strassen1','strassen8'])row.cpuNsPerProduct[name]=median([...byRound.values()].map(r=>r[name].cpu));
 for(const name of ['strassen1','strassen8']) {const ratios=[...byRound.values()].map(r=>r[name].cpu/r.naive.cpu),wall=[...byRound.values()].map(r=>r[name].wall/r.naive.wall);row.comparisons[name]={cpuRatioMedian:median(ratios),cpuRatioBootstrap95:interval(ratios),wallRatioMedian:median(wall),pairs:9};}
 families.push(row);
}
assert.equal(measured,162);assert.equal(observations,216);
const result={observations,measured,verifiedMeasuredProducts,families,cpuAffinity:6,commonPoolParallelism:1,warmupRoundsPerFamily:3,measuredRoundsPerFamily:9,scope:'donor-only prepared exact BigInteger matrix products plus full result verification; not Hyper speedups',newProductionChanges:0};
writeFileSync(root+'/matrix-bench-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
