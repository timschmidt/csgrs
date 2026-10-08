import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const names=['naive','strassen1','strassen8'];
const median=a=>{const b=[...a].sort((x,y)=>x-y),n=b.length;return n%2?b[n>>1]:(b[n/2-1]+b[n/2])/2;};
let state=0x2f790301;const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return(state>>>0)/4294967296;};
function interval(a) {const samples=[];for(let i=0;i<20000;i++)samples.push(median(a.map(()=>a[Math.floor(random()*a.length)])));samples.sort((x,y)=>x-y);return[samples[500],samples[19499]];}
const runs=JSON.parse(readFileSync(root+'/matrix-bench-stable-runs.json','utf8'));assert.equal(runs.length,6);
assert.deepEqual(runs.map(r=>[r.n,r.bits]),[4,8,16].flatMap(n=>[32,256].map(bits=>[n,bits])));
let observations=0,measured=0,verifiedMeasuredProducts=0,minMeasuredCpuNs=Infinity;const families=[];
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/MatrixBenchStable.java'),r.sourceSha256);assert.equal(hash(build+'/matrix/MatrixBenchStable.class'),r.classSha256);assert.equal(hash(root+'/RuffiniMatrix.java'),r.oracleSourceSha256);
 assert.deepEqual(r.args.slice(0,3),['-c','6','java']);assert(r.args.includes('-Djava.util.concurrent.ForkJoinPool.common.parallelism=1'));
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));assert.equal(rows.length,42);
 const warmup=rows.filter(r=>r[0]==='WARMUP'),calibration=rows.filter(r=>r[0]==='CALIBRATE');assert.deepEqual(warmup.map(r=>r[3]),names);assert.deepEqual(calibration.map(r=>r[3]),names);
 for(const row of [...warmup,...calibration]) {assert.equal(+row[1],r.n);assert.equal(+row[2],r.bits);assert(+row[4]>0);assert(+row[5]>=(row[0]==='WARMUP'?800_000_000:1_000_000_000));}
 const loopsByName=Object.fromEntries(calibration.map(r=>[r[3],+r[4]]));
 const data=rows.filter(r=>r[0]==='BENCH');assert.equal(data.length,36);observations+=36;
 const allRounds=new Map(),byRound=new Map();
 for(const [kind,n,bits,round,position,name,loops,cpu,wall]of data) {
  assert.equal(+n,r.n);assert.equal(+bits,r.bits);assert(+cpu>0&&+wall>0);assert(names.includes(name));assert.equal(+position,(names.indexOf(name)-Number(round)+30)%3);assert.equal(+loops,loopsByName[name]);
  assert(Number.isInteger(+round)&&+round>=-3&&+round<=8);
  if(!allRounds.has(+round))allRounds.set(+round,new Set());assert(!allRounds.get(+round).has(name));allRounds.get(+round).add(name);
  if(+round<0)continue;
  assert(+cpu>=500_000_000,'short measured sample');minMeasuredCpuNs=Math.min(minMeasuredCpuNs,+cpu);measured++;verifiedMeasuredProducts+=+loops;
  if(!byRound.has(+round))byRound.set(+round,{});byRound.get(+round)[name]={cpu:+cpu/+loops,wall:+wall/+loops,loops:+loops};
 }
 assert.equal(allRounds.size,12);for(const round of allRounds.values())assert.equal(round.size,3);assert.equal(byRound.size,9);
 const row={n:r.n,bits:r.bits,cpuNsPerProduct:{},comparisons:{},loopsByName};
 for(const name of names)row.cpuNsPerProduct[name]=median([...byRound.values()].map(r=>r[name].cpu));
 for(const name of names.slice(1)) {const ratios=[...byRound.values()].map(r=>r[name].cpu/r.naive.cpu),wall=[...byRound.values()].map(r=>r[name].wall/r.naive.wall);row.comparisons[name]={cpuRatioMedian:median(ratios),cpuRatioBootstrap95:interval(ratios),wallRatioMedian:median(wall),wallRatioBootstrap95:interval(wall),pairs:9,cpuRatios:ratios,wallRatios:wall};}
 families.push(row);
}
assert.equal(measured,162);assert.equal(observations,216);
const result={observations,measured,verifiedMeasuredProducts,minMeasuredCpuNs,families,cpuAffinity:6,commonPoolParallelism:1,warmupCpuNsPerAlgorithm:800_000_000,calibrationCpuNsMinimum:1_000_000_000,additionalWarmupRoundsPerFamily:3,measuredRoundsPerFamily:9,scope:'donor-only prepared exact BigInteger matrix products plus full result verification; not Hyper speedups',limitations:'Shared host, no exclusive CPU reservation; one JVM per family, nine paired within-process rounds, descriptive bootstrap intervals are not independent-process reproducibility estimates. Process CPU timer quantized at approximately 10ms. Preliminary MatrixBench logs retained separately.',newProductionChanges:0};
writeFileSync(root+'/matrix-bench-stable-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result,null,2));
