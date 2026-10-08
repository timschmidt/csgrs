import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,build=resolve(root,'../../.audit-ruffini-build.LmZgYM');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const median=a=>{const b=[...a].sort((x,y)=>x-y);return b[b.length>>1];};
let state=0x4508a72b;const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return(state>>>0)/4294967296;};
function interval(a){const s=[];for(let i=0;i<20000;i++)s.push(median(a.map(()=>a[Math.floor(random()*a.length)])));s.sort((a,b)=>a-b);return[s[500],s[19499]];}
const first=JSON.parse(readFileSync(root+'/polynomial-bench-runs.json','utf8'));assert.equal(first.length,2);
for(const r of first){assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/PolynomialBench.java'),r.sourceSha256);assert.equal(hash(build+'/polynomial/PolynomialBench.class'),r.classSha256);assert.equal(hash(root+'/PolynomialContracts.java'),r.oracleSourceSha256);assert.equal(r.error,null);assert.equal(r.signal,null);}
assert.equal(first[0].status,0);assert.equal(first[1].status,1);assert.equal(first[1].density,'sparse');assert(readFileSync(root+'/'+first[1].file+'.stderr','utf8').includes('AssertionError: calibration cap'));assert(!readFileSync(root+'/'+first[1].file,'utf8').includes('BENCH\t'));
const runs=JSON.parse(readFileSync(root+'/polynomial-bench-extended-runs.json','utf8'));assert.equal(runs.length,12);
assert.deepEqual(runs.map(r=>[r.n,r.bits,r.density]),[16,64,128].flatMap(n=>[32,2048].flatMap(b=>['dense','sparse'].map(d=>[n,b,d]))));
const families=[];let measured=0,observations=0,verifiedMeasuredProducts=0,minMeasuredCpuNs=Infinity;
for(const r of runs) {
 assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);assert.equal(hash(root+'/'+r.file),r.sha256);
 assert.equal(hash(root+'/PolynomialBenchExtended.java'),r.sourceSha256);assert.equal(hash(build+'/polynomial/PolynomialBenchExtended.class'),r.classSha256);assert.equal(hash(root+'/PolynomialBench.java'),r.baseSourceSha256);assert.equal(hash(build+'/polynomial/PolynomialBench.class'),r.baseClassSha256);assert.equal(hash(root+'/PolynomialContracts.java'),r.oracleSourceSha256);
 assert.equal(readFileSync(root+'/'+r.file+'.stderr','utf8'),'');assert.deepEqual(r.args.slice(0,3),['-c','6','java']);assert(r.args.includes('-Djava.util.concurrent.ForkJoinPool.common.parallelism=1'));
 const rows=readFileSync(root+'/'+r.file,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));assert.equal(rows.length,28);
 const names=['ordinary','karatsuba'],warm=rows.filter(r=>r[0]==='WARMUP'),cal=rows.filter(r=>r[0]==='CALIBRATE');assert.deepEqual(warm.map(r=>r[1]),names);assert.deepEqual(cal.map(r=>r[1]),names);
 for(const row of [...warm,...cal])assert(+row[2]>0&&+row[3]>=(row[0]==='WARMUP'?500_000_000:1_000_000_000));
 const loopsByName=Object.fromEntries(cal.map(r=>[r[1],+r[2]])),byRound=new Map(),seen=new Set();
 const data=rows.filter(r=>r[0]==='BENCH');assert.equal(data.length,24);observations+=24;
 for(const[kind,round,position,name,loops,cpu,wall]of data) {
  assert(Number.isInteger(+round)&&+round>=-3&&+round<=8);assert(names.includes(name));assert.equal(+position,(names.indexOf(name)-Number(round)+20)%2);assert.equal(+loops,loopsByName[name]);assert(+cpu>0&&+wall>0);assert(!seen.has(round+'/'+name));seen.add(round+'/'+name);
  if(+round<0)continue;assert(+cpu>=500_000_000);minMeasuredCpuNs=Math.min(minMeasuredCpuNs,+cpu);measured++;verifiedMeasuredProducts+=+loops;
  if(!byRound.has(+round))byRound.set(+round,{});byRound.get(+round)[name]={cpu:+cpu/+loops,wall:+wall/+loops};
 }
 assert.equal(byRound.size,9);assert.equal(seen.size,24);
 const cpuRatios=[...byRound.values()].map(v=>v.karatsuba.cpu/v.ordinary.cpu),wallRatios=[...byRound.values()].map(v=>v.karatsuba.wall/v.ordinary.wall);
 families.push({n:r.n,bits:r.bits,density:r.density,loopsByName,ordinaryCpuNsPerProduct:median([...byRound.values()].map(v=>v.ordinary.cpu)),karatsubaCpuRatio:median(cpuRatios),cpuRatioBootstrap95:interval(cpuRatios),karatsubaWallRatio:median(wallRatios),wallRatioBootstrap95:interval(wallRatios),cpuRatios,wallRatios});
}
assert.equal(observations,288);assert.equal(measured,216);
const result={observations,measured,pairedRounds:108,verifiedMeasuredProducts,minMeasuredCpuNs,families,scope:'donor-only prepared polynomial products with exact full coefficient verification; not Hyper speedups',limitations:'One shared-host JVM per family, rotated within-process pairs, descriptive bootstrap not independent-process reproducibility; CPU6 affinity is not exclusive reservation; approximately10ms process CPU timer. Initial calibration-cap run preserved separately.',newProductionChanges:0};
writeFileSync(root+'/polynomial-bench-analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({...result,families:families.map(({cpuRatios,wallRatios,...r})=>r)},null,2));
