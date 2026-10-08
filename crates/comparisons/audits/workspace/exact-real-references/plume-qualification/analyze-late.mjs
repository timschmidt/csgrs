import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const dir=import.meta.dirname;
const read=p=>readFileSync(resolve(dir,p),'utf8');
const json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(dir,p))).digest('hex');
const hashes={
  'LateProbe.hs':'b02ff829139382457ad20cba4906b03f43291090e9cd1017ba1353bd12c8676c',
  'LateBenchmark.hs':'d3ca7d8aa46ba789f1e42a89c584fe5aa0244d67f377034c071abf5feb58240d',
  'LateBenchmark.bench-final.hs':'d3ca7d8aa46ba789f1e42a89c584fe5aa0244d67f377034c071abf5feb58240d',
  'late_oracle.rs':'e37beb789ee61d654f5d8f4b221afd1f5425a26ffcb8c82160c33719a7d9a10a',
  '../../.audit-plume-late.4Yth3g/late-benchmark':'140522bd3521d354dd55494bf5b8949a28b6483399c5b012d2215937a1ebe271'
};
for(const [p,want] of Object.entries(hashes)) if(sha(p)!==want) throw Error(`Changed ${p}`);
for(const line of read('../PLUME_FILE_INVENTORY.tsv').trim().split('\n').slice(1)) {
  const [p,,bytes,,want]=line.split('\t');
  if(sha('../Plume/'+p)!==want||readFileSync(resolve(dir,'../Plume',p)).length!==+bytes) throw Error(`Original changed ${p}`);
}
const originalTests=read('../Plume/versioned/v1.2/Tests.hs');
// apply_patch also removed one final blank line; account for exactly that
// byte-level difference, not a broad whitespace-normalized comparison.
const expectedTests=originalTests.replace('module Tests (tabulate) where','module Tests where').replace(/\n$/,'');
if(read('../../.audit-plume-late.4Yth3g/overlay/Tests.hs')!==expectedTests) throw Error('Unexpected Tests bridge');
const qualification={};
for(const [name,n,finite,fail] of [['logistic',392,350,28],['logistic-debug',392,350,28],
  ['long',42,34,2],['functional',16,12,0],['functional-debug',16,10,0],['first-digit',8,8,null]]) {
  const runs=json(`late-${name}-runs.json`);
  const returned=runs.filter(x=>x.status===0&&!x.error);
  if(runs.length!==n||returned.length!==finite) throw Error(`Changed counts ${name}`);
  for(const x of runs) if(x.status!==0&&!(x.error==='ETIMEDOUT'&&x.signal==='SIGKILL')) throw Error(`Unexpected failure ${name}`);
  const rows=read(`late-${name}-results.tsv`).trim().split('\n');
  if(rows.length!==finite) throw Error('Missing finite row');
  if(fail!==null&&!read(`late-${name}-oracle.log`).includes(`checks=${finite}\tfailures=${fail}`)) throw Error(`Oracle mismatch ${name}`);
  qualification[name]={requests:n,finitePrefixes:finite,cappedObservations:n-finite,numericalFailures:fail};
}
if(read('late-logistic-results.tsv')!==read('late-logistic-debug-results.tsv')) throw Error('O0/O2 short prefixes differ');
const keyed=filename=>new Map(read(filename).trim().split('\n').map(row=>[row.split('\t').slice(0,2).join('\t'),row]));
const optimized=keyed('late-functional-results.tsv');
for(const [key,row] of keyed('late-functional-debug-results.tsv')) if(optimized.has(key)&&optimized.get(key)!==row) throw Error('Shared functional prefix differs');
for(const mode of ['release','debug']) if(!read(`late-hyper-grid-${mode}.log`).includes('PASS Hyper logistic interval/history checks=160; 40 input/iteration cases')) throw Error('Hyper grid missing');
if(!read('late-oracle-selfcheck.log').includes('recurrence=91')) throw Error('Oracle selfcheck missing');
if(!read('late-hyper-memcheck.log').includes('ERROR SUMMARY: 0 errors')) throw Error('Memcheck failure');
const runs=json('late-bench-runs.json');
const rows=read('late-bench-results.tsv').trim().split('\n').slice(1).map(row=>{
  const [round,variant,input,iterations,bits,cpu,allocation,checksum,result]=row.split('\t');
  if(result!=='PASS'||+bits!==32||+cpu<=0) throw Error('Unqualified benchmark row');
  return {round:+round,variant,input,iterations:+iterations,cpu:+cpu,allocation:+allocation,checksum};
});
if(runs.length!==108||rows.length!==108||runs.some(x=>x.status!==0||x.error)) throw Error('Benchmark incomplete');
for(let i=0;i<runs.length;i++) {
  const r=runs[i],row=rows[i];
  if(r.round!==row.round||r.variant!==row.variant||r.input!==row.input||r.iterations!==row.iterations||
    r.args[0]!=='-c'||r.args[1]!=='6'||!r.stdout.trim().endsWith('PASS')) throw Error('Benchmark record mismatch');
}
const kept=rows.filter(x=>x.round>0);
if(kept.length!==96) throw Error('Warmup count');
const median=a=>{const b=[...a].sort((x,y)=>x-y);return (b[(b.length-1)>>1]+b[b.length>>1])/2;};
let seed=0x5eeda17;
const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/2**32;};
const bootstrap=a=>{
  const estimates=Array.from({length:10000},()=>median(Array.from({length:a.length},()=>a[Math.floor(random()*a.length)]))).sort((a,b)=>a-b);
  return [estimates[249],estimates[9749]];
};
const groups=[];
for(const input of ['0.1','0.5467']) for(const iterations of [4,8,10]) {
  const selected=kept.filter(x=>x.input===input&&x.iterations===iterations);
  const variants={};
  for(const variant of ['sb-float','dy-float']) {
    const a=selected.filter(x=>x.variant===variant);
    if(a.length!==8||new Set(a.map(x=>x.round)).size!==8||new Set(a.map(x=>x.checksum)).size!==1) throw Error('Unpaired/inconsistent group');
    variants[variant]={medianCpuPs:median(a.map(x=>x.cpu)),minCpuPs:Math.min(...a.map(x=>x.cpu)),maxCpuPs:Math.max(...a.map(x=>x.cpu)),
      medianAllocatedBytes:median(a.map(x=>x.allocation)),minAllocatedBytes:Math.min(...a.map(x=>x.allocation)),maxAllocatedBytes:Math.max(...a.map(x=>x.allocation))};
  }
  const ratios=Array.from({length:8},(_,i)=>{
    const pair=selected.filter(x=>x.round===i+1),a=pair.find(x=>x.variant==='sb-float'),b=pair.find(x=>x.variant==='dy-float');
    return b.cpu/a.cpu;
  });
  const group={input,iterations,variants,dyadicOverSignedMedianCpu:variants['dy-float'].medianCpuPs/variants['sb-float'].medianCpuPs,
    pairedMedianRatio:median(ratios),pairedMedianBootstrap95:bootstrap(ratios),pairedRatios:ratios,
    dyadicOverSignedAllocation:variants['dy-float'].medianAllocatedBytes/variants['sb-float'].medianAllocatedBytes};
  groups.push(group);
  console.log(`${input}\tk=${iterations}\tdy/signed CPU=${group.dyadicOverSignedMedianCpu.toFixed(2)}\tallocation=${group.dyadicOverSignedAllocation.toFixed(2)}\tpaired95=${group.pairedMedianBootstrap95.map(x=>x.toFixed(2)).join('..')}`);
}
const result={hashes,qualification,shortO0O2Identical:true,sharedFunctionalPrefixesIdentical:true,
  hyperChecksPerBuild:160,oracleSelfChecks:91,memcheckErrors:0,benchObservations:108,benchKept:96,
  groups,scope:'Within-GHC valid logistic recurrences; no Hyper speed/RSS/binary-size claim. Resource caps are observations, not nontermination proofs.'};
writeFileSync(resolve(dir,'late-summary.json'),JSON.stringify(result,null,2)+'\n');
console.log('PASS source/result/benchmark verification; details in late-summary.json');
