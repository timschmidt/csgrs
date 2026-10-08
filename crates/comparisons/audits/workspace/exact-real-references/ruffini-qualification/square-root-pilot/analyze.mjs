import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(root+'/'+p,'utf8'));
const lines=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const snapshot=json('snapshot.json');assert.equal(snapshot.files.length,397);
for(const[p,h]of snapshot.files)assert.equal(hash(root+'/snapshot/'+p),h,p);
const original=readFileSync(root+'/snapshot/hypersolve/src/curve_resultant.rs','utf8');
const extract=name=>{const start=original.indexOf('fn '+name+'('),end=original.indexOf('\n}\n',start);assert(start>=0&&end>start);return original.slice(start,end+3);};
assert.equal(readFileSync(root+'/baseline.rs','utf8'),extract('exact_polynomial_square_root'));assert.equal(hash(root+'/baseline.rs'),snapshot.baselineSha256);
assert.equal(readFileSync(root+'/shared.rs','utf8'),['add_exact_polynomials','multiply_exact_polynomials','exact_polynomial_is_zero','strict_reciprocal','exact_real_is_zero','scale_exact_polynomial','subtract_exact_polynomials'].map(extract).join('\n'));assert.equal(hash(root+'/shared.rs'),snapshot.sharedSha256);
const before='        let product = multiply_exact_polynomials(&root, &root);\n        let known = product\n            .get(root_degree + power)\n            .cloned()\n            .unwrap_or_else(Real::zero);';
const after='        // Coefficients through `power` are still exact zero. Only this diagonal\n        // of the square is consumed; keep the full convolution\'s summation order.\n        let mut known = Real::zero();\n        for first_power in power + 1..root_degree {\n            known += &root[first_power] * &root[root_degree + power - first_power];\n        }';
assert(extract('exact_polynomial_square_root').includes(before));assert.equal(readFileSync(root+'/diagonal.rs','utf8'),extract('exact_polynomial_square_root').replace(before,after));
function validate(records,folder='',runner,success=true){for(const r of records){
 assert.equal(hash(root+'/'+folder+r.file),r.sha256);assert.equal(hash(root+'/'+folder+r.file+'.stderr'),r.stderrSha256);
 for(const[p,h]of r.sources)assert.equal(hash(root+'/'+folder+p),h,p);
 assert.equal(hash(root+'/'+folder+'Cargo.lock'),r.lockSha256);assert.equal(hash(root+'/'+folder+runner),r.runnerSha256);
 if(r.binary)assert.equal(hash(r.binary),r.binarySha256);
 if(success){assert.equal(r.status,0);assert.equal(r.signal,null);assert.equal(r.error,null);}
}}
const first=json('build-runs.json');assert.equal(first.length,1);validate(first,'','run.mjs',false);assert.equal(first[0].status,101);assert(readFileSync(root+'/'+first[0].file+'.stderr','utf8').includes('Bus error'));
const host=json('build-host-runs.json');assert.equal(host.length,1);validate(host,'','build-host.mjs',false);assert.equal(host[0].status,101);assert(readFileSync(root+'/'+host[0].file+'.stderr','utf8').includes('Disk quota exceeded'));
const builds=json('workspace-build-runs.json');assert.equal(builds.length,2);validate(builds,'','run-workspace.mjs');
const v2builds=json('v2/build-runs.json');assert.equal(v2builds.length,2);validate(v2builds,'v2/','run.mjs');
const checks=json('v2/check-runs.json');assert.equal(checks.length,2);validate(checks,'v2/','run.mjs');
const expected=[...[0,1,2,3,4,7,8,15,16].map((d,i)=>['PROGRESS','rational-degree',String(d),String(162+i*216)]),['PASS','rational-grid','1890'],['PASS','high-degree-cost','33'],['PASS','zero-domain-boundaries','15'],['PASS','symbolic-boundaries','18'],['SUMMARY','1956']];
for(const r of checks){assert.deepEqual(lines('v2/'+r.file),expected);assert.equal(readFileSync(root+'/v2/'+r.file+'.stderr','utf8'),'');}
assert.equal(checks[0].sha256,checks[1].sha256);
const median=a=>{a=[...a].sort((a,b)=>a-b);return(a[Math.floor((a.length-1)/2)]+a[Math.floor(a.length/2)])/2;};
const records=json('v2/bench-runs.json');assert.equal(records.length,48);validate(records,'v2/','run.mjs');
let observations=0,measuredBatches=0,verifiedCalls=0,minCPU=Infinity;const processes=[];const names=['baseline','control','diagonal'];
for(const r of records){
 const rows=lines('v2/'+r.file);assert.equal(rows.length,29);assert.deepEqual(rows.at(-1),['PASS','bench']);assert.equal(rows[0][0],'INPUT');
 assert.deepEqual(r.args.slice(0,2),['-c','6']);assert.deepEqual(r.args.slice(-2),['0.25','6']);
 const seed=+rows[0][6],cal=rows.filter(r=>r[0]==='CALIBRATE'),warm=rows.filter(r=>r[0]==='WARMUP'),data=rows.filter(r=>r[0]==='MEASURE');
 assert.equal(cal.length,3);assert.equal(warm.length,6);assert.equal(data.length,18);assert.equal(new Set(cal.map(r=>r[1])).size,3);
 observations+=warm.length+data.length;measuredBatches+=data.length;
 for(const row of [...warm,...data]){
  const[phase,round,position,name,count,c,w]=row;assert.equal(name,names[(+round + +position + seed)%3]);assert.equal(count,cal.find(r=>r[1]===name)[2]);assert(+c>0&&+w>0);
  if(phase==='MEASURE'){assert(+round>=2&&+round<8);assert(+c>=.125);minCPU=Math.min(minCPU,+c);verifiedCalls+=+count;}
 }
 const ratios={};for(const name of ['control','diagonal']){const cpu=[],wall=[];for(let round=2;round<8;round++){
  const b=data.find(r=>+r[1]===round&&r[3]==='baseline'),a=data.find(r=>+r[1]===round&&r[3]===name);assert(a&&b);cpu.push((+a[5]/+a[4])/(+b[5]/+b[4]));wall.push((+a[6]/+a[4])/(+b[6]/+b[4]));
 }ratios[name]={cpuMedian:median(cpu),wallMedian:median(wall),pairedCPU:cpu,pairedWall:wall};}
 processes.push({name:r.name,input:rows[0].slice(1),ratios});
}
const grouped={};for(const p of processes)(grouped[p.input.slice(0,5).join('-')]??=[]).push(p);
assert.equal(Object.keys(grouped).length,16);
const families=Object.entries(grouped).map(([family,p])=>{
 assert.equal(p.length,3);assert.deepEqual(p.map(r=>r.input[5]).sort(),['137','149','163']);const ratios={};
 for(const name of ['control','diagonal']){const c=p.map(r=>r.ratios[name].cpuMedian),w=p.map(r=>r.ratios[name].wallMedian);ratios[name]={cpuMedian:median(c),cpuProcessMedianRange:[Math.min(...c),Math.max(...c)],wallMedian:median(w),wallProcessMedianRange:[Math.min(...w),Math.max(...w)]};}
 return{family,ratios,controlWithinTenPercent:p.every(r=>r.ratios.control.cpuMedian>=.9&&r.ratios.control.cpuMedian<=1.1)};
});
const memoryRuns=json('memory/runs.json');assert.equal(memoryRuns.length,2);validate(memoryRuns,'memory/','run.mjs');
const memoryRows=lines('memory/check.log');assert.deepEqual(memoryRows.pop(),['SUMMARY','288']);assert.equal(memoryRows.length,384);
const allocations=[];
for(let i=0;i<memoryRows.length;i+=4){
 const input=memoryRows[i];assert.equal(input[0],'INPUT');const rows=memoryRows.slice(i+1,i+4);assert.deepEqual(rows.map(r=>r.slice(0,2)),names.map(n=>['MEMORY',n]));
 for(const r of rows){assert.equal(r.length,8);assert.equal(r[7],'0');for(const index of [2,3,4])assert(Number.isSafeInteger(+r[index])&&+r[index]>=0);}
 assert.deepEqual(rows[0].slice(2),rows[1].slice(2));
 const base=rows[0],candidate=rows[2];allocations.push({family:input.slice(1).join('-'),baseline:base.slice(2).map(Number),diagonal:candidate.slice(2).map(Number),callsRatio:+candidate[2]/+base[2],bytesRatio:+candidate[3]/+base[3],peakRatio:+candidate[4]/+base[4]});
}
const allocationSummary={cases:96,profiles:288,allMeasuredOwnersReleased:true,identicalCodeControlsMatch:true,fields:['calls','bytes','peakExtraLive','beforeOutputDropDelta','afterOutputDropDelta','afterAllOwnersDropDelta'],nonincreasingCalls:allocations.every(r=>r.diagonal[0]<=r.baseline[0]),nonincreasingBytes:allocations.every(r=>r.diagonal[1]<=r.baseline[1]),nonincreasingPeak:allocations.every(r=>r.diagonal[2]<=r.baseline[2])};
const result={snapshotFiles:snapshot.files.length,currentDrift:snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p])=>p),checksPerMode:1956,semanticCases:652,algorithms:'baseline, identical-code control, diagonal',oracleUsesHyperEqualityForRationalCoefficients:false,initialOracleDraft:'not executed; independently noticed wrong radical x^2 coefficient corrected in v2, original source/binaries preserved',buildFailures:'sandboxed linker bus error and explicit host /tmp quota failure preserved; unchanged numerical sources build in workspace',processes:records.length,observations,measuredBatches,verifiedCalls,minCPU,families,processResults:processes,allocationSummary,allocations,newProductionChanges:0,status:'isolated caller experiment; public caller, binary/code-size and downstream retention gates still open'};
writeFileSync(root+'/analysis.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({...result,processResults:undefined,allocations:undefined},null,2));
