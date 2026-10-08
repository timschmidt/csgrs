import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const root=import.meta.dirname,ws=resolve(root,'../../..');
const hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(root+'/'+p,'utf8'));
const rows=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const snapshot=json('snapshot.json');assert.equal(snapshot.files.length,393);
for(const[p,h]of snapshot.files)assert.equal(hash(root+'/snapshot/'+p),h,p);
assert.equal(hash(root+'/baseline.rs'),snapshot.baselineSha256);
const original=readFileSync(root+'/snapshot/hypersolve/src/curve_resultant.rs','utf8');
const extract=name=>{const start=original.indexOf('fn '+name+'(');assert(start>=0);const end=original.indexOf('\n}\n',start);assert(end>start);return original.slice(start,end+3);};
assert.equal(readFileSync(root+'/baseline.rs','utf8'),['multiply_exact_polynomials','exact_real_is_zero'].map(extract).join('\n'));
function validate(records,folder='',success=true){for(const r of records){
 assert.equal(hash(root+'/'+folder+r.file),r.sha256);assert.equal(hash(root+'/'+folder+r.file+'.stderr'),r.stderrSha256);
 if(r.harnessSha256)assert.equal(hash(root+'/main.rs'),r.harnessSha256);
 if(r.baselineSha256)assert.equal(hash(root+'/baseline.rs'),r.baselineSha256);
 if(r.source)for(const[p,h]of r.source)assert.equal(hash(root+'/'+folder+p),h);
 if(r.binary)assert.equal(hash(r.binary),r.binarySha256);
 if(success){assert.equal(r.status,0);assert.equal(r.error,null);assert.equal(r.signal,null);}
}}
const builds=json('build-runs.json');assert.equal(builds.length,3);validate(builds);
for(const r of builds)assert.equal(hash(root+'/Cargo.lock'),r.lockSha256);
const checks=json('check-runs.json');assert.equal(checks.length,2);validate(checks);
for(const r of checks){assert.equal(hash(root+'/binaries/'+r.name),r.binarySha256);assert.deepEqual(rows(r.file),[
 ['PASS','shape-integer-grid','2880'],['PASS','large-rational-grid','270'],['PASS','symbolic-and-zero-boundaries','95'],['SUMMARY','3245'],
]);}
const failed=json('memory-runs.json');assert.equal(failed.length,1);validate(failed,'',false);assert.equal(failed[0].status,101);
assert(readFileSync(root+'/'+failed[0].file+'.stderr','utf8').includes('allocation accounting escaped measured window'));
assert(readFileSync(root+'/'+failed[0].file+'.stderr','utf8').includes('left: 21384'));
const oracleFolder='coefficient-oracle/';
const oracleOriginal=json(oracleFolder+'runs.json');assert.equal(oracleOriginal.length,1);validate(oracleOriginal,oracleFolder,false);
assert.equal(oracleOriginal[0].mode,'debug');assert.equal(oracleOriginal[0].status,null);assert.equal(oracleOriginal[0].error,'ETIMEDOUT');assert.equal(oracleOriginal[0].signal,'SIGKILL');
assert.deepEqual(rows(oracleFolder+oracleOriginal[0].file),[['PASS','coefficient-export-shapes','2880']]);
assert.equal(hash(root+'/'+oracleFolder+'debug-build.log'),oracleOriginal[0].buildSha256);
const oracleExtended=json(oracleFolder+'extended-runs.json');assert.equal(oracleExtended.length,2);validate(oracleExtended,oracleFolder);
assert.deepEqual(oracleExtended.map(r=>r.mode),['release','debug']);
assert.equal(oracleExtended[1].binarySha256,oracleOriginal[0].binarySha256,'extended debug reruns the identical binary');
const oracleExpected=[['PASS','coefficient-export-shapes','2880'],['PASS','coefficient-export-cost-density-lifetimes','1800'],['PASS','coefficient-export-zero-boundaries','225'],['SUMMARY','4905']];
for(const r of [...oracleOriginal,...oracleExtended]){
 for(const[p,h]of r.sources)assert.equal(hash(root+'/'+oracleFolder+p),h);
 assert.equal(hash(root+'/'+oracleFolder+'Cargo.lock'),r.lockSha256);
 assert.equal(readFileSync(root+'/'+oracleFolder+r.file+'.stderr','utf8'),'');
 if(r.error===null){assert.deepEqual(rows(oracleFolder+r.file),oracleExpected);assert(Date.parse(r.finished)-Date.parse(r.started)<r.capMs);}
}
assert.equal(oracleExtended[0].sha256,oracleExtended[1].sha256);
for(const[p,h]of [
 ['extended-release-build.log','732c3a3845ff8a1850210ec0e4056268b898d4aae39ab4d12ab2f8192b44e0fb'],
 ['run.mjs','6c3b3f49054fc8faa3d393e5f0f00c3a897549cb1537f19054cff6b694ceacfd'],
 ['run-extended.mjs','0059dc5540479adf7348f3f6ba52ea238028cef5451455f17b899de96c77d7f7'],
])assert.equal(hash(root+'/'+oracleFolder+p),h);
for(const p of ['debug-build.log','extended-release-build.log'])assert.deepEqual(JSON.parse(readFileSync(root+'/'+oracleFolder+p,'utf8').trimEnd().split('\n').at(-1)),{status:0,error:null});
const coefficientExport={checksPerMode:4905,usesHyperEqualityForOracle:false,identicalDebugReleaseOutput:true,originalDebugTimeoutSeconds:300,originalTimeoutPreserved:true,extendedDebugCapSeconds:1200,debugWallSeconds:(Date.parse(oracleExtended[1].finished)-Date.parse(oracleExtended[1].started))/1000,releaseWallSeconds:(Date.parse(oracleExtended[0].finished)-Date.parse(oracleExtended[0].started))/1000};
const median=a=>{a=[...a].sort((a,b)=>a-b);return(a[Math.floor((a.length-1)/2)]+a[Math.floor(a.length/2)])/2;};
function timing(records,folder,algorithmNames,rounds,seconds,last){
 validate(records,folder);let verifiedProducts=0,observations=0,measuredBatches=0,minCPU=Infinity;
 const summaries=[];
 for(const r of records){
  assert.equal(hash(root+'/'+folder+'binaries/release'),r.binarySha256);
  const data=rows(folder+r.file);assert.deepEqual(data.at(-1),['PASS',last]);assert.equal(data[0][0],'INPUT');
  assert.deepEqual(r.args.slice(0,2),['-c','6']);assert.equal(r.args.at(-2),String(seconds));assert.equal(r.args.at(-1),String(rounds));
  const calibration=data.filter(r=>r[0]==='CALIBRATE'),warm=data.filter(r=>r[0]==='WARMUP'),measured=data.filter(r=>r[0]==='MEASURE');
  assert.equal(calibration.length,algorithmNames.length);assert.equal(new Set(calibration.map(r=>r[1])).size,algorithmNames.length);
  assert.equal(warm.length,2*algorithmNames.length);assert.equal(measured.length,rounds*algorithmNames.length);
  observations+=warm.length+measured.length;measuredBatches+=measured.length;
  const seed=folder?+data[0][6]:0;
  for(const row of [...warm,...measured]){
   const[phase,round,position,name,count,c,w]=row;assert.equal(algorithmNames[(+round + +position + seed)%algorithmNames.length],name);
   const cal=calibration.find(r=>r[1]===name);assert.equal(count,cal[2]);assert(+cal[3]>=seconds);assert(+c>0&&+w>0);
   if(phase==='MEASURE'){assert(+round>=2&&+round<rounds+2);assert(+c>=seconds*.5);minCPU=Math.min(minCPU,+c);verifiedProducts+=+count;}
  }
  const ratios={};for(const name of algorithmNames.filter(n=>n!=='baseline')){
   const cpu=[],wall=[];for(let round=2;round<rounds+2;round++){
    const b=measured.find(r=>+r[1]===round&&r[3]==='baseline'),a=measured.find(r=>+r[1]===round&&r[3]===name);assert(a&&b);
    cpu.push((+a[5]/+a[4])/(+b[5]/+b[4]));wall.push((+a[6]/+a[4])/(+b[6]/+b[4]));
   }
   ratios[name]={cpuMedian:median(cpu),wallMedian:median(wall),cpuPaired:cpu,wallPaired:wall};
  }
  summaries.push({case:r.name,input:data[0].slice(1),ratios});
 }
 return{processes:records.length,observations,measuredBatches,verifiedProducts,minCPU,summaries};
}
const preliminary=json('explore-runs.json');assert.equal(preliminary.length,32);
const explore=timing(preliminary,'',['baseline','k8','k16','k32','gated'],3,.1,'bench');
const lifeBuilds=json('lifetime/build-runs.json');assert.equal(lifeBuilds.length,2);validate(lifeBuilds,'lifetime/');
for(const r of lifeBuilds)assert.equal(hash(root+'/lifetime/Cargo.lock'),r.lockSha256);
const memory=json('lifetime/memory-runs.json');assert.equal(memory.length,72);validate(memory,'lifetime/');const allocations=[];
for(const r of memory){
 assert.equal(hash(root+'/lifetime/binaries/memory'),r.binarySha256);
 const data=rows('lifetime/'+r.file);assert.equal(data.length,5);assert.equal(data[0][0],'INPUT');
 const m=data.slice(1);assert.deepEqual(m.map(r=>r.slice(0,2)),['baseline','control','k8','gated'].map(n=>['MEMORY',n]));
 for(const row of m){assert.equal(row.length,8);assert.equal(row[7],'0');for(const i of [2,3,4])assert(Number.isSafeInteger(+row[i])&&+row[i]>=0);}
 assert.deepEqual(m[0].slice(2),m[1].slice(2),'identical-code allocation control');
 const baseline=m[0];allocations.push({case:r.name,algorithms:m.map(row=>({name:row[1],calls:+row[2],bytes:+row[3],peakExtraLive:+row[4],beforeOutputDropDelta:+row[5],afterOutputDropDelta:+row[6],afterAllOwnersDropDelta:+row[7],callsRatio:+row[2]/+baseline[2],bytesRatio:+row[3]/+baseline[3],peakRatio:+row[4]/+baseline[4]}))});
}
const stableRuns=json('lifetime/stable-runs.json');assert.equal(stableRuns.length,60);
const stable=timing(stableRuns,'lifetime/',['baseline','control','k8','gated'],6,.25,'bench-v2');
const grouped={};for(const r of stable.summaries){const key=r.input.slice(0,5).join('-');(grouped[key]??=[]).push(r);}
assert.equal(Object.keys(grouped).length,20);
const families=Object.entries(grouped).map(([family,runs])=>{
 assert.equal(runs.length,3);assert.deepEqual(runs.map(r=>r.input[5]).sort(),['137','149','163']);
 const ratios={};for(const name of ['control','k8','gated']){const cpu=runs.map(r=>r.ratios[name].cpuMedian),wall=runs.map(r=>r.ratios[name].wallMedian);ratios[name]={cpuMedian:median(cpu),cpuProcessMedianRange:[Math.min(...cpu),Math.max(...cpu)],wallMedian:median(wall),wallProcessMedianRange:[Math.min(...wall),Math.max(...wall)]};}
 return{family,ratios,identicalCodeControlWithinTenPercent:runs.every(r=>r.ratios.control.cpuMedian>=.9&&r.ratios.control.cpuMedian<=1.1)};
});
const currentDrift=snapshot.files.filter(([p,h])=>hash(ws+'/'+p)!==h).map(([p])=>p);
const result={snapshotFiles:snapshot.files.length,currentDrift,correctnessChecksPerMode:3245,coefficientExport,originalWindowedMemoryFailure:'preserved; cache ownership invalidated zero-live-after-output-drop assumption',allocationCases:memory.length,allocationProfiles:memory.length*4,allMeasuredOwnersReleased:true,allocations,explore,stable,families,productionChanges:0,status:'isolated experiment; actual-caller and binary/code-size retention gates remain open',limitations:['shared host, pinned CPU not exclusive','warm/fresh input lifetimes measured separately','exact output verification is included in timed region','process medians across three seeds are ranges, not confidence bounds','no production caller or whole-application speedup claimed']};
writeFileSync(root+'/analysis.json',JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify({...result,allocations:undefined,explore:{...explore,summaries:undefined},stable:{...stable,summaries:undefined}},null,2));
