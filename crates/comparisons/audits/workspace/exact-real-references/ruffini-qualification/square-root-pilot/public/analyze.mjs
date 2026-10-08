import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname,pilot=root+'/..',hash=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(root+'/'+p,'utf8')),rows=p=>readFileSync(root+'/'+p,'utf8').trimEnd().split('\n').map(s=>s.split('\t'));
const partial=process.argv.includes('--partial');
const snapshot=json('../snapshot.json');for(const[p,h]of snapshot.files)assert.equal(hash(pilot+'/snapshot/'+p),h,p);
const origin=json('../public-copy-origin.json');assert.equal(origin.files.length,152);
for(const[p,h]of origin.files){
 let expected=readFileSync(pilot+'/snapshot/hypersolve/'+p,'utf8');
 if(p==='Cargo.toml'){expected=expected.replace('name = "hypersolve"','name = "hypersolve-diagonal-pilot"');for(const name of ['hyperreal','hyperlattice','hyperlimit'])expected=expected.replace('path = "../'+name+'"','path = "../snapshot/'+name+'"');}
 else if(p==='src/curve_resultant.rs')expected=expected.replace(readFileSync(pilot+'/baseline.rs','utf8'),readFileSync(pilot+'/diagonal.rs','utf8'));
 else assert.equal(hash(pilot+'/hypersolve-diagonal/'+p),h,p);
 assert.equal(readFileSync(pilot+'/hypersolve-diagonal/'+p,'utf8'),expected,p);
}
function validate(data,runner='run.mjs'){for(const r of data){
 assert.equal(r.status,0);assert.equal(r.signal,null);assert.equal(r.error,null);
 assert.equal(hash(root+'/'+r.file),r.sha256);assert.equal(hash(root+'/'+r.file+'.stderr'),r.stderrSha256);
 assert.equal(hash(r.binary),r.binarySha256);assert.equal(hash(root+'/'+runner),r.runnerSha256);
 if(r.sources)for(const[p,h]of r.sources)assert.equal(hash(root+'/'+p),h,p);
 if(r.lockSha256)assert.equal(hash(root+'/Cargo.lock'),r.lockSha256);
}}
const builds=json('build-runs.json');assert.equal(builds.length,4);validate(builds);
const checks=json('check-runs.json');assert.equal(checks.length,4);validate(checks);
const expected=[];for(const d of [1,2,3,4,8])for(const bits of [4,32])for(const seed of [17,42])for(const layout of ['terminal','sampled'])for(const orientation of ['First','Second'])expected.push(['PASS','public',String(d),String(bits),String(seed),layout,orientation,String(2*d+(layout==='sampled'?1:0))]);
expected.push(['PASS','symbolic-public-map','1'],['SUMMARY','81']);
for(const r of checks){assert.deepEqual(rows(r.file),expected);assert.equal(r.sha256,checks[0].sha256);assert.equal(readFileSync(root+'/'+r.file+'.stderr','utf8'),'');}
const probes=json('probe-size-runs.json');assert.equal(probes.length,4);validate(probes,'probe-and-size.mjs');
const sizes={};for(const r of probes){
 const text=readFileSync(root+'/'+r.file,'utf8');
 if(r.mode==='probe'){assert.equal(hash(root+'/'+r.algorithm+'-probe.gdb'),r.scriptSha256);assert.equal(text.split('AUDIT_SQUARE_ROOT_HIT').length-1,1);assert(text.includes('PASS\tpublic-probe')&&text.includes('exited normally'));}
 else {const sections={};for(const line of text.split('\n')){const m=/^(\.[^ ]+)\s+(\d+)\s+\d+$/.exec(line);if(m)sections[m[1]]=+m[2];}assert(sections['.text']>0);sizes[r.algorithm]={fileBytes:r.binaryFileBytes,sections,totalSections:+/^Total\s+(\d+)$/m.exec(text)[1]};}
}
const records=json('bench-runs.json');if(!partial)assert.equal(records.length,108);validate(records);
const median=a=>{a=[...a].sort((a,b)=>a-b);return(a[Math.floor((a.length-1)/2)]+a[Math.floor(a.length/2)])/2;};
let observations=0,measuredBatches=0,verifiedCalls=0,minCPU=Infinity;const processes=[];
for(const r of records){
 const data=rows(r.file);assert.equal(data.length,11);assert.deepEqual(data.at(-1),['PASS','public-bench']);assert.equal(data[0][0],'INPUT');
 assert.deepEqual(r.args.slice(0,2),['-c','6']);assert.deepEqual(r.args.slice(-2),['0.5','6']);assert.equal(r.algorithm,['baseline','control','diagonal'][(r.position + +data[0][5])%3]);
 const calibration=data[1];assert.equal(calibration[0],'CALIBRATE');assert(+calibration[2]>=.5);
 const samples=data.slice(2,-1);assert.equal(samples.length,8);observations+=8;
 for(let i=0;i<samples.length;i++){const row=samples[i];assert.equal(row[0],i<2?'WARMUP':'MEASURE');assert.equal(+row[1],i);assert.equal(row[2],calibration[1]);assert(+row[3]>0&&+row[4]>0);if(i>=2){assert(+row[3]>=.25);minCPU=Math.min(minCPU,+row[3]);verifiedCalls+=+row[2];measuredBatches++;}}
 processes.push({name:r.name,input:data[0].slice(1),algorithm:r.algorithm,cpuPerCall:median(samples.slice(2).map(r=>+r[3]/+r[2])),wallPerCall:median(samples.slice(2).map(r=>+r[4]/+r[2]))});
}
const groups={};for(const p of processes)(groups[p.input.join('-')]??=[]).push(p);
const pairs=[];for(const[key,group]of Object.entries(groups)){
 if(group.length!==3){assert(partial);continue;}assert.equal(new Set(group.map(r=>r.algorithm)).size,3);
 const b=group.find(r=>r.algorithm==='baseline');const ratios={};for(const name of ['control','diagonal']){const a=group.find(r=>r.algorithm===name);ratios[name]={cpu:a.cpuPerCall/b.cpuPerCall,wall:a.wallPerCall/b.wallPerCall};}
 pairs.push({key,family:b.input.slice(0,4).join('-'),seed:b.input[4],ratios});
}
const grouped={};for(const p of pairs)(grouped[p.family]??=[]).push(p);
const families=Object.entries(grouped).map(([family,p])=>{const ratios={};for(const name of ['control','diagonal']){const c=p.map(r=>r.ratios[name].cpu),w=p.map(r=>r.ratios[name].wall);ratios[name]={cpuMedian:median(c),cpuRange:[Math.min(...c),Math.max(...c)],wallMedian:median(w),wallRange:[Math.min(...w),Math.max(...w)]};}return{family,seedGroups:p.length,ratios,controlWithinTenPercent:p.every(r=>r.ratios.control.cpu>=.9&&r.ratios.control.cpu<=1.1)};});
if(!partial){assert.equal(pairs.length,36);assert.equal(families.length,12);assert(families.every(f=>f.seedGroups===3));}
const result={snapshotFiles:snapshot.files.length,candidateFiles:origin.files.length,candidateChanges:['Cargo package/paths','square-root backward coefficient loop'],publicChecksPerModePerAlgorithm:81,identicalCheckOutput:true,helperHitsPerProbe:1,sizes,sizeCaveat:'separate package identities change symbol/debug-string sizes; no production binary-size saving claimed',processes:records.length,observations,measuredBatches,verifiedCalls,minCPU,pairs,families,limitations:['shared host, pinned CPU not exclusive','separate binaries/processes; code-layout and time-order effects remain','six measured batches per process after warmup/calibration','seed-dependent order cycles use two of three cyclic orders, not full counterbalancing','CPU and wall ranges are descriptive across three seed groups, not confidence bounds'],status:partial?'ONGOING exploratory public timing; no retention conclusion':'exploratory public timing; fully counterbalanced retention study still required',newProductionChanges:0};
writeFileSync(root+'/'+(partial?'partial-analysis.json':'analysis.json'),JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({...result,pairs:undefined},null,2));
