import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sources,sha,json} from './twelfth-cost-sources-v78.mjs';
import {groups,key,orders,shuffle,validate} from './twelfth-cost-protocol-v78.mjs';
import {median,correctedPairedStatistics} from './paired-statistics-v60.mjs';
export function isolatedEvidence(){
 const source=sources(),o=json('twelfth-isolated-origin-v78.json'),runs=json('twelfth-isolated-runs-v78.json'),binary=json('twelfth-cost-binaries-v78.json'),cal=json('twelfth-native-calibration-v78.json');
 assert.deepEqual(source,o.source);assert.equal(o.scriptSha256,sha('probe-twelfth-native-v78.mjs'));assert.equal(o.previousSummarySha256,sha('twelfth-native-summary-v78.json'));
 assert.deepEqual(o.selected,[0,2,34,39,84,93]);assert.deepEqual(o.cpuOrders,orders(12,'isolated-cpu'));assert.deepEqual(o.allocationOrders,orders(4,'isolated-allocation'));
 assert.equal(runs.originSha256,sha('twelfth-isolated-origin-v78.json'));assert.equal(runs.runs.length,192);
 const order=[];for(const phase of ['cpu','allocation'])for(const id of o.selected)for(let round=0;round<(phase==='cpu'?12:4);round++)for(const v of (phase==='cpu'?o.cpuOrders:o.allocationOrders)[round])order.push([phase,id,round,v]);
 assert.deepEqual(runs.runs.map(r=>[r.phase,r.case,r.round,r.variant]),order);
 let previous=Date.parse(o.recorded);const observed=[];
 assert(previous>Date.parse(json('results/twelfth-native-campaign-v78.json').finished));
 for(const r of runs.runs){
  const unshuffled=r.phase==='cpu'?groups().filter(g=>g.case===r.case).map(g=>({...g,iterations:cal.iterations[key(g)]})):[1,16].flatMap(n=>groups(n).filter(g=>g.case===r.case));
  const expected=shuffle(unshuffled,'isolated-'+r.phase+'-'+r.case+'-'+r.round);
  assert.equal(sha(r.plan),r.planSha256);assert.deepEqual(json(r.plan),expected);
  const g=json('results/'+r.tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(g.command,'taskset');assert.equal(g.cwd,resolve('.'));
  assert.equal(readFileSync('results/'+r.tag+'.stderr').length,0);
  const b=binary.binaries.find(b=>b.variant===r.variant&&b.mode===r.phase);assert.equal(sha(b.path),b.sha256);
  assert.deepEqual(g.args,['-c','2',b.path,r.phase,resolve('twelfth-cost-input-v78.json'),resolve(r.plan)]);
  assert(Date.parse(g.started)>=previous);assert(Date.parse(g.finished)>=Date.parse(g.started));previous=Date.parse(g.finished);
  for(const row of validate('results/'+r.tag+'.stdout',expected,r.variant,r.phase))observed.push({...row,phase:r.phase,round:r.round,variant:r.variant});
 }
 assert(Date.parse(runs.finished)>=previous);assert.equal(observed.length,1440);
 const comparisons=[];
 for(const g of groups().filter(g=>o.selected.includes(g.case))){
  const rs=observed.filter(r=>key(r)===key(g)),cpu=rs.filter(r=>r.phase==='cpu');assert.equal(cpu.length,24);
  const ratios=Array.from({length:12},(_,round)=>cpu.find(r=>r.round===round&&r.variant==='candidate').elapsed_ns/cpu.find(r=>r.round===round&&r.variant==='baseline').elapsed_ns);
  const statistics=correctedPairedStatistics(ratios,'twelfth-isolated-v78:'+key(g));
  const allocations=[];for(const n of [1,16]){
   const a=rs.filter(r=>r.phase==='allocation'&&r.iterations===n);assert.equal(a.length,8);const fields={};
   for(const field of ['requests','requested_bytes','peak_delta','live_delta']){
    const number=r=>r[field]/(['requests','requested_bytes'].includes(field)?n:1);
    fields[field]=Object.fromEntries(['baseline','candidate'].map(v=>{const values=a.filter(r=>r.variant===v).map(number);return[v,{median:median(values),min:Math.min(...values),max:Math.max(...values)}];}));
   }allocations.push({iterations:n,fields});
  }
  comparisons.push({...g,iterations:cal.iterations[key(g)],baselineNs:median(cpu.filter(r=>r.variant==='baseline').map(r=>r.elapsed_ns/r.iterations)),
   candidateNs:median(cpu.filter(r=>r.variant==='candidate').map(r=>r.elapsed_ns/r.iterations)),statistics,allocations});
 }
 assert.equal(comparisons.length,36);
 return{checkpoint:78,status:'isolated-replication-verified',source,selected:o.selected,processes:192,cpuRows:864,allocationRows:576,comparisons,
  started:runs.started,finished:runs.finished,limits:o.reason};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const evidence=isolatedEvidence();if(process.argv.includes('--record'))writeFileSync('twelfth-isolated-summary-v78.json',JSON.stringify(evidence,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(evidence,json('twelfth-isolated-summary-v78.json'));
 console.log(JSON.stringify({checkpoint:78,status:evidence.status,selected:evidence.selected,processes:192,cpuRows:864,allocationRows:576,
  examples:evidence.comparisons.filter(r=>[34,39,84,93].includes(r.case)&&r.precision===-64&&r.lifecycle==='retained'),retained:false}));
}
