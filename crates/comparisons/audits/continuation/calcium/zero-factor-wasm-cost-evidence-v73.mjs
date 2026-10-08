import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {costEvidence,gate} from './check-zero-factor-wasm-cost-v73.mjs';
import {sha,json} from './zero-factor-wasm-cost-sources-v73.mjs';
import {validateRow,validateRows,readRows} from './zero-factor-wasm-cost-protocol-v73.mjs';
export function evidence(){
 const cost=costEvidence();assert.deepEqual(cost,json('zero-factor-wasm-cost-summary-v73.json'));
 const runs=json('zero-factor-wasm-cost-runs-v73.json').runs,outer=gate('zero-factor-wasm-cost-campaign-v73');
 const children=['zero-factor-wasm-cost-environment-before-v73',...runs.filter(r=>r.phase==='pilot').map(r=>r.tag),
  'zero-factor-wasm-cost-environment-after-pilots-v73',...runs.filter(r=>r.phase==='cpu').map(r=>r.tag),'zero-factor-wasm-cost-environment-after-v73'];
 assert.equal(children.length,107);
 const rows=readRows('results/zero-factor-wasm-cost-campaign-v73.stdout'),begins=rows.filter(r=>r.pid),ends=rows.filter(r=>r.finished),pairs=rows.filter(r=>r.phase==='pair-complete');
 assert.equal(rows.length,263);assert.equal(begins.length,107);assert.equal(ends.length,107);assert.equal(pairs.length,48);
 assert.deepEqual(ends.map(r=>r.tag),children);let previous=Date.parse(outer.started);
 for(const [i,tag]of children.entries()){
  const g=gate(tag);assert.deepEqual(ends[i],g);assert.deepEqual({...begins[i],pid:undefined},{tag,started:g.started,cwd:g.cwd,command:g.command,args:g.args,pid:undefined});
  assert(Number.isSafeInteger(begins[i].pid)&&begins[i].pid>0);assert(Date.parse(g.started)>=previous);previous=Date.parse(g.finished);
 }
 assert(Date.parse(outer.finished)>=previous);assert.deepEqual(pairs,runs.filter(r=>r.phase==='cpu').filter((r,i)=>i%2===1)
  .map(r=>({checkpoint:73,phase:'pair-complete',round:r.round,instanceMode:r.instanceMode})));
 assert.deepEqual(rows.at(-1),{checkpoint:73,status:'wasm-cost-collection-terminal',runs:104,pilotRows:3648,cpuRows:43776,
  limits:'All complete reports checked; statistical analysis and consumer/size/retention decisions remain open.'});
 const controls=json('zero-factor-wasm-cost-controls-v73.json');
 assert.deepEqual(controls.recordControls,{accepted:2,rejected:36});assert.deepEqual(controls.streamControls,{accepted:2,rejected:14});
 assert.equal(controls.results.length,38);assert.equal(controls.streamResults.length,16);
 for(const[p,h]of Object.entries(controls.scripts))assert.equal(sha(p),h,p);
 for(const r of controls.results){let error=null;try{validateRow(r.row,r.group,'baseline',r.instanceMode,r.index);}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error,r.error);assert.equal(error===null,r.accepted);
 }
 for(const r of controls.streamResults){assert.equal(sha(r.path),r.sha256);assert.equal(sha(r.sourceCapture),r.sourceCaptureSha256);assert.equal(r.authoredExcerpt,true);
  let error=null;try{validateRows(r.path,json(r.plan),'baseline',r.instanceMode);}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
  assert.equal(error,r.error);assert.equal(error===null,r.accepted);
 }
 assert.deepEqual(json('results/zero-factor-wasm-cost-controls-v73.stdout'),{checkpoint:73,status:'wasm-cost-controls-pass',recordControls:controls.recordControls,streamControls:controls.streamControls});
 const gates=[...children,'zero-factor-wasm-cost-campaign-v73','zero-factor-wasm-cost-check-v73','zero-factor-wasm-cost-controls-v73'];
 assert.equal(gates.length,110);assert.equal(new Set(gates).size,110);for(const t of gates){gate(t);assert.equal(readFileSync('results/'+t+'.stderr').length,0);}
 const check=json('results/zero-factor-wasm-cost-check-v73.stdout');
 assert.deepEqual(check,{checkpoint:73,status:cost.status,campaign:cost.campaign,categories:cost.categories,memory:cost.memory,storage:cost.storage,limits:cost.limits});
 const examples=cost.cpu.filter(r=>r.policy===0&&r.lifecycle==='retained'&&[3,23,35,41,104,113].includes(r.case)).map(r=>({
  case:r.case,label:r.label,instanceMode:r.instanceMode,equalResult:r.equalResult,category:r.category,transition:r.transition,
  baselineNs:r.baselineNs,candidateNs:r.candidateNs,pairedMedianRatio:r.pairedMedianRatio,bootstrap:r.statistics.bootstrap.interval,orderStatistic:r.statistics.orderStatistic.interval,
  classification:r.classification}));
 return {checkpoint:73,status:'isolated-wasm-cost-qualified',sourceOriginSha256:cost.sourceOriginSha256,liveFiles:956,candidateFiles:176,
  retainedContinuationTransfers:6,newDonorLines:0,gates,campaign:cost.campaign,categories:cost.categories,examples,
  controls:{record:controls.recordControls,stream:controls.streamControls},memory:cost.memory,storage:cost.storage,limits:cost.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(evidence()));
