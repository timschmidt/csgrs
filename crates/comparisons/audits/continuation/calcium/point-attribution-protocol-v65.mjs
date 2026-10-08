import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sha,json} from './point-demand-sources.mjs';
import {wasmBindings,groupKey,flags,variants,orderFor} from './point-wasm-protocol.mjs';
export {sha,json,groupKey,flags,variants,orderFor};
export const keys=['12:0:2:retained','17:0:2:fresh','44:1:3:retained','37:1:0:retained',
 '3:1:3:retained','4:0:2:retained','29:0:3:retained','42:1:2:retained'];
export const config={blocks:36,passes:['default','single','single','default'],cpu:6,gcEvery:8,
 qualificationIterations:[1,8],emptyControlsPerEnd:128,
 metrics:['elapsed_ns','thread_cpu_ns','process_cpu_ns'],primaryComparison:'demand/eager',
 statistics:'20,000-replicate corrected v60 bootstrap and separate binomial order-statistic median interval; per pass, unadjusted',
 selection:'Eight authored diagnostic groups selected from earlier 59/60 observations before this replay. Not a random sample or confirmation cohort: unchanged-path noisy controls, known Unknown benefit, and three lost-witness retry cases.',
 inference:'No exclusion of slow observations; no subtraction of empty-envelope overhead; no pooling passes. GC settings are ABBA at process granularity, not independently randomized interventions. Thread/process CPU envelopes include instrumentation outside the inner wall clock. Resource counters enclose both CPU envelopes. No causal attribution or population-wide performance claim.'};
export const paths=pass=>({raw:`results/point-attribution-${pass}-v65.jsonl`,
 controls:`results/point-attribution-${pass}-controls-v65.jsonl`,
 failures:`results/point-attribution-${pass}-failures-v65.jsonl`,
 summary:`point-attribution-${pass}-v65.json`});
export function plan(){
 const old=json('point-wasm-cost-summary.json'),binding=wasmBindings();
 const groups=keys.map(key=>{
  const g=old.summaries.find(g=>groupKey(g)===key);assert(g);
  return{case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,iterations:g.iterations};
 });
 assert.equal(new Set(keys).size,8);
 return{schema:1,config,groups,binaries:binding.artifacts,
  files:Object.fromEntries(['point-attribution-protocol-v65.mjs','run-point-attribution-v65.mjs','capture.mjs',
   'point-wasm-protocol.mjs','point-wasm-binaries.json','point-wasm-cost-summary.json',
   'point-demand-source-binding.json','paired-statistics-v60.mjs','statistical-matches-v65.json'].map(p=>[p,sha(p)]))};
}
export function checkedPlan(){const p=json('point-attribution-plan-v65.json');assert.deepEqual(p.plan,plan());return p;}
export function envelope(fn){
 const r0=process.resourceUsage(),p0=process.cpuUsage(),t0=process.threadCpuUsage();
 const begin=process.hrtime.bigint();
 const value=fn();
 const elapsed_ns=Number(process.hrtime.bigint()-begin);
 const t=process.threadCpuUsage(t0),p=process.cpuUsage(p0),r=process.resourceUsage();
 return{value,elapsed_ns,thread_cpu_ns:1000*(t.user+t.system),process_cpu_ns:1000*(p.user+p.system),
  threadUserUs:t.user,threadSystemUs:t.system,processUserUs:p.user,processSystemUs:p.system,
  resources:Object.fromEntries(['voluntaryContextSwitches','involuntaryContextSwitches','minorPageFault','majorPageFault'].map(k=>[k,r[k]-r0[k]]))};
}
export function validateCounters(row,empty=false){
 for(const k of config.metrics)assert(Number.isSafeInteger(row[k])&&row[k]>=(empty?0:1),k);
 for(const k of ['threadUserUs','threadSystemUs','processUserUs','processSystemUs'])assert(Number.isSafeInteger(row[k])&&row[k]>=0,k);
 assert.equal(row.thread_cpu_ns,1000*(row.threadUserUs+row.threadSystemUs));
 assert.equal(row.process_cpu_ns,1000*(row.processUserUs+row.processSystemUs));
 for(const v of Object.values(row.resources))assert(Number.isSafeInteger(v)&&v>=0);
 assert.deepEqual(Object.keys(row.resources),['voluntaryContextSwitches','involuntaryContextSwitches','minorPageFault','majorPageFault']);
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 assert(process.argv.includes('--record'));
 const result={registered:new Date().toISOString(),plan:plan()};
 writeFileSync('point-attribution-plan-v65.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({registered:result.registered,groups:keys.length,config}));
}
