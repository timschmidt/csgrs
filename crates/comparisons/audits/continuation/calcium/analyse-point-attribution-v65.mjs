import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {isDeepStrictEqual} from 'node:util';
import assert from 'node:assert/strict';
import {checkedPlan,config,paths,flags,variants,orderFor,groupKey,sha,json,validateCounters} from './point-attribution-runtime-v65.mjs';
import {nativeExpected,frame,validateWasmRecord} from './point-wasm-protocol.mjs';
import {median,correctedPairedStatistics,statisticsSelfTest} from './paired-statistics-v60.mjs';
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').filter(Boolean).map(JSON.parse);
export function validateRows(rows,pass,plan,expected){
 assert.equal(rows.length,1776);let index=0,lastFinished=0;
 function check(g,variant,iterations,phase,block,position){
  const row=rows[index];
  assert.equal(row.observation,index);assert.equal(row.gcsBefore,Math.floor(index/8));
  validateWasmRecord(row,{...g,variant,iterations,phase,pass,setting:config.passes[pass],block,position});
  validateCounters(row);assert.equal(row.value,row.host_checksum);
  assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(g)));
  assert(Date.parse(row.started)>=lastFinished);assert(Date.parse(row.finished)>=Date.parse(row.started));
  assert(row.elapsed_ns<=(Date.parse(row.finished)-Date.parse(row.started)+1)*1e6);
  lastFinished=Date.parse(row.finished);index++;
 }
 for(const{iterations:unused,...g}of plan.groups)for(const v of variants)for(const n of config.qualificationIterations)
  check(g,v,n,'qualification',-1,-1);
 for(let block=0;block<config.blocks;block++)for(const{iterations,...g}of plan.groups)
  for(const[position,v]of orderFor('cpu',block).entries())check(g,v,iterations,'measurement',block,position);
 assert.equal(index,1776);
}
export function paired(blocks,reference,metric,label){
 const pairs=blocks.map(rs=>Object.fromEntries(['demand',reference].map(v=>[v,median(rs.filter(r=>r.variant===v).map(r=>r[metric]))])));
 assert.equal(pairs.length,36);
 if(pairs.some(p=>p.demand===0||p[reference]===0))return{available:false,blockMedians:pairs,
  reason:'At least one zero block median; all observations retained, whole metric inference unavailable.'};
 return{available:true,blockMedians:pairs,...correctedPairedStatistics(pairs.map(p=>p.demand/p[reference]),label)};
}
const describe=xs=>{const s=xs.toSorted((a,b)=>a-b);return{min:s[0],median:median(s),p95:s[Math.floor(.95*(s.length-1))],max:s.at(-1)};};
function diagnostic(rs){
 const positive=rs.filter(r=>r.thread_cpu_ns>0);
 return{observations:rs.length,zeroThread:rs.length-positive.length,
  zeroProcess:rs.filter(r=>r.process_cpu_ns===0).length,
  perQuery:Object.fromEntries(config.metrics.map(k=>[k,describe(rs.map(r=>r[k]/r.iterations))])),
  wallOverThread:positive.length?describe(positive.map(r=>r.elapsed_ns/r.thread_cpu_ns)):null,
  processOverThread:positive.length?describe(positive.map(r=>r.process_cpu_ns/r.thread_cpu_ns)):null,
  wallOverThreadOver125:positive.filter(r=>r.elapsed_ns>1.25*r.thread_cpu_ns).length,
  excessWithInvoluntarySwitch:positive.filter(r=>r.elapsed_ns>1.25*r.thread_cpu_ns&&r.resources.involuntaryContextSwitches>0).length,
  resourceSums:Object.fromEntries(Object.keys(rs[0].resources).map(k=>[k,rs.reduce((n,r)=>n+r.resources[k],0)]))};
}
export function analyse(log=false){
 const registered=checkedPlan(),plan=registered.plan,expected=nativeExpected(),passes=[];
 const initial=json('results/point-attribution-0.json');assert.equal(initial.code,1);assert.equal(initial.signal,null);
 assert(Date.parse(initial.finished)<Date.parse(registered.registered));
 assert.equal(readFileSync('results/point-attribution-0.stdout','utf8'),'');
 const initialRows=readRows('results/point-attribution-0-v65.jsonl');assert.equal(initialRows.length,1);
 assert.equal(initialRows[0].phase,'qualification');assert.equal(initialRows[0].thread_cpu_ns,0);
 assert(initialRows[0].process_cpu_ns>0);assert.deepEqual(frame(initialRows[0]),expected.get('baseline:12:0:2:retained'));
 assert.equal(readRows('results/point-attribution-0-controls-v65.jsonl').length,128);
 assert.equal(readRows('results/point-attribution-0-failures-v65.jsonl').length,1);
 assert.match(readFileSync('results/point-attribution-0.stderr','utf8'),/AssertionError.*thread_cpu_ns/);
 let previousFinished=registered.registered;
 for(let pass=0;pass<4;pass++){
  const p=paths(pass),s=json(p.summary),gate=json(`results/point-attribution-revised-${pass}.json`),rows=readRows(p.raw);
  assert.equal(s.status,'pass');assert.equal(s.pass,pass);assert.equal(s.setting,config.passes[pass]);
  assert.equal(s.planSha256,sha('point-attribution-revised-plan-v65.json'));assert.deepEqual(s.config,config);
  assert.deepEqual([s.qualification,s.measured,s.observations,s.gcs],[48,1728,1776,222]);
  for(const[f,h]of Object.entries(s.files))assert.equal(sha(f),h,f);
  assert.deepEqual(Object.keys(s.files),[p.raw,p.controls,p.failures]);assert.equal(readFileSync(p.failures).length,0);
  assert.equal(gate.code,0);assert.equal(gate.signal,null);assert.equal(gate.command,'taskset');assert.equal(gate.cwd,resolve('.'));
  assert.deepEqual(gate.args,['-c','6','node',...flags,...(s.setting==='single'?['--single-threaded-gc']:[]),'run-point-attribution-revised-v65.mjs',String(pass)]);
  assert(Date.parse(gate.started)>=Date.parse(previousFinished));assert(Date.parse(s.started)>=Date.parse(gate.started));
  assert(Date.parse(s.finished)>=Date.parse(s.started));assert(Date.parse(gate.finished)>=Date.parse(s.finished));previousFinished=gate.finished;
  assert.equal(readFileSync(`results/point-attribution-revised-${pass}.stderr`).length,0);
  const stdout=readRows(`results/point-attribution-revised-${pass}.stdout`);assert.equal(stdout.length,7);
  for(let i=0;i<6;i++)assert.deepEqual(Object.fromEntries(Object.entries(stdout[i]).filter(([k])=>k!=='finished')),
   {pass,setting:s.setting,blocks:6*(i+1),measured:288*(i+1)});
  assert.deepEqual(stdout.at(-1),{status:'pass',pass,setting:s.setting,qualification:48,measured:1728,gcs:222,finished:s.finished});
  assert.deepEqual(s.runtime,{node:'v22.22.2',v8:s.runtime.v8,execArgv:gate.args.slice(3,-2),affinity:'6',binaries:plan.binaries});
  if(pass)assert.equal(s.runtime.v8,passes[0].runtime.v8);
  validateRows(rows,pass,plan,expected);
  assert(Date.parse(rows[0].started)>=Date.parse(s.started));assert(Date.parse(rows.at(-1).finished)<=Date.parse(s.finished));
  const controls=readRows(p.controls);assert.equal(controls.length,256);
  for(const[i,r]of controls.entries()){
   assert.equal(r.pass,pass);assert.equal(r.setting,s.setting);assert.equal(r.end,i<128?'before':'after');
   assert.equal(r.index,i%128);assert.equal(r.value,0);validateCounters(r,true);
  }
  const measured=rows.filter(r=>r.phase==='measurement'),groups=plan.groups.map(g=>{
   const rs=measured.filter(r=>groupKey(r)===groupKey(g));assert.equal(rs.length,216);
   const blocks=Array.from({length:36},(_,b)=>rs.filter(r=>r.block===b));
   const comparisons=Object.fromEntries(['baseline','eager'].map(reference=>[reference,{
    sameFullResult:isDeepStrictEqual(expected.get(reference+':'+groupKey(g)).report,expected.get('demand:'+groupKey(g)).report),
    metrics:Object.fromEntries(config.metrics.map(metric=>[metric,paired(blocks,reference,metric,`point-attribution-v65:${pass}:${groupKey(g)}:${reference}:${metric}`)]))}]));
   assert(comparisons.eager.sameFullResult);
   return{...g,statuses:Object.fromEntries(variants.map(v=>[v,rs.find(r=>r.variant===v).actual.status])),comparisons,
    diagnostics:Object.fromEntries(variants.map(v=>[v,diagnostic(rs.filter(r=>r.variant===v))]))};
  });
  const result={pass,setting:s.setting,started:s.started,finished:s.finished,runtime:s.runtime,groups,
   emptyEnvelope:Object.fromEntries(config.metrics.map(k=>[k,describe(controls.map(r=>r[k]))])),
   diagnostics:diagnostic(measured),sinceGc:Array.from({length:8},(_,position)=>({position,...diagnostic(measured.filter(r=>r.observation%8===position))})),
   files:{[p.summary]:sha(p.summary),...s.files,...Object.fromEntries(['json','stdout','stderr'].map(ext=>{
    const f=`results/point-attribution-revised-${pass}.${ext}`;return[f,sha(f)];}))}};
  passes.push(result);if(log)console.log(JSON.stringify({pass,groups:8,measured:1728,qualification:48,status:'pass'}));
 }
 assert.deepEqual(checkedPlan(),registered);
 return{checkpoint:65,status:'pass',planSha256:sha('point-attribution-revised-plan-v65.json'),config,
  measured:6912,qualification:192,emptyControls:1024,initialFailure:{measured:0,qualificationRows:1,emptyControls:128,
   cause:'Incorrectly required strictly positive CPU counters for a short qualification query; zero is preserved, not a numerical failure.'},
  passes,tests:statisticsSelfTest(),limits:config.inference+' Positive-only wall/thread diagnostic ratios explicitly report zero counts and are descriptive, not timing-sample exclusions or inferential evidence. Shared-process CPU includes helpers and broader instrumentation. Final full reports agree with frozen native records, not newly independent algebraic oracles. No consumer, binary-size, memory-allocation or retention gate.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const result=analyse(true);
 writeFileSync('point-attribution-analysis-v65.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:65,status:'pass',measured:result.measured,qualification:result.qualification,emptyControls:result.emptyControls}));
}
