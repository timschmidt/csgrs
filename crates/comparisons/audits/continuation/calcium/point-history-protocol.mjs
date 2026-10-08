import {spawn} from 'node:child_process';
import {readFileSync,statSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
export const variants=['baseline','candidate'];
export const modes=['cpu','allocation'];
export const config=Object.freeze({cpu:6,cases:48,policies:2,histories:4,lifecycles:['retained','fresh'],
 preconditioningCalls:9,cpuPilotIterations:16,targetBatchNs:4e6,minIterations:4,maxIterations:4096,
 cpuBlocks:12,allocationBlocks:3,allocationIterations:8,qualificationIterations:{cpu:1,allocation:8}});
export const groups=()=>Array.from({length:48},(_,which)=>which).flatMap(which=>[0,1].flatMap(policy=>
 [0,1,2,3].flatMap(history=>config.lifecycles.map(lifecycle=>({case:which,policy,history,lifecycle})))));
export const groupKey=r=>[r.case,r.policy,r.history,r.lifecycle].join(':');
export const frame=r=>({type:'query',case:r.case,policy:r.policy,history:r.history,left:r.left,right:r.right,report:r.actual});
export const lines=p=>readFileSync(p,'utf8').trim().split('\n').filter(Boolean).map(JSON.parse);
export const iterationsFor=pilots=>Math.max(config.minIterations,Math.min(config.maxIterations,
 Math.ceil(config.targetBatchNs/Math.max(...pilots.map(r=>r.elapsed_ns/r.iterations)))));
export const orderFor=(mode,block)=>mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
export function checkBindings(){
 sources();const b=json('point-history-binaries.json');assert.equal(b.artifacts.length,4);
 assert.equal(b.sourceOriginSha256,sha('point-qualified-origin.json'));
 for(const[p,h]of Object.entries(b.harnessSources))assert.equal(sha(p),h,p);
 for(const variant of variants)for(const mode of modes){
  const found=b.artifacts.filter(a=>a.variant===variant&&a.mode===mode);assert.equal(found.length,1);
  const a=found[0];assert.equal(sha(a.path),a.sha256);assert.equal(statSync(a.path).size,a.bytes);
 }
 for(const variant of variants)for(const gate of ['lock','build','clippy']){
  const g=json('results/point-history-'+variant+'-'+gate+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 }
 return b;
}
export function validateRecord(row,expected){
 for(const[k,v]of Object.entries(expected))assert.equal(row[k],v,k);
 assert(Number.isSafeInteger(row.iterations)&&row.iterations>0);
 assert(Number.isSafeInteger(row.elapsed_ns)&&row.elapsed_ns>0);
 assert.equal(row.final_matches_preconditioned,true);assert.equal(row.input_records_unchanged,true);
 assert.equal(row.checksum,row.iterations*(row.actual.root?.polynomial.length??1));
 for(const k of ['requests','requested_bytes','return_live_delta','peak_delta']){
  assert(Number.isSafeInteger(row[k]),k);if(k!=='return_live_delta')assert(row[k]>=0,k);
  if(row.mode==='cpu')assert.equal(row[k],0,k);
 }
}
export async function observe(binding,variant,mode,group,iterations,failurePath){
 const b=binding.artifacts.find(b=>b.variant===variant&&b.mode===mode);
 const args=['-c',String(config.cpu),b.path,String(group.case),String(group.policy),String(group.history),group.lifecycle,String(iterations),mode];
 const started=new Date().toISOString();
 const termination=await new Promise((ok,fail)=>{
  const p=spawn('taskset',args,{stdio:['ignore','pipe','pipe']});let stdout='',stderr='';
  p.stdout.on('data',s=>stdout+=s);p.stderr.on('data',s=>stderr+=s);p.on('error',fail);
  p.on('close',(code,signal)=>ok({code,signal,stdout,stderr}));
 });
 const finished=new Date().toISOString();
 if(termination.code!==0||termination.signal!==null||termination.stderr!==''){
  appendFileSync(failurePath,JSON.stringify({variant,mode,group,iterations,started,finished,command:'taskset',args,...termination})+'\n');
  throw Error('Child failed; preserved complete output in '+failurePath);
 }
 const row={variant,started,finished,...JSON.parse(termination.stdout)};
 // The caller writes the actual full row before asserting its contents.
 return row;
}
