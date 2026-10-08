import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {generator,boundedInteger} from './paired-statistics-v60.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
import {reference} from './zero-factor-cost-protocol-v71.mjs';
import {commonRecord,flags} from './zero-factor-wasm-runner-v72.mjs';
export {flags};
export const config={checkpoint:73,cpu:2,orchestratorCpu:0,pairs:24,instanceModes:['persistent','fresh'],
 pilotIterations:[1,32],targetNs:3000000,maxIterations:20000,warmups:8,gcEvery:8,seed:'zero-factor-wasm-cost-v73',flags,
 limits:'Warmed-query batches, not cold module/process startup. Fresh source and fresh instance are distinct. All rows retained; conditional, non-multiplicity-adjusted intervals.'};
export const key=g=>[g.case,g.policy,g.lifecycle].join(':');
export const groups=(iterations=1)=>Array.from({length:114},(_,which)=>[0,1].flatMap(policy=>['retained','fresh'].map(lifecycle=>({case:which,policy,lifecycle,iterations})))).flat();
export function shuffle(list,label){const a=[...list],rng=generator(config.seed+':'+label);
 for(let i=a.length-1;i>0;i--){const j=boundedInteger(()=>rng.word(),i+1);[a[i],a[j]]=[a[j],a[i]];}return a;}
export const orders=label=>shuffle([...Array(12).fill('baseline'),...Array(12).fill('candidate')],label).map(v=>v==='baseline'?['baseline','candidate']:['candidate','baseline']);
export const modeOrders=()=>shuffle([...Array(12).fill('persistent'),...Array(12).fill('fresh')],'mode-order').map(v=>v==='persistent'?['persistent','fresh']:['fresh','persistent']);
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
let cases;
export function validateRow(row,group,variant,instanceMode,index){
 cases??=costCases();for(const[k,v]of Object.entries(group))assert.equal(row[k],v,k);
 assert.equal(row.variant,variant);assert.equal(row.instanceMode,instanceMode);assert.equal(row.mode,'cpu');assert.equal(row.kind,'rational');
 assert(!('qualification_batch_ns'in row));assert(Number.isSafeInteger(row.elapsed_ns)&&row.elapsed_ns>0);
 commonRecord({...row,mode:'qualification',qualification_batch_ns:row.elapsed_ns},group,group.iterations);
 assert.equal(row.sequence,instanceMode==='persistent'?index:0);assert.deepEqual(row.expected,reference(cases[group.case],group.policy,variant));
 for(const name of ['initialMemory','initializedMemory','memoryBeforePrepare','memoryAfterPrepare','memoryAfterBatch','memoryAfterFinish'])
  assert(Number.isSafeInteger(row[name])&&row[name]>0&&row[name]%65536===0,name);
}
export function validateRows(path,expected,variant,instanceMode){
 const rows=readRows(path);assert.equal(rows.length,expected.length+1);const terminal=rows.pop();
 assert.equal(terminal.terminal,true);assert.equal(terminal.checkpoint,73);assert.equal(terminal.variant,variant);assert.equal(terminal.instanceMode,instanceMode);
 assert.equal(terminal.groups,expected.length);assert.equal(terminal.gcs,Math.floor(expected.length/config.gcEvery));
 assert.deepEqual(terminal.flags,flags);assert.equal(terminal.affinity,'2');assert.equal(terminal.node,'v22.22.2');assert.equal(terminal.v8,'12.4.254.21-node.39');
 let previous=0,previousMemory;
 for(const [i,r]of rows.entries()){
  validateRow(r,expected[i],variant,instanceMode,i);assert(Date.parse(r.started)>=previous);previous=Date.parse(r.finished);
  if(instanceMode==='persistent'&&previousMemory){assert(r.memoryBeforePrepare>=previousMemory.memoryAfterFinish);
   assert.equal(r.initialMemory,previousMemory.initialMemory);assert.equal(r.initializedMemory,previousMemory.initializedMemory);}
  previousMemory=r;
 }
 assert(Date.parse(terminal.finished)>=previous);return{rows,terminal};
}
