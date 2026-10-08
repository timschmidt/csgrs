import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {generator,boundedInteger} from './paired-statistics-v60.mjs';
import {cases} from './twelfth-cost-input-v78.mjs';
export const config={checkpoint:78,cpu:2,cpuPairs:24,allocationPairs:4,warmups:4,pilotIterations:[1,16],targetNs:2000000,maxIterations:2000,allocationIterations:[1,16],seed:'twelfth-native-v78'};
export const key=g=>[g.case,g.precision,g.lifecycle].join(':');
export const groups=(iterations=1)=>cases().flatMap(c=>[-64,-256].flatMap(precision=>['fresh','cold','retained'].map(lifecycle=>({case:c.id,precision,lifecycle,iterations}))));
export const rows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export const plan=(p,g)=>writeFileSync(p,JSON.stringify(g)+'\n',{flag:'wx'});
export function shuffle(list,label){const a=[...list],rng=generator(config.seed+':'+label);for(let i=a.length-1;i>0;i--){const j=boundedInteger(()=>rng.word(),i+1);[a[i],a[j]]=[a[j],a[i]];}return a;}
export function orders(n,label){assert.equal(n%2,0);return shuffle([...Array(n/2).fill('baseline'),...Array(n/2).fill('candidate')],label).map(v=>v==='baseline'?['baseline','candidate']:['candidate','baseline']);}
export function validate(path,expected,variant,mode){
 const observed=rows(path),input=cases();assert.equal(observed.length,expected.length+1);assert.deepEqual(observed.at(-1),{terminal:true,mode,groups:expected.length});
 let previousEnd=0n;
 for(let i=0;i<expected.length;i++){
  const g=expected[i],r=observed[i];for(const[k,v]of Object.entries(g))assert.equal(r[k],v,'group '+i+' '+k);
  assert.equal(r.mode,mode);assert.equal(r.final_matches,true);assert([0,1,2].includes(r.outcome));
  if(g.case<32)assert.equal(r.outcome,variant==='baseline'?2:0);
  else if(g.case<64||input[g.case].truth==='not-equal')assert.equal(r.outcome,1);
  else if(g.case<84)assert.equal(r.outcome,0);
  else assert.notEqual(r.outcome,1);
  const counts=[0,0,0];counts[r.outcome]=g.iterations;assert.deepEqual(r.counts,counts);
  assert(typeof r.certificate==='string'&&r.certificate.startsWith(['Equal','NotEqual','Unknown'][r.outcome]));
  assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);const begin=BigInt(r.wall_before),end=BigInt(r.wall_after);
  assert(begin>=previousEnd&&end>=begin&&end-begin>=BigInt(r.elapsed_ns));previousEnd=end;
  for(const n of ['requests','requested_bytes','start_live','end_live','peak_delta'])assert(Number.isSafeInteger(r[n])&&r[n]>=0);
  assert(Number.isSafeInteger(r.live_delta));assert.equal(r.live_delta,r.end_live-r.start_live);assert(r.peak_delta>=r.live_delta);
  if(mode==='cpu')for(const n of ['requests','requested_bytes','start_live','end_live','live_delta','peak_delta'])assert.equal(r[n],0);
 }return observed.slice(0,-1);
}
