import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {generator,boundedInteger} from './paired-statistics-v60.mjs';
import {costCases} from './zero-factor-cost-input-v71.mjs';
export const config={checkpoint:71,cpu:2,cpuPairs:24,allocationPairs:4,warmups:8,pilotIterations:[1,32],
 targetNs:3000000,maxIterations:20000,allocationIterations:[1,16],seed:'zero-factor-native-v71',
 interpretation:'Fresh source in warmed process; new answers are not equal-work speedups. All samples retained, intervals conditional and not multiplicity-adjusted.'};
export const key=g=>[g.case,g.policy,g.lifecycle].join(':');
export function groups(iterations=1){return costCases().flatMap(c=>[0,1].flatMap(policy=>['retained','fresh'].map(lifecycle=>({case:c.id,policy,lifecycle,iterations}))));}
export function shuffle(list,label){
 const a=[...list],rng=generator(config.seed+':'+label);
 for(let i=a.length-1;i>0;i--){const j=boundedInteger(()=>rng.word(),i+1);[a[i],a[j]]=[a[j],a[i]];}return a;
}
export function variantOrders(n,label){assert(n%2===0);return shuffle([...Array(n/2).fill('baseline'),...Array(n/2).fill('candidate')],label)
 .map(first=>first==='baseline'?['baseline','candidate']:['candidate','baseline']);}
export const readRows=p=>readFileSync(p,'utf8').trimEnd().split('\n').map(JSON.parse);
export function plan(path,rows){writeFileSync(path,JSON.stringify(rows)+'\n',{flag:'wx'});}
const referenceCache=new Map();
export function reference(c,policy,variant){
 const source=c.source;let tag;
 if(source.corpus==='public')tag=variant==='baseline'?'power-rebased-baseline-'+(policy?'approx':'public')+'-v68':'zero-factor-'+(policy?'approx':'public')+'-run-v70';
 else if(source.corpus==='degree')tag=variant==='baseline'?'zero-factor-degree-baseline-v70':'zero-factor-degree-run-v70';
 else tag=variant==='baseline'?'power-wide-baseline-run-v69':'zero-factor-wide-run-v70';
 if(!referenceCache.has(tag))referenceCache.set(tag,readRows('results/'+tag+'.stdout'));
 const rows=referenceCache.get(tag),r=source.corpus==='public'?rows.find(r=>r.type==='cost-case'&&r.case===source.case):rows.find(r=>r.id===source.id&&r.policy===policy);
 assert(r);assert.deepEqual(r.left,c.left);assert.deepEqual(r.right,c.right);assert.equal(r.report.operation,c.operation);return r.report;
}
export function validateRows(path,expectedGroups,variant,mode){
 const rows=readRows(path),cases=costCases();assert.equal(rows.length,expectedGroups.length+1);
 assert.deepEqual(rows.at(-1),{terminal:true,groups:expectedGroups.length,mode});
 let previousEnd=0n;
 for(let i=0;i<expectedGroups.length;i++){
  const r=rows[i],g=expectedGroups[i];for(const[k,v]of Object.entries(g))assert.equal(r[k],v);
  assert.equal(r.mode,mode);assert.equal(r.final_matches,true);assert.deepEqual(r.expected,reference(cases[g.case],g.policy,variant));
  assert.equal(r.checksum,g.iterations*(r.expected.root?.polynomial.length??1));assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);
  const begin=BigInt(r.wall_before),end=BigInt(r.wall_after);assert(begin>=previousEnd&&end>=begin);assert(BigInt(r.elapsed_ns)<=end-begin);previousEnd=end;
  for(const name of ['requests','requested_bytes','start_live','end_live','peak_delta'])assert(Number.isSafeInteger(r[name])&&r[name]>=0);
  assert(Number.isSafeInteger(r.live_delta));assert.equal(r.live_delta,r.end_live-r.start_live);assert(r.peak_delta>=r.live_delta);
  if(mode==='cpu')for(const name of ['requests','requested_bytes','start_live','end_live','live_delta','peak_delta'])assert.equal(r[name],0);
 }return rows.slice(0,-1);
}
