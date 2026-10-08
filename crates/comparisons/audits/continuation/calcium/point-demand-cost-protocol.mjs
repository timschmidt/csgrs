import assert from 'node:assert/strict';
import {config as historyConfig,groupKey,frame,iterationsFor} from './point-history-protocol.mjs';
export const variants=['baseline','eager','demand'];
export const config={...historyConfig,variants,cpuOrder:'six permutations each followed by its reverse, repeated twice',allocationOrder:'three rotations'};
const permutations=[[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]];
export function orderFor(mode,block){
 if(mode==='allocation')return variants.map((_,i)=>variants[(i+block)%3]);
 const p=permutations[block%6].map(i=>variants[i]);return[...p,...p.toReversed()];
}
const median=a=>{const v=[...a].sort((a,b)=>a-b);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export const counters=['requests','requested_bytes','return_live_delta','peak_delta'];
export function summarizer(){
 let seed=532825;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 return (mode,group,rows,pilots)=>{
  const blocks=mode==='cpu'?config.cpuBlocks:config.allocationBlocks,perBlock=mode==='cpu'?6:3;
  assert.equal(rows.length,blocks*perBlock);assert.equal(pilots.length,mode==='cpu'?3:0);
  const iterations=mode==='cpu'?iterationsFor(pilots):config.allocationIterations;
  const expected=Object.fromEntries(variants.map(v=>[v,frame(rows.find(r=>r.variant===v))]));
  for(const row of rows){assert.equal(groupKey(row),groupKey(group));assert.equal(row.iterations,iterations);assert.deepEqual(frame(row),expected[row.variant]);}
  if(mode==='cpu'){
   assert.deepEqual(pilots.map(r=>r.variant),variants);
   for(const p of pilots){assert.equal(groupKey(p),groupKey(group));assert.equal(p.iterations,config.cpuPilotIterations);assert.deepEqual(frame(p),expected[p.variant]);}
  }
  for(let block=0;block<blocks;block++){
   const rs=rows.slice(block*perBlock,(block+1)*perBlock);assert(rs.every(r=>r.block===block));assert.deepEqual(rs.map(r=>r.variant),orderFor(mode,block));
  }
  assert.deepEqual(expected.eager,expected.demand);
  assert.deepEqual(expected.baseline.left,expected.demand.left);assert.deepEqual(expected.baseline.right,expected.demand.right);
  const s={...group,iterations,observations:rows.length,statuses:Object.fromEntries(variants.map(v=>[v,expected[v].report.status])),comparisons:{}};
  for(const reference of ['baseline','eager']){
   const c={sameFullResult:JSON.stringify(expected[reference].report)===JSON.stringify(expected.demand.report)};
   if(mode==='cpu'){
    const ratios=Array.from({length:blocks},(_,block)=>{
     const clock=v=>median(rows.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));return clock('demand')/clock(reference);
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
    Object.assign(c,{pairedBlockRatios:ratios,pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]]});
   }
   s.comparisons[reference]=c;
  }
  if(mode==='cpu')s.nsPerQuery=Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]));
  else s.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(counters.map(k=>{
   const values=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];}))]));
  return s;
 };
}
