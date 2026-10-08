import assert from 'node:assert/strict';
import {config,variants,groupKey,frame,iterationsFor,orderFor} from './point-history-protocol.mjs';
const median=a=>{const v=[...a].sort((a,b)=>a-b);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export function summarizer(){
 let seed=532825;
 const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 return function summarize(mode,group,rows,pilots){
  const blocks=mode==='cpu'?config.cpuBlocks:config.allocationBlocks;
  assert.equal(rows.length,blocks*(mode==='cpu'?4:2));
  assert.equal(pilots.length,mode==='cpu'?2:0);
  const iterations=mode==='cpu'?iterationsFor(pilots):config.allocationIterations;
  const expected=Object.fromEntries(variants.map(v=>[v,frame(rows.find(r=>r.variant===v))]));
  for(const r of rows){assert.equal(groupKey(r),groupKey(group));assert.equal(r.iterations,iterations);assert.deepEqual(frame(r),expected[r.variant]);}
  if(mode==='cpu'){
   assert.deepEqual(pilots.map(r=>r.variant),variants);
   for(const p of pilots){assert.equal(groupKey(p),groupKey(group));assert.equal(p.iterations,config.cpuPilotIterations);assert.deepEqual(frame(p),expected[p.variant]);}
  }
  for(let block=0;block<blocks;block++){
   const r=rows.slice(block*(mode==='cpu'?4:2),(block+1)*(mode==='cpu'?4:2));
   assert(r.every(r=>r.block===block));assert.deepEqual(r.map(r=>r.variant),orderFor(mode,block));
  }
  assert.deepEqual(expected.baseline.left,expected.candidate.left);assert.deepEqual(expected.baseline.right,expected.candidate.right);
  const sameFullResult=JSON.stringify(expected.baseline.report)===JSON.stringify(expected.candidate.report);
  const s={...group,iterations,observations:rows.length,sameFullResult,
   statuses:Object.fromEntries(variants.map(v=>[v,expected[v].report.status]))};
  if(mode==='cpu'){
   const ratios=Array.from({length:blocks},(_,block)=>{
    const clock=v=>median(rows.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));
    return clock('candidate')/clock('baseline');
   });
   const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
   Object.assign(s,{pairedBlockRatios:ratios,pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
    nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
  }else s.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','return_live_delta','peak_delta'].map(k=>{
   const values=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];}))]));
  return s;
 };
}
