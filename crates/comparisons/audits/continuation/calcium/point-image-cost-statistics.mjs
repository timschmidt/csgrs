import assert from 'node:assert/strict';
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export function summarizePointImageCosts(mode,rows,pilots){
 const variants=['baseline','candidate'],summaries=[];let seed=532825;
 const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 assert.equal(rows.length,mode==='cpu'?3840:480);assert.equal(pilots.length,mode==='cpu'?160:0);
 for(let which=0;which<40;which++)for(const lifecycle of ['retained','fresh']){
  const group=rows.filter(r=>r.case===which&&r.lifecycle===lifecycle),blocks=mode==='cpu'?12:3;
  assert.equal(group.length,mode==='cpu'?48:6);
  const iterations=group[0].iterations;assert(group.every(r=>r.iterations===iterations));
  for(let block=0;block<blocks;block++)for(const variant of variants)
   assert.equal(group.filter(r=>r.block===block&&r.variant===variant).length,mode==='cpu'?2:1);
  const expected=Object.fromEntries(variants.map(v=>[v,group.find(r=>r.variant===v).expected]));
  for(const r of group)assert.deepEqual(r.expected,expected[r.variant]);
  const sameFullResult=JSON.stringify(expected.baseline)===JSON.stringify(expected.candidate);
  const s={case:which,lifecycle,iterations,observations:group.length,sameFullResult,
   statuses:Object.fromEntries(variants.map(v=>[v,expected[v].status]))};
  if(mode==='cpu'){
   const p=pilots.filter(r=>r.case===which&&r.lifecycle===lifecycle);assert.equal(p.length,2);
   assert.equal(iterations,Math.max(10,Math.min(10000,Math.ceil(8e6/Math.max(...p.map(r=>r.elapsed_ns/r.iterations))))));
   const ratios=Array.from({length:blocks},(_,block)=>{
    const clock=v=>median(group.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');});
   const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
   Object.assign(s,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
    nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(group.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
  }else s.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>{
   const values=group.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];}))]));
  summaries.push(s);
 }
 return summaries;
}
