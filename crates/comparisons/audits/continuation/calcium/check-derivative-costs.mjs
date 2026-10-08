import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export function costs() {
  const binaries=json('derivative-cost-binaries.json');
  const variants=['baseline','candidate'],metrics=['requests','requested_bytes','live_delta','peak_delta'],summaries={};
  let seed=9261;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  for(const mode of ['cpu','allocation']) {
    const s=json('derivative-cost-'+mode+'-summary.json');
    const g=json('results/derivative-cost-'+mode+'.json');
    assert.equal(g.code,0);assert.equal(g.signal,null);assert.equal(s.mode,mode);assert.equal(s.cpu,6);
    assert.deepEqual(s.binaries,binaries);assert.equal(s.summaries.length,80);
    assert.equal(read('results/derivative-cost-'+mode+'.stderr'),'');
    assert.deepEqual(read('results/derivative-cost-'+mode+'.stdout').trimEnd().split('\n').map(JSON.parse),s.summaries);
    const begin=Date.parse(s.started),end=Date.parse(s.finished);
    assert(Date.parse(g.started)<=begin&&end<=Date.parse(g.finished)&&begin<=end);
    const rows=read('results/derivative-cost-'+mode+'.jsonl').trimEnd().split('\n').map(JSON.parse);
    assert.equal(rows.length,mode==='cpu'?3840:480);
    let index=0,group=0,last=begin;
    function validate(r,kind,degree,order,lifecycle,iterations,variant) {
      for(const[k,v]of Object.entries({mode,kind,degree,order,lifecycle,iterations,variant}))assert.equal(r[k],v);
      assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);
      const a=Date.parse(r.started),b=Date.parse(r.finished);assert(a>=last&&a<=b&&b<=end);last=b;
      for(const k of metrics)assert(Number.isSafeInteger(r[k]));
      if(mode==='cpu')for(const k of metrics)assert.equal(r[k],0);
      else {assert(r.requests>0&&r.requested_bytes>0&&r.peak_delta>0);assert(r.live_delta>=0);}
    }
    for(let kind=0;kind<2;kind++)for(const degree of [1,3,8,24])for(const order of [0,1,3,24,128])
    for(const lifecycle of ['retained_curve','fresh_curve']) {
      const q=s.summaries[group++];assert.deepEqual([q.kind,q.degree,q.order,q.lifecycle],[kind,degree,order,lifecycle]);
      assert.equal(q.pilots.length,2);q.pilots.forEach((r,i)=>validate(r,kind,degree,order,lifecycle,2,variants[i]));
      const iterations=mode==='cpu'?Math.max(2,Math.min(5000,Math.ceil(6e6/Math.max(...q.pilots.map(r=>r.elapsed_ns/2))))):8;
      assert.equal(q.iterations,iterations);const blocks=mode==='cpu'?12:3,local=[];
      for(let block=0;block<blocks;block++) {
        const runOrder=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
        for(const v of runOrder){const r=rows[index++];assert.equal(r.block,block);validate(r,kind,degree,order,lifecycle,iterations,v);local.push(r);}
      }
      assert.equal(q.observations,local.length);
      if(mode==='cpu') {
        const ratios=Array.from({length:blocks},(_,b)=>{
          const clock=v=>median(local.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));
          return clock('candidate')/clock('baseline');
        });
        const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
        assert.equal(q.pairedMedianRatio,median(ratios));assert.deepEqual(q.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
        assert.deepEqual(q.nsPerQuery,Object.fromEntries(variants.map(v=>[v,median(local.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))])));
      } else {
        const counts=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(metrics.map(k=>{
          const values=local.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];
        }))]));
        assert.deepEqual(q.measurements,counts);
        for(const v of variants)for(const k of metrics)assert.equal(counts[v][k].min,counts[v][k].max);
        // Warming eight queries is not steady state: preserve shared retention,
        // require the same measured live change in both variants, and report it.
        assert.deepEqual(counts.baseline.live_delta,counts.candidate.live_delta);
      }
    }
    assert.equal(index,rows.length);summaries[mode]=s;
  }
  assert(Date.parse(summaries.allocation.started)>=Date.parse(summaries.cpu.finished));
  const allocation={};
  for(const key of ['requests','requested_bytes','peak_delta']) {
    const delta=summaries.allocation.summaries.map(q=>q.measurements.candidate[key].min-q.measurements.baseline[key].min);
    allocation[key]={lower:delta.filter(v=>v<0).length,equal:delta.filter(v=>v===0).length,higher:delta.filter(v=>v>0).length};
  }
  const q=summaries.cpu.summaries;
  const faster=q.filter(v=>v.pairedMedianBootstrap95[1]<1),slower=q.filter(v=>v.pairedMedianBootstrap95[0]>1);
  return {groups:80,cpuRows:3840,allocationRows:480,allocation,
    nonzeroLiveGroups:summaries.allocation.summaries.filter(q=>q.measurements.baseline.live_delta.min!==0).length,
    maxSharedLiveDelta:Math.max(...summaries.allocation.summaries.map(q=>q.measurements.baseline.live_delta.min)),
    ratioRange:[Math.min(...q.map(v=>v.pairedMedianRatio)),Math.max(...q.map(v=>v.pairedMedianRatio))],
    intervalBelowOne:faster.length,intervalAboveOne:slower.length,
    slowerControls:slower.map(({kind,degree,order,lifecycle,pairedMedianRatio,pairedMedianBootstrap95})=>({kind,degree,order,lifecycle,pairedMedianRatio,pairedMedianBootstrap95}))};
}
