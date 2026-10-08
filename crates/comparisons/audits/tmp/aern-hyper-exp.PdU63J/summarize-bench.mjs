import {readFileSync} from 'node:fs';
const rows=readFileSync(process.argv[2] || '/tmp/aern-hyper-exp-ab.jsonl','utf8').trim().split('\n').map(JSON.parse);
const median=xs=>{ const a=[...xs].sort((a,b)=>a-b); return a.length%2?a[a.length>>1]:(a[a.length/2-1]+a[a.length/2])/2; };
let state=987654321;
const rng=()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state/2**32;};
for(const name of [...new Set(rows.map(r=>r.case))]) {
  const pairs=rows.filter(r=>r.kind==='pair'&&r.case===name);
  if(pairs.length!==15) throw Error(`incomplete case ${name}: ${pairs.length} pairs`);
  const averages=pairs.map(row=>['baseline','candidate'].map(side=>{
    const samples=row.samples.filter(s=>s.side===side);
    return samples.reduce((sum,s)=>sum+s.ns/s.iterations,0)/samples.length;
  }));
  const ratios=averages.map(([a,b])=>b/a);
  const boot=Array.from({length:10000},()=>median(Array.from({length:ratios.length},()=>ratios[Math.floor(rng()*ratios.length)]))).sort((a,b)=>a-b);
  const allocation=rows.filter(r=>r.kind==='allocation'&&r.case===name).map(r=>({
    side:r.side,calls:r.allocations/r.iterations,bytes:r.requested_bytes/r.iterations,
  }));
  console.log(JSON.stringify({case:name,blocks:pairs.length,
    baseline_ns:median(averages.map(a=>a[0])),candidate_ns:median(averages.map(a=>a[1])),
    ratio:median(ratios),bootstrap95:[boot[250],boot[9750]],allocation}));
}
