import fs from 'node:fs';
const rows=fs.readFileSync(process.argv[2],'utf8').trim().split('\n').map(JSON.parse);
const median=xs=>{xs=[...xs].sort((a,b)=>a-b);return xs.length%2?xs[(xs.length-1)/2]:(xs[xs.length/2-1]+xs[xs.length/2])/2;};
let state=0x193ad581;
const random=()=>{state^=state<<13;state^=state>>>17;state^=state<<5;return (state>>>0)/4294967296;};
for(const name of [...new Set(rows.map(r=>r.case))]) {
  const pairs=rows.filter(r=>r.kind==='pair'&&r.case===name);
  const mean=(r,side)=>{const a=r.samples.filter(s=>s.side===side);return a.reduce((n,s)=>n+s.ns/s.iterations,0)/a.length;};
  const ratios=pairs.map(r=>mean(r,'candidate')/mean(r,'baseline'));
  const bootstrap=Array.from({length:10000},()=>median(Array.from({length:ratios.length},()=>ratios[Math.floor(random()*ratios.length)]))).sort((a,b)=>a-b);
  const alloc=rows.filter(r=>r.kind==='allocation'&&r.case===name).map(r=>({side:r.side,calls:r.allocations/r.iterations,bytes:r.requested_bytes/r.iterations}));
  const result=rows.find(r=>r.kind==='results'&&r.case===name)?.results;
  console.log(JSON.stringify({case:name,baseline_ns:median(pairs.map(r=>mean(r,'baseline'))),candidate_ns:median(pairs.map(r=>mean(r,'candidate'))),ratio:median(ratios),ci95:[bootstrap[250],bootstrap[9750]],alloc,identical_result:result?.[0].result===result?.[1].result}));
}
