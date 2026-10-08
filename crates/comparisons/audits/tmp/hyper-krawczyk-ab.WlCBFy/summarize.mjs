import {readFileSync} from 'node:fs';
const rows=readFileSync('/tmp/hypersolve-krawczyk-ab.jsonl','utf8').trim().split('\n').map(JSON.parse);
let state=314159265;
const random=()=>((state=(Math.imul(state,1664525)+1013904223)>>>0)/4294967296);
const median=a=>{a=[...a].sort((x,y)=>x-y);return a.length%2?a[a.length>>1]:(a[a.length/2-1]+a[a.length/2])/2;};
for(const [n,kind] of [[1,'dense'],[2,'dense'],[4,'dense'],[8,'dense'],[12,'dense'],[8,'cauchy']]) {
  const rs=rows.filter(r=>r.n===n&&r.kind===kind&&!r.record);
  if(rs.length!==84)throw Error(`incomplete ${n} ${kind}: ${rs.length}`);
  const cost=method=>Array.from({length:21},(_,block)=>{
    const xs=rs.filter(r=>r.method===method&&r.block===block);
    if(xs.length!==2||xs.some(r=>!Number.isFinite(r.ns)||r.ns<=0||r.calls%13))throw Error('bad pair');
    return median(xs.map(r=>r.ns));
  });
  const baseline=cost('baseline'),fixed=cost('fixed'),ratios=fixed.map((v,i)=>v/baseline[i]);
  const boots=Array.from({length:5000},()=>median(Array.from({length:21},()=>ratios[Math.floor(random()*21)]))).sort((a,b)=>a-b);
  console.log(JSON.stringify({n,kind,baseline_ns:median(baseline),fixed_ns:median(fixed),paired_ratio:median(ratios),ci95:[boots[125],boots[4874]],allocations:rows.filter(r=>r.n===n&&r.kind===kind&&r.record==='alloc')}));
}
