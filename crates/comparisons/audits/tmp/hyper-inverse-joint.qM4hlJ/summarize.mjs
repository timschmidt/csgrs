import {readFileSync} from 'node:fs';
const path=process.argv[2]&&process.argv[2]!=='partial'?process.argv[2]:'/tmp/hypersolve-joint-ab.jsonl';
const rows=readFileSync(path,'utf8').trim().split('\n').map(JSON.parse);
let state=314159265;
const random=()=>((state=(Math.imul(state,1664525)+1013904223)>>>0)/4294967296);
const median=a=>{a=[...a].sort((x,y)=>x-y);return a.length%2?a[a.length>>1]:(a[a.length/2-1]+a[a.length/2])/2;};
for(const key of [...new Set(rows.map(r=>`${r.suite}/${r.n}/${r.kind}`))]) {
  const [suite,nText,kind]=key.split('/'),n=Number(nText);
  const rs=rows.filter(r=>r.suite===suite&&r.n===n&&r.kind===kind&&!r.record);
  if(rs.length!==84){if(process.argv.includes('partial'))continue;throw Error(`incomplete ${key}: ${rs.length}`);}
  const cost=method=>Array.from({length:21},(_,block)=>{
    const xs=rs.filter(r=>r.method===method&&r.block===block);
    if(xs.length!==2||xs.some(r=>!Number.isFinite(r.ns)||r.ns<=0||r.calls%13))throw Error('bad pair');
    return median(xs.map(r=>r.ns));
  });
  const baseline=cost('baseline'),joint=cost('joint'),ratios=joint.map((v,i)=>v/baseline[i]);
  const boots=Array.from({length:5000},()=>median(Array.from({length:21},()=>ratios[Math.floor(random()*21)]))).sort((a,b)=>a-b);
  console.log(JSON.stringify({suite,n,kind,baseline_ns:median(baseline),joint_ns:median(joint),paired_ratio:median(ratios),ci95:[boots[125],boots[4874]],allocations:rows.filter(r=>r.suite===suite&&r.n===n&&r.kind===kind&&r.record==='alloc')}));
}
