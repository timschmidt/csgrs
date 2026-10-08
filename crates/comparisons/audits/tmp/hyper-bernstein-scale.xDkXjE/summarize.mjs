import {readFileSync} from 'node:fs';
const path=process.argv[2]&&process.argv[2]!=='partial'?process.argv[2]:'/tmp/bernstein-public-ab.jsonl';
const rows=readFileSync(path,'utf8').trim().split('\n').map(JSON.parse);
let state=314159265;const rand=()=>((state=(Math.imul(state,1664525)+1013904223)>>>0)/4294967296);
const median=a=>{a=[...a].sort((x,y)=>x-y);return a.length%2?a[a.length>>1]:(a[a.length/2-1]+a[a.length/2])/2;};
for(const key of [...new Set(rows.map(r=>`${r.degree}/${r.kind}/${r.depth}`))]) {
  const [degreeText,kind,depthText]=key.split('/'),degree=+degreeText,depth=+depthText;
  const rs=rows.filter(r=>r.degree===degree&&r.kind===kind&&r.depth===depth&&!r.record);
  if(rs.length!==84){if(process.argv.includes('partial'))continue;throw Error(`incomplete ${key} ${rs.length}`);}
  const costs=method=>Array.from({length:21},(_,block)=>{
    const xs=rs.filter(r=>r.method===method&&r.block===block);
    if(xs.length!==2||xs.some(r=>r.calls%13||!Number.isFinite(r.ns)||r.ns<=0))throw Error('bad block');
    return median(xs.map(r=>r.ns));
  });
  const base=costs('baseline'),candidate=costs('candidate'),ratios=candidate.map((v,i)=>v/base[i]);
  const bs=Array.from({length:5000},()=>median(Array.from({length:21},()=>ratios[Math.floor(rand()*21)]))).sort((a,b)=>a-b);
  console.log(JSON.stringify({degree,kind,depth,baseline_ns:median(base),candidate_ns:median(candidate),paired_ratio:median(ratios),ci95:[bs[125],bs[4874]],allocations:rows.filter(r=>r.degree===degree&&r.kind===kind&&r.depth===depth&&r.record==='alloc')}));
}
