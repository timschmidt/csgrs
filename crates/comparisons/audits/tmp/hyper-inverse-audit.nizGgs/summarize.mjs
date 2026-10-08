import fs from 'node:fs';
let state=314159265;const rand=()=>{state=(Math.imul(state,1664525)+1013904223)>>>0;return state/4294967296;};
const median=a=>{a=[...a].sort((x,y)=>x-y);return a.length%2?a[a.length>>1]:(a[a.length/2-1]+a[a.length/2])/2;};
for(const [n,kind] of [[1,'dense'],[2,'dense'],[4,'dense'],[8,'dense'],[12,'dense'],[4,'cauchy'],[8,'cauchy'],[8,'diagonal'],[8,'permutation']]) {
    const rows=fs.readFileSync(`/tmp/hyper-inverse-bench-${n}-${kind}.csv`,'utf8').trim().split('\n').map(l=>l.split(','));
    const rounds=Array.from({length:21},()=>({old:[],joint:[],bareiss:[]}));
    for(const [round,method,,,calls,ns] of rows) rounds[+round][method].push(+ns);
    const cost=method=>rounds.map(r=>median(r[method]));
    const base=cost('old');const out={n,kind,old_ns:median(base)};
    for(const method of ['joint','bareiss']) {
        const values=cost(method),ratios=values.map((v,i)=>v/base[i]);
        const boots=Array.from({length:5000},()=>median(Array.from({length:21},()=>ratios[Math.floor(rand()*21)]))).sort((a,b)=>a-b);
        out[method]={ns:median(values),paired_ratio:median(ratios),ci95:[boots[125],boots[4874]]};
    }
    console.log(JSON.stringify(out));
}
