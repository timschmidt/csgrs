import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha} from './twelfth-cost-sources-v78.mjs';
import {groups,rows,validate} from './twelfth-cost-protocol-v78.mjs';
const group=groups()[0],row=rows('results/twelfth-cost-baseline-cpu-check-v78.stdout')[0];
const terminal={terminal:true,mode:'cpu',groups:1},records=[];
const tests=[['valid',a=>a],['false-equality',a=>{a[0].outcome=0;a[0].counts=[1,0,0];return a;}],
 ['wrong-case',a=>{a[0].case=1;return a;}],['wrong-precision',a=>{a[0].precision=-256;return a;}],
 ['wrong-lifecycle',a=>{a[0].lifecycle='retained';return a;}],['wrong-counts',a=>{a[0].counts=[0,0,2];return a;}],
 ['zero-time',a=>{a[0].elapsed_ns=0;return a;}],['reverse-clock',a=>{a[0].wall_after=String(BigInt(a[0].wall_before)-1n);return a;}],
 ['cpu-instrumented',a=>{a[0].requests=1;return a;}],['false-certificate',a=>{a[0].certificate='Equal';return a;}],
 ['truncated',a=>a.slice(0,1)],['duplicate',a=>[a[0],a[0],a[1]]],['terminal-count',a=>{a[1].groups=2;return a;}]];
for(const[name,mutate]of tests){
 const a=mutate(structuredClone([{...row,mode:'cpu',requests:0,requested_bytes:0,start_live:0,end_live:0,live_delta:0,peak_delta:0},terminal]));
 const path='results/twelfth-control-'+name+'-v78.jsonl';writeFileSync(path,a.map(r=>JSON.stringify(r)).join('\n')+'\n',{flag:'wx'});
 let error=null;try{validate(path,[group],'baseline','cpu');}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
 assert.equal(error===null,name==='valid');records.push({name,path,sha256:sha(path),accepted:error===null,error});
}
const result={checkpoint:78,valid:1,rejected:12,group,records};writeFileSync('twelfth-cost-controls-v78.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:78,status:'controls-pass',valid:1,rejected:12}));
