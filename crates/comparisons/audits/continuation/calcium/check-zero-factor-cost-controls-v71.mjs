import {writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {validateRows,readRows,key} from './zero-factor-cost-protocol-v71.mjs';
import {json,sha} from './zero-factor-cost-sources-v71.mjs';
const plan=json('zero-factor-native-iterations-v71.json').find(g=>g.case===23&&g.policy===0&&g.lifecycle==='retained'),
 {baselinePilotNs,candidatePilotNs,...group}=plan,
 reference=readRows('results/zero-factor-native-cpu-0-baseline-v71.stdout').find(r=>key(r)===key(group));
assert(reference.expected.root!==null);assert.notEqual(reference.expected.root.polynomial[0],'0/1');
const changes=[['valid',()=>{}],['coefficient-sign',r=>{r.expected.root.polynomial[0]='-2/1';}],
 ['endpoint',r=>{r.expected.root.lower='0/1';}],['checksum',r=>{r.checksum++;}],
 ['iterations',r=>{r.iterations++;}],['elapsed-zero',r=>{r.elapsed_ns=0;}],
 ['instrumented-cpu',r=>{r.requests=1;}],['backward-clock',r=>{r.wall_after='1';}],
 ['final-mismatch',r=>{r.final_matches=false;}],['wrong-policy',r=>{r.policy=1;}]];
const results=[];
for(const [name,alter]of changes){
 const row=structuredClone(reference);alter(row);
 const path='zero-factor-cost-control-'+name+'-v71.jsonl';
 writeFileSync(path,JSON.stringify(row)+'\n'+JSON.stringify({terminal:true,groups:1,mode:'cpu'})+'\n',{flag:'wx'});
 let error=null;try{validateRows(path,[group],'baseline','cpu');}catch(e){assert.equal(e.name,'AssertionError');error=e.message;}
 assert.equal(error===null,name==='valid');results.push({name,path,sha256:sha(path),accepted:error===null,error});
}
writeFileSync('zero-factor-cost-controls-v71.json',JSON.stringify({checkpoint:71,status:'pass',valid:1,rejected:9,group,results},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:71,status:'pass',valid:1,rejected:9,scope:'Targeted corruptions of full polynomial/endpoint, metadata, completion, timing and allocator separation; not an exhaustive validator proof.'}));
