import {createReadStream,writeFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import assert from 'node:assert/strict';
import {recordValidator,observationPlan} from './check-zero-factor-wasm-records-v72.mjs';
import {sha} from './zero-factor-wasm-sources-v72.mjs';
const fixtures={},plan=observationPlan();
for await(const line of createInterface({input:createReadStream('results/zero-factor-wasm-qualification-v72.stdout'),crlfDelay:Infinity})){
 const row=JSON.parse(line),spec=plan.next().value;
 if(!fixtures[row.kind]&&((row.expected??row.actual)?.status==='Transformed'))fixtures[row.kind]={row,spec};
 if(fixtures.rational&&fixtures.history)break;
}
assert(fixtures.rational&&fixtures.history);const validate=recordValidator(),results=[];
const mutations=[
 ['rational','signed-polynomial',r=>{const p=r.expected.root.polynomial,i=p.findIndex(x=>x!=='0');assert(i>=0);p[i]=p[i].startsWith('-')?p[i].slice(1):'-'+p[i];}],
 ['rational','endpoint',r=>r.expected.root.lower='987654321'],
 ['rational','status',r=>r.expected.status='Undecided'],
 ['rational','source-completion',r=>r.sources_unchanged=false],
 ['rational','result-completion',r=>r.final_matches=false],
 ['rational','checksum',r=>r.checksum++],
 ['rational','host-checksum',r=>r.host_checksum++],
 ['rational','iterations',r=>r.iterations++],
 ['rational','wrong-policy',r=>r.policy=1-r.policy],
 ['rational','wrong-variant',r=>r.variant='candidate'],
 ['rational','wrong-instance-mode',r=>r.instanceMode='fresh'],
 ['rational','wrong-sequence',r=>r.sequence++],
 ['rational','zero-duration',r=>r.qualification_batch_ns=0],
 ['rational','backward-clock',r=>r.finished='1970-01-01T00:00:00.000Z'],
 ['rational','memory-decrease',r=>r.memoryAfterBatch=0],
 ['rational','fractional-memory',r=>r.memoryAfterFinish+=0.5],
 ['rational','allocation-field',r=>r.requests=0],
 ['history','source-metadata',r=>r.left.constraint=987654321],
 ['history','source-completion',r=>r.input_records_unchanged=false],
 ['history','result-completion',r=>r.final_matches_preconditioned=false],
 ['history','wrong-history',r=>r.history=1-r.history],
];
for(const kind of ['rational','history']){
 const {row,spec}=fixtures[kind];validate(row,spec);results.push({kind,name:'valid',accepted:true,row,spec});
}
for(const [kind,name,change]of mutations){
 const {row:original,spec}=fixtures[kind],row=structuredClone(original);change(row);let error;
 assert.throws(()=>validate(row,spec),e=>{assert.equal(e.name,'AssertionError');error=e.message;return true;});
 results.push({kind,name,accepted:false,error,row,spec});
}
const result={checkpoint:72,status:'record-corruptions-pass',accepted:2,rejected:21,
 scripts:Object.fromEntries(['check-zero-factor-wasm-corruptions-v72.mjs','check-zero-factor-wasm-records-v72.mjs'].map(p=>[p,sha(p)])),results};
writeFileSync('zero-factor-wasm-corruptions-v72.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:72,status:result.status,accepted:2,rejected:21,sha256:sha('zero-factor-wasm-corruptions-v72.json')}));
