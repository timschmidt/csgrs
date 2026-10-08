import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources, sha, json } from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url));
sources();const binaries=json('derivative-cost-binaries.json');
for(const v of ['baseline','candidate'])assert.equal(sha(binaries[v+'-allocation'].path),binaries[v+'-allocation'].sha256);
const path=resolve(here,'results/derivative-retention-staircase.jsonl');writeFileSync(path,'',{flag:'wx'});
const started=new Date().toISOString();let rows=0;
function run(file,args) {
 return new Promise((ok,fail)=>{
  const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
  c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
  c.on('close',code=>code===0?ok(JSON.parse(out)):fail(Error(code+': '+err)));
 });
}
for(const degree of [8,24])for(const kind of [0,1])for(const order of degree===8?[128]:[0,24,128])
for(const lifecycle of ['retained_curve','fresh_curve'])for(const iterations of [1,8,32,128,512])
for(let repeat=0;repeat<2;repeat++)for(const variant of ['baseline','candidate']) {
 const begin=new Date().toISOString(),r=await run(binaries[variant+'-allocation'].path,
  [variant,String(kind),String(degree),String(order),lifecycle,String(iterations)]);
 const end=new Date().toISOString();
 for(const[k,v]of Object.entries({variant,mode:'allocation',degree,kind,order,lifecycle,iterations}))assert.equal(r[k],v);
 const row={repeat,started:begin,finished:end,...r};appendFileSync(path,JSON.stringify(row)+'\n');rows++;
}
sources();
const summary={started,finished:new Date().toISOString(),rows,binaries,
 limits:'Separate processes start with the original exact precheck and eight warm queries, then measure 1/8/32/128/512 additional queries. Two repeats per variant; rational/1/3 parameter, degree-eight high-order control plus degree-24 orders 0/24/128, two scalar families and two lifecycles. Allocation/retention diagnostics only; concurrent consumer qualification means elapsed times are not comparative performance evidence. Counts measure requested Rust bytes, not RSS or proof of arbitrary long-run retention bounds.'};
assert.equal(rows,320);writeFileSync(resolve(here,'derivative-retention-staircase.json'),JSON.stringify(summary,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify(summary));
