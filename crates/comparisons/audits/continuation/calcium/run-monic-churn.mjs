import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const binaries=JSON.parse(readFileSync(resolve(here,'monic-cost-binaries.json')));
const output=resolve(here,'results/monic-cost-churn.jsonl'); writeFileSync(output,'',{flag:'wx'});
for(const b of Object.values(binaries)) assert.equal(createHash('sha256').update(readFileSync(b.path)).digest('hex'),b.sha256);
const started=new Date().toISOString(); let observations=0;
for(const kind of [0,1,2]) for(const code of [5,14,26]) for(const lifecycle of ['fresh','retained'])
for(const iterations of [1,100,1000]) for(const variant of ['baseline','trial']) {
    const value=await new Promise((ok,fail)=> {
        const child=spawn(binaries[`${variant}-allocation`].path,[String(kind),String(code),lifecycle,String(iterations)],{stdio:['ignore','pipe','pipe']});
        let out='',err=''; child.stdout.on('data',s=>out+=s); child.stderr.on('data',s=>err+=s);
        child.on('error',fail); child.on('close',c=>c===0?ok(JSON.parse(out)):fail(Error(`${c}: ${err}`)));
    });
    assert.equal(value.kind,kind); assert.equal(value.code,code); assert.equal(value.lifecycle,lifecycle);
    assert.equal(value.iterations,iterations); assert.equal(value.known,kind===2&&code===26?0:iterations);
    appendFileSync(output,JSON.stringify({variant,...value})+'\n'); observations++;
}
writeFileSync(resolve(here,'monic-cost-churn-summary.json'),JSON.stringify({started,finished:new Date().toISOString(),observations,
    limits:'Nine recipes at 1/100/1000 queries and both lifecycles. Requested live/peak deltas only, not RSS, no timing claim and no assumption of deterministic weak-cache occupancy.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({observations}));
