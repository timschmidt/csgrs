import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
for(const [p,h] of [
  ['exact-real-references/numbers-qualification/series_limit.rs','532e283afdbd070a6dd0d53b0def3c62293b97a4e2a32c94e0eb5e1cf5839069'],
  ['.audit-numbers.rjcbha/series-limit-before-debug','393332456d20a847d61b9216568dff99782bcb5084f90ddb21ac710f04b01546'],
  ['.audit-numbers.rjcbha/series-limit-before-release','3baa4a47b160aa9fa1bc412a452318890183e5052d590942c0c7296264d55215'],
])assert.equal(hash(p),h,p);
const rows=JSON.parse(readFileSync(resolve(dir,'series-limit-results.json'),'utf8'));
assert.equal(rows.length,8);
for(const r of rows){
  const control=r.bits===184000;assert(control||r.bits===192000);assert.equal(r.timeout_ms,60000);
  const s=readFileSync(resolve(dir,`series-limit-${control?'control':'before'}-${r.profile}-${r.op}.log`),'utf8');
  assert(s.includes(`START ${r.op}(1/16) bits=${r.bits} oracle_bits=${r.bits+256}`));
  if(control){assert.equal(r.status,0);assert(s.includes(`PASS ${r.op}(1/16) bits=${r.bits}`));}
  else if(r.profile==='debug'){assert.equal(r.status,101);assert(s.includes('attempt to multiply with overflow'));}
  else {assert.equal(r.status,2);assert(s.includes(`FAIL ${r.op}(1/16) bits=${r.bits}`));}
}
console.log(JSON.stringify({status:'CONFIRMED OPEN coefficient-width defect',requests:8,controlsPassed:4,debugOverflowExceptions:2,releaseNumericalFailures:2,caps:0,
  next:'Repair and qualify all four asin/asinh coefficient recurrences; no coefficient change is present at this checkpoint.'}));
