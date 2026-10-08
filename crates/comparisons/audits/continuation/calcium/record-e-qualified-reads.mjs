import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
const paths=['abs_bound_lt_2exp_si.c','call_mpfr_func.c','ceil.c','cmp.c','debug.c','div.c','equal.c','floor.c',
 'frexp.c','inlines.c','is_int.c','is_int_2exp_si.c','nint.c','randtest.c','root.c','rsqrt.c','sqrt.c','urandom.c'].map(p=>'src/arf/'+p);
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),prior=[...json('coverage.json'),...json('coverage-extensions.json')],records=[];
let lines=0;
for(const path of paths) {
 assert(!prior.some(r=>r.repo==='flint'&&r.path===path),path);const f=inv.files.find(f=>f.path===path);assert(f?.text);
 assert.equal(sha(resolve('../../../../exact-real-references/flint',path)),f.sha256,path);lines+=f.lines;
 records.push({repo:'flint',path,ranges:[[1,f.lines]],note:'Checkpoint43 complete source read during e-planner qualification. Remaining ARF top-level implementation: exact finite comparisons/rounding, exponent decomposition, sticky quotient correction, MPFR-backed root rescaling, adapter state or test support. Source credit only; no new independent numerical campaign for these routines, no wrapper state/failure or invalid-input reproduction, no additional transfer selected.'});
}
assert.equal(records.length,18);assert.equal(lines,1176);
const old=json('coverage-extensions.json');writeFileSync('coverage-extensions.json',JSON.stringify([...old,...records],null,2)+'\n');
writeFileSync('e-qualified-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
const top=inv.files.filter(f=>/^src\/arf\/[^/]+\.c$/.test(f.path));
console.log(JSON.stringify({newFullFiles:records.length,newLines:lines,topLevelArfFiles:top.length,coverage:effectiveSummary()}));
