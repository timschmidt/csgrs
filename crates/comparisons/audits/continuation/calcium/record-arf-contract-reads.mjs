import {writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const tests=['main','t-abs_bound_le_2exp_fmpz','t-abs_bound_lt_2exp_fmpz','t-abs_bound_lt_2exp_si',
 't-add','t-add_fmpz','t-add_fmpz_2exp','t-add_si','t-add_ui','t-addmul_fmpz','t-addmul_si','t-addmul_ui',
 't-approx_dot','t-ceil','t-cmp','t-cmp_2exp_si','t-cmpabs','t-cmpabs_2exp_si','t-div','t-dump_file',
 't-dump_str','t-floor','t-frexp','t-get_d','t-get_fmpz','t-get_mpfr','t-get_str','t-is_int_2exp_si',
 't-mul_fmpz','t-mul_si','t-mul_ui','t-nint','t-root','t-rsqrt','t-set_d','t-set_fmpq','t-set_fmpz_2exp',
 't-set_round_fmpz','t-sgn','t-sqrt','t-sub','t-sub_fmpz','t-sub_si','t-sub_ui','t-submul_fmpz',
 't-submul_si','t-submul_ui'];
assert.equal(tests.length,47);
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),old=effectiveCoverage(),records=[];
const selected=tests.map(p=>({path:'src/arf/test/'+p+'.c',ranges:null}));
selected.push({path:'src/arf.h',ranges:[[229,376],[531,564],[607,609],[651,699],[786,820],[863,999],[1136,1171]]},
 {path:'doc/source/arf.rst',ranges:[[116,180],[251,549],[671,709]]});
let newLines=0;
for(const {path,ranges}of selected) {
 const f=inv.files.find(f=>f.path===path);assert(f?.text,path);
 assert.equal(sha(resolve('../../../../exact-real-references/flint',path)),f.sha256,path);
 const prior=old.find(r=>r.repo==='flint'&&r.path===path),actual=ranges??[[1,f.lines]];
 if(!ranges)assert(!prior,path);
 for(const[a,b]of actual) {
  assert(a>=1&&b>=a&&b<=f.lines,path);newLines+=b-a+1;
  assert(!prior?.ranges.some(([c,d])=>a<=d&&b>=c),path);
 }
 const note=path==='src/arf/test/t-add.c'||path==='src/arf/test/t-sub.c'
  ? 'Source-read: randomized mode is overwritten by ARF_RND_DOWN; naive reference uses the same arithmetic family. Not an all-mode independent oracle.'
  :path==='src/arf/test/t-add_si.c'
  ? 'Source-read: n_randint(state,1) makes the in-place branch unreachable. Other wrappers have two branches; this is a test-coverage gap, not a proved library failure.'
  :path==='src/arf/test/t-get_d.c'
  ? 'Source-read: first/third n_randint(state,4) switches have unreachable nearest-even defaults; roundtrip/backend agreement is narrower than all-mode correct rounding.'
  :/t-(root|sqrt|rsqrt)\.c$/.test(path)
  ? 'Source-read: four directed modes omit nearest-even; reference shares MPFR root backend. Independent exact-power midpoint/neighbor checks are needed for stronger numerical evidence.'
  :path==='src/arf/test/t-approx_dot.c'
  ? 'Source-read: reference error sum indexes both vectors with revx even when revy differs. Reversal and inflated Arb containment share backend machinery and are not an independent exact oracle. No crash/invalid-memory reproduction.'
  :'Source-read: public finite-dyadic/rounding/storage contract, wrapper consistency, destination reuse, comparison or valid conversion/I-O tests. Special/unbounded/exceptional paths are read-only here, not newly executed.';
 records.push({repo:'flint',path,ranges:actual,note:'Checkpoint44. '+note});
}
assert.equal(newLines,5783);assert.equal(records.length,49);
writeFileSync('arf-contract-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
const coverage=effectiveCoverage();const scope=inv.files.filter(f=>f.path.startsWith('src/arf/')||['src/arf.h','doc/source/arf.rst'].includes(f.path));
for(const f of scope)assert.deepEqual(coverage.find(r=>r.repo==='flint'&&r.path===f.path)?.ranges,[[1,f.lines]],f.path);
console.log(JSON.stringify({newRecords:records.length,newLines,completeScopeFiles:scope.length,coverage:effectiveSummary()}));
