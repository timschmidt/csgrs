import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkMagTranscendental} from './check-mag-transcendental.mjs';
const tags=['compile','native','memcheck','linked-libraries','output-check','inventory'].map(t=>'mag-transcendental-'+t);
const files=['flint-mag-transcendental-controls.c','check-mag-transcendental.mjs','record-mag-transcendental-reads.mjs',
 'mag-transcendental-read-selection.json','bind-mag-transcendental.mjs','verify-mag-transcendental.mjs','magnitude-experiment.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);}
const previous=json('magnitude-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('mag-transcendental-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const linked=readFileSync('results/mag-transcendental-linked-libraries.stdout','utf8');
const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));
assert.deepEqual(libraries,previous.libraries);
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperreal/src/computable/approximation/exp_sqrt.rs':[[1,265]],
 'hyperreal/src/computable/approximation/logarithms.rs':[[1,143]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 const s=readFileSync(resolve('../../../..',p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 assert(live[p]);assert.equal(ranges[0][1],n,p);
}
const binary='/tmp/calcium-mag-transcendental.p8Wy83/mag-transcendental-controls';
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),liveSources:live,libraries,configurationFiles:previous.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},readRecords,newDonorLines:4660,newFullFileReads:44,
 completedTopLevelMagImplementations:52,coverageAtBinding:effectiveSummary(),hyperReadRanges,checks:checkMagTranscendental(),
 status:'Remaining top-level magnitude implementation reads complete; bounded transcendental/root/exp-tail corpus passes directed MPFR checks and focused Memcheck. Full ecosystem audit remains incomplete.',
 production:'No production or donor edit, fifth retained continuation transfer, matched Hyper benchmark, cleanup, deletion, commit, push or external report. All955 live hashes match checkpoint39. One33256-byte executable reuses existing libraries; all earlier snapshots, gates and crash evidence remain intact.',
 findings:'Thirty-bit bounds are deliberately coarse outside selected domains. Tiny/moderate/large exponential and logarithm paths carry different slack; d_log source claims universal rounding-mode padding by inspection, but only default FE_TONEAREST is newly checked. Large lower sinh combines padded lower exp with upper subtraction; sampled transitions pass, not a proof for all mantissas. Exp-tail donor tests compare only50 downward terms; the new independent oracle also encloses the entire omitted tail. Planner log2 is approximate and not a bound. All512 factorial/reciprocal table pairs and log/other constants were read, but combinatorial, conversion/IO and other tail functions remain source-only.',
 comparison:'Hyper already certifies local exponential/log-series domains before coarse shortcuts, uses demand-sized guards and truncation budgets, and retains exact-rational reduction/ln2 work. Square root and bounded-degree nth roots use integer enclosures; replacing them with fixed-mantissa exp/log bounds changes the exact-real contract without an established gain. Keep architecture and four existing transfers; no nonredundant implementation candidate or performance benefit selected.',
 limits:checkMagTranscendental().limits+' All52 top-level mag C files have complete read records, but the entire mag module, all tests/profiles and recursive dependencies are not complete. External Arb atan tables and remaining ARF get.c/MPFR support retain separate coverage gaps. Source-only I/O observations were not reproduced. Runtime and allocation totals include the MPFR oracle and are not performance comparisons.',
 followup:'Qualify source-read combinatorial/conversion/other-tail routines with bounded independent references as warranted; finish remaining mag tests/profiles, unread ARF conversion and scalar support including external MPFR high-helper source. Continue generic field/matrix/algebraic and all original references; complete inventory reconciliation and unresolved transfers remain open.'};
assert.equal(files.length,26);assert.equal(readRecords.length,44);assert.equal(m.binary.bytes,33256);
writeFileSync('mag-transcendental-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'Magnitude transcendental binding',files:files.length,gates:tags.length,
 liveFiles:Object.keys(live).length,readRecords:readRecords.length,readLines:m.newDonorLines,
 coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,checks:m.checks}));
