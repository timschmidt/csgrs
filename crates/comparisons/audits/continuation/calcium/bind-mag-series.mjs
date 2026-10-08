import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkMagSeries} from './check-mag-series.mjs';
const tags=['compile','native','memcheck','linked-libraries','output-check','inventory'].map(t=>'mag-series-'+t);
const files=['flint-mag-series-controls.c','check-mag-series.mjs','record-mag-series-reads.mjs','mag-series-read-selection.json',
 'bind-mag-series.mjs','verify-mag-series.mjs','mag-transcendental-experiment.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);}
const previous=json('mag-transcendental-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('mag-series-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const linked=readFileSync('results/mag-series-linked-libraries.stdout','utf8');
const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));assert.deepEqual(libraries,previous.libraries);
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperreal/src/real/arithmetic/elementary_functions.rs':[[1810,2050]],
 'hyperreal/src/computable/approximation/constants.rs':[[140,225]],'hyperreal/src/real/convert.rs':[[1,170]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)){
 const s=readFileSync(resolve('../../../..',p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 assert(live[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const binary='/tmp/calcium-mag-series.o08QL0/mag-series-controls';
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),liveSources:live,libraries,configurationFiles:previous.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},readRecords,newDonorLines:3478,newFullFileReads:29,
 completedExistingPaths:['src/arf/get.c'],completedMagDirectoryFiles:104,coverageAtBinding:effectiveSummary(),hyperReadRanges,checks:checkMagSeries(),
 reusedDonorFiles:['src/mag/fac_ui.c','src/mag/bin_uiui.c','src/mag/binpow_uiui.c','src/mag/bernoulli_div_fac_ui.c',
  'src/mag/geom_series.c','src/mag/polylog_tail.c','src/mag/hurwitz_zeta_uiui.c','src/mag/set_d.c','src/mag/set_d_2exp_fmpz.c','src/mag/io.c'],
 status:'All104 inventoried files under src/mag/ are completely read, together with the separately credited public header/manual. Finite combinatorial, full-tail and conversion controls pass native, independent BigInt checks and focused Memcheck. Full ecosystem and recursive support audit remains incomplete.',
 production:'No production/donor edit, fifth retained transfer, cleanup, deletion, commit, push or external report. All955 live hashes match checkpoint40. One38448-byte executable reuses existing libraries; previous artifacts and failures remain preserved.',
 findings:'All256 factorial and256 reciprocal table entries pass exact checks, with fallback sweep through4096. Exact binomial recurrences cover36237 inputs; Bernoulli absolute coefficients through128 use two different rational recurrences. Geometric tails use full rational closed forms; polylog adds a complete eventual-geometric remainder and Hurwitz uses zeta-minus-prefix.199 of576 known-convergent polylog cases per precision yield valid but uninformative infinity;377 yield finite bounds. Exact finite binary64 import and bounded magnitude exports preserve their specified bound/rounding contracts; only valid self-generated string roundtrips run. Earlier source-only I/O failure observations are not reproduced.',
 comparison:'Hyper already uses word-batched product trees and structural factorial cancellation for exact gamma/beta coefficients; those exact values cannot be replaced with magnitude estimates. Float imports retain exact dyadic rational classes. A narrow unimplemented planning candidate remains: e_terms_for_precision grows an exact factorial solely to certify a bit threshold before binary splitting builds the denominator again. A normalized fixed-word LOWER factorial enclosure may certify the same or conservatively larger term count without that first growing BigInt. An upper factorial bound is the wrong polarity. Requires a proof, isolated exact/MPFR/state controls and matched low/high-precision CPU/allocation/size tests before selection; no performance benefit or transfer yet claimed.',
 limits:checkMagSeries().limits+' Full directory reads are not full numerical coverage. Direct arbitrary-width ARF rounding, file IO, unsupported special domains and recursive formatter/integer/MPFR internals remain separate gaps. Valid infinity counts do not establish finite-tail completeness. Timings and cumulative allocations include oracle work, not a donor/Hyper benchmark.',
 followup:'Evaluate the isolated planning-only e term-count lower-bound candidate with exact threshold proofs, independent numeric controls, matched lifecycle benchmarks and size/allocation gates; retain only if worthwhile. Continue remaining ARF/scalar/MPFR and generic field/matrix/algebraic support and every original reference. Full inventory reconciliation and older unresolved transfers remain open.'};
assert.equal(files.length,26);assert.equal(readRecords.length,30);assert.equal(m.binary.bytes,38448);
writeFileSync('mag-series-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'Magnitude series/conversion binding',files:files.length,gates:tags.length,liveFiles:Object.keys(live).length,
 readRecords:readRecords.length,readLines:m.newDonorLines,coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,checks:m.checks}));
