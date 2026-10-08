import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkControls} from './check-nfloat-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const specs=[['nfloat-controls-compile',1],['nfloat-controls-compile-v2',0],['nfloat-controls-native',0],
 ['nfloat-controls-memcheck',0],['nfloat-support-inventory',1],['nfloat-support-inventory-v2',0],
 ['nfloat-hyper-dyadic-debug',0],['nfloat-hyper-dyadic-release',0],['nfloat-controls-linked-libraries',0]];
const files=['bind-nfloat-support.mjs','verify-nfloat-support.mjs','check-nfloat-controls.mjs',
 'record-nfloat-support-reads.mjs','nfloat-support-read-selection.json','flint-nfloat-controls.c',
 'flint-nfloat-controls-initial.c','derivative-qualification-experiment.json','capture.mjs','inventory.mjs',
 'inventory.json','effective-coverage.mjs'];
for(const[tag,code]of specs) {
 const g=json('results/'+tag+'.json');assert.equal(g.code,code,tag);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
}
const candidateSources=sources();
for(const[p,h]of Object.entries(candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
const selection=json('nfloat-support-read-selection.json'),coverage=json('coverage.json'),extensions=json('coverage-extensions.json');
const reads=selection.map(s=>{
 const e=(s.extension?extensions:coverage).find(e=>e.repo===s.repo&&e.path===s.path);
 assert.deepEqual(e.ranges,s.newRanges);return e;
});
const binary='/tmp/calcium-nfloat-controls.A4BEA9/nfloat-controls';
const libraryPaths=[...readFileSync(resolve(here,'results/nfloat-controls-linked-libraries.stdout'),'utf8')
 .matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
assert(libraryPaths.some(p=>p.endsWith('/libflint.so.25')));
const nativeLibrary=json('spectral-experiment.json').nativeLibrary;
assert.equal(sha(nativeLibrary.path),nativeLibrary.sha256);
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:specs.map(([tag,code])=>({tag,code,signal:null})),candidateSources,
 binary:{path:binary,sha256:sha(binary),bytes:statSync(binary).size},
 nativeLibrary,libraries:Object.fromEntries(libraryPaths.map(p=>[p,sha(p)])),reads,
 coverageAtBinding:effectiveSummary(),controls:checkControls(),
 hyperReadRanges:{
  'hyperreal/src/real/arithmetic/format_parse.rs':[[1,106]],
  'hyperreal/src/real/arithmetic/facts.rs':[[1360,1430]],
  'hyperreal/src/rational/arithmetic/aggregate_products.rs':[[1,240],[699,890],[3335,3665],[5580,5680]],
  'hyperreal/src/rational/arithmetic/tests.rs':[[1,70],[3930,4088]]
 },
 production:'No new production or donor change. The four earlier continuation transfers remain retained; all 955 live source/support hashes match checkpoint 27.',
 status:'Completed bounded nfloat arithmetic/conversion/dot/matrix support pass. Full Calcium/FLINT and original ecosystem scope remains open.',
 limits:'Deterministic native 64-bit finite moderate-exponent controls only. Exact GMP comparisons test enclosure direction, not correctly rounded equality; square/root comparisons use exact squared inequalities. Four caller FENV modes are distributed among patterns, not a full independent Cartesian campaign. No vector non-dot coverage, arbitrary random inputs, exponent-limit/underflow flushing, nonfinite values, 32-bit ABI or concurrent donor context qualification. Public directed matrix dispatcher chooses classical multiplication, so these controls do not qualify fixed/block/complex matrix kernels. Parsing/transcendentals and most mixed operations explicitly lack directed contracts. Earlier field-relation assertion remains unresolved; no assertion bypass or failed-output inspection. No new performance claim or candidate benchmark: fixed approximate arithmetic is not a contract-preserving replacement for Hyper exact accumulation. Initial driver include-order compile failure and nested-sandbox inventory failure are preserved.'};
writeFileSync(resolve(here,'nfloat-support-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'nfloat support binding',files:files.length,gates:specs.length,reads:reads.length,
 binaryBytes:manifest.binary.bytes,sourceFiles:955,coverageAtBinding:manifest.coverageAtBinding,controls:manifest.controls}));
