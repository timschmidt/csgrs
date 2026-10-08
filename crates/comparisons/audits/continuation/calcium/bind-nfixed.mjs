import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkFixedControls} from './check-nfixed-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const specs=[['nfixed-controls-compile',1],['nfixed-controls-compile-v2',0],['nfixed-controls-native',1],
 ['nfixed-controls-memcheck',1],['nfixed-hyper-filter-debug',0],['nfixed-hyper-filter-release',0],
 ['nfixed-inventory',0],['nfixed-controls-linked-libraries',0],['nfixed-output-check-initial',1],
 ['nfixed-fenv-compile',0],['nfixed-fenv-native',0],['nfixed-fenv-memcheck',1]];
const files=['bind-nfixed.mjs','verify-nfixed.mjs','check-nfixed-controls.mjs','record-nfixed-reads.mjs',
 'nfixed-read-selection.json','flint-nfixed-controls.c','flint-nfixed-controls-initial.c','nfloat-support-experiment.json','capture.mjs',
 'check-nfixed-controls-initial.mjs','fenv-product-control.c'];
for(const[tag,code]of specs) {
 const g=json('results/'+tag+'.json');assert.equal(g.code,code,tag);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
}
const candidateSources=sources(),coverage=json('coverage.json');
for(const[p,h]of Object.entries(candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
const binary='/tmp/calcium-nfixed-controls.qZn5o6/nfixed-controls',fenvBinary='/tmp/calcium-nfixed-controls.qZn5o6/fenv-product-control';
const paths=[...readFileSync(resolve(here,'results/nfixed-controls-linked-libraries.stdout'),'utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(paths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,json('nfloat-support-experiment.json').libraries);
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:specs.map(([tag,code])=>({tag,code,signal:null})),candidateSources,libraries,
 binary:{path:binary,sha256:sha(binary),bytes:statSync(binary).size},
 fenvBinary:{path:fenvBinary,sha256:sha(fenvBinary),bytes:statSync(fenvBinary).size},
 reads:json('nfixed-read-selection.json').map(s=>coverage.find(e=>e.repo===s.repo&&e.path===s.path)),
 coverageAtBinding:effectiveSummary(),controls:checkFixedControls(),hyperReadRanges:{
  'hyperlattice/src/matrix/ops.rs':[[1,4]],'hyperlattice/src/matrix/core.rs':[[6460,6590]],
  'hyperlimit/src/predicates/filters.rs':[[1,225]],'hyperlimit/src/resolve.rs':[[1,55],[240,430]],
  'hyperlimit/src/predicates/ring.rs':[[275,367]],'hyperlimit/benches/predicates.rs':[[1830,1885]],
  'hyperlimit/Cargo.toml':[[1,43]]
 },
 instrumentation:'Native/Memcheck outcome counts and reported maximum errors agree, but 908 rows/1225 binary64 bound fields differ. Each run is independently checked, not required identical. A separate no-FLINT volatile binary64 multiplication control matches MPFR in all 12 native cases and differs in four under Memcheck; instrumented values match the nearest-mode native controls even while FENV reports each requested mode. This demonstrates a local instrumentation discrepancy, not a general explanation of every donor numerical result. Initial byte-identical-output checker failure is preserved.',
 production:'No new production or donor change. All 955 live source/support files match checkpoint 28; previous four continuation transfers remain retained.',
 status:'Fixed-point kernel/test/profiler source pass completed; independent final-output controls pass, intermediate-bound witness fails as preserved. Full ecosystem audit remains open.',
 limits:'One native-vector witness for a known Strassen intermediate repeated across six precisions/four FENV modes is not 24 independent bugs or a demonstrated wrong final matrix output. The 52 automatic/explicit-cutoff bound differences are source-supported dispatch inconsistencies, not 52 numerical product failures. Exact GMP checks cover deterministic finite 64-bit positive-dimension matrices and nonempty raw dots, no output aliases, overflow, zero-length raw dots, 32-bit, arbitrary concurrency or complex arithmetic. Matrix entries are scaled below 2^-20; raw dots below 2^-12. High matrix precisions 22/47/66 limbs use only four small shapes. No giant input, invalid access or previous assertion reproduction. Memcheck has zero errors/all allocations freed but exit 1 preserves the numerical bound failures; cumulative allocations include the oracle and are not donor-only performance, peak RSS or disk usage. No candidate performance/size claim; donor profiling thresholds are not transferable benchmarks.',
 followup:'Potential Hyperlimit transfer: the >4-term structural same-sign filter collects a Vec although a constant-size sign summary may suffice. Preserve Unknown evaluation/trace behavior and validate public nonrational consumers before considering a change. Existing rational ring benchmark bypasses that filter and cannot qualify the candidate. No prototype or retention claimed.'};
writeFileSync(resolve(here,'nfixed-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'nfixed binding',files:files.length,gates:specs.length,reads:manifest.reads.length,
 binaryBytes:manifest.binary.bytes+manifest.fenvBinary.bytes,binaries:2,sourceFiles:955,
 coverageAtBinding:manifest.coverageAtBinding,controls:manifest.controls.summary}));
