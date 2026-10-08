import { readFileSync, writeFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft=process.argv.includes('--draft-spectral');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},gates:[],reads:[],binaries:{},
  status:'Both ca_mat source directories are complete at the pinned versions, not all called support or the ecosystem. No new production/donor change. Jordan certificates pass. Native matrix exp/log has 14 unresolved exact comparisons, no observed incorrect result; its instrumented run aborts in an internal LLL assertion and is not a clean memory gate. Rank v1 remains unselected; demand-bounded Hyper polynomial derivative work is an untested follow-up idea.',
  limits:'Known finite Jordan recipes, shifts and dense similarities are not arbitrary spectra or full branch coverage. Explicit scalar chain sums avoid CA matrix multiplication and an exact rational determinant checks P, but scalar arithmetic and the FLINT rational subsystem remain shared. Function coefficient formulas avoid matrix exp/log/Jordan calls but share scalar elementary functions and the standard exponential factorial recurrence. Unknown equality is not an incorrect value or a proved identity. One Valgrind abort does not establish native reproduction or its floating-point/root cause; aborted-process possible losses do not establish normal-exit leaks. Buffered output is only a prefix and does not locate the precise failing case. No new Hyper candidate, CPU/allocation campaign or size-improvement claim. Preserve all prior failures and user edits.'};
const files=['bind-spectral.mjs','verify-spectral.mjs','capture.mjs','charpoly-domain-experiment.json',
  'record-spectral-reads.mjs','spectral-read-selection.json','effective-coverage.mjs',
  'spectral-corpus.h','flint-spectral-probe.c','flint-matrix-function-probe.c'];
const gates=[['spectral-native-compile',0,null],['spectral-native',0,null],['spectral-memcheck',0,null],
  ['matrix-function-native-compile',0,null],['matrix-function-native',1,null],['matrix-function-memcheck',null,'SIGABRT']];
for(const [tag,code,signal]of gates) {
  const g=json(`results/${tag}.json`);assert.equal(g.code,code,tag);assert.equal(g.signal,signal,tag);
  manifest.gates.push({tag,code,signal});for(const ext of ['json','stdout','stderr'])files.push(`results/${tag}.${ext}`);
}
for(const p of files)manifest.files[p]=sha(p);
const coverage=json('coverage.json');
manifest.reads=json('spectral-read-selection.json').map(s=>{const e=coverage.find(e=>e.repo===s.repo&&e.path===s.path);assert(e);return e;});
assert.equal(manifest.reads.length,66);
manifest.extensions=json('coverage-extensions.json').filter(e=>e.repo==='flint'&&e.path==='doc/source/ca_mat.rst');
assert.equal(manifest.extensions.length,1);assert.deepEqual(manifest.extensions[0].ranges,[[515,624]]);
manifest.coverageAtBinding=effectiveSummary();
const retained=json('retained-monic.json');
for(const [p,h]of Object.entries(retained.liveSources)) {
  assert.equal(sha(resolve(workspace,p)),h,`live ${p}`);assert.equal(sha(`${retained.frozenSnapshot}/${p}`),h,`qualified ${p}`);
}
manifest.hyperReadRanges={
  'hypersolve/src/integer_interpolation.rs':[[500,605]],
  'hypercurve/src/rational_bezier_general.rs':[[9450,9555]],
  'hyperlattice/src/matrix/core.rs':[[315,435]]};
for(const name of ['flint-spectral','flint-matrix-function']) {
  const p=`/tmp/calcium-spectral.nqRkRT/${name}`;manifest.binaries[p]={sha256:sha(p),bytes:statSync(p).size};
}
const core='results/matrix-function-memcheck.vgcore';manifest.failureCore={path:core,sha256:sha(core),bytes:statSync(resolve(here,core)).size};
const prior=json('charpoly-domain-experiment.json');manifest.nativeLibrary=prior.nativeLibrary;
assert.equal(sha(manifest.nativeLibrary.path),manifest.nativeLibrary.sha256);
if(!draft)writeFileSync(resolve(here,'spectral-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'spectral binding',draft,files:files.length,gates:gates.length,reads:manifest.reads.length,
  extensions:manifest.extensions.length,binaries:Object.keys(manifest.binaries).length,
  binaryBytes:Object.values(manifest.binaries).reduce((n,b)=>n+b.bytes,0),failureCoreBytes:manifest.failureCore.bytes}));
export {manifest};
