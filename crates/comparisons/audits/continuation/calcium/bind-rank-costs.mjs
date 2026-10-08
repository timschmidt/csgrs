import { readFileSync, writeFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)), workspace=resolve(here,'../../../..');
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft=process.argv.includes('--draft-rank-costs');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},gates:[],reads:[],binaries:{},
  status:'Rank v1 is not selected as implemented: same-outcome unresolved scans have severe CPU/allocation costs. The mathematical witness idea remains open for better scheduling/reuse. No production or donor change; entire ecosystem audit incomplete.',
  limits:'Costs cover seven patterns, four widths, two warm-input lifecycles on one host, not cold scalar construction. Different-outcome witness gains are not equal-work speedups. Allocation counts are Rust requested bytes/calls, not RSS/native overhead/stack or allocator retention. Per-group paired bootstrap intervals are not multiplicity-adjusted or universal bounds. Driver binary deltas include build-path/layout and do not measure application size. No broader state/downstream qualification for this unselected v1. Native matrix oracles avoid tested matrix kernels but share the scalar backend; sampled shapes, whole aliases and denominator sizes are not full branch coverage. Archived support was read, not executed.'};
const files=['bind-rank-costs.mjs','verify-rank-costs.mjs','capture.mjs','matrix-solve-experiment.json',
  'record-rank-cost-reads.mjs','rank-cost-read-selection.json','run-rank-costs.mjs',
  'rank-cost-corpus.rs','rank-cost-cpu.rs','rank-cost-allocation.rs','rank-cost-binaries.json',
  'rank-cost-cpu-summary.json','rank-cost-allocation-summary.json',
  'results/rank-cost-cpu.jsonl','results/rank-cost-allocation.jsonl','flint-matrix-support-probe.c'];
for(const v of ['baseline','candidate'])for(const f of ['Cargo.toml','Cargo.lock'])files.push(`rank-cost-${v}/${f}`);
const tags=['rank-cost-baseline-build','rank-cost-candidate-build','rank-cost-cpu-run','rank-cost-allocation-run',
  'matrix-support-native-compile','matrix-support-native','matrix-support-memcheck'];
for(const tag of tags) {
  const g=json(`results/${tag}.json`);assert.equal(g.code,0,tag);assert.equal(g.signal,null);
  manifest.gates.push(tag);for(const ext of ['json','stdout','stderr'])files.push(`results/${tag}.${ext}`);
}
for(const p of files)manifest.files[p]=sha(p);
const coverage=json('coverage.json');
manifest.reads=json('rank-cost-read-selection.json').map(e=>{const v=coverage.find(v=>v.repo===e.repo&&v.path===e.path);assert(v);return v;});
assert.equal(manifest.reads.length,40);
const retained=json('retained-monic.json'),prior=json('matrix-solve-experiment.json');
for(const[p,h]of Object.entries(retained.liveSources)) {
  assert.equal(sha(resolve(workspace,p)),h,`live ${p}`);
  assert.equal(sha(`${retained.frozenSnapshot}/${p}`),h,`baseline ${p}`);
}
for(const[p,h]of Object.entries(prior.candidateSources))assert.equal(sha(`rank-dominance-trial/${p}`),h,`v1 ${p}`);
manifest.hyperReadRanges={'hyperlattice/src/matrix/core.rs':[[1338,1448],[1740,1820],[5300,5360],[5905,5985]]};
for(const b of Object.values(json('rank-cost-binaries.json'))) {
  assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);
  manifest.binaries[b.path]=b.sha256;
}
const native='/tmp/calcium-rank-costs.lbsszv/flint-matrix-support';manifest.binaries[native]=sha(native);
manifest.nativeLibrary={path:prior.nativeLibrary.path,sha256:sha(prior.nativeLibrary.path)};
assert.deepEqual(manifest.nativeLibrary,prior.nativeLibrary);
if(!draft)writeFileSync(resolve(here,'rank-cost-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'rank cost binding',draft,files:files.length,gates:tags.length,reads:manifest.reads.length,
  binaries:Object.keys(manifest.binaries).length,binaryBytes:Object.keys(manifest.binaries).reduce((n,p)=>n+statSync(p).size,0)}));
export { manifest };
