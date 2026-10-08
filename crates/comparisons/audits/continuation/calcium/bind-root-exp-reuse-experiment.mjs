import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here,p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const tags = {
  'randtest-canonical-compile': 1,
  'randtest-canonical-compile-current-api': 0,
  'randtest-canonical-native': 1,
  'randtest-canonical-memcheck': 1,
  'root-exp-reuse-debug': 0,
  'root-exp-reuse-release': 0,
  'root-exp-reuse-clippy': 0,
  'root-exp-reuse-fmt': 0,
  'root-exp-reuse-memcheck-deep-fresh': 0,
  'run-root-exp-reuse-cpu': 0,
  'run-root-exp-reuse-confirm-cpu': 0,
  'run-root-exp-reuse-confirm-alloc': 0,
};
for(const v of ['baseline','sign','reuse']) for(const mode of ['cpu','alloc'])
  tags[`root-exp-reuse-${v}-${mode}-build`] = 0;
for(const [tag,code] of Object.entries(tags)) {
  const result=JSON.parse(read(`results/${tag}.json`));
  assert.equal(result.code,code,tag); assert.equal(result.signal,null,tag);
}
const baseline=JSON.parse(read('baseline-hyperreal.json'));
const changed=['src/computable/node.rs','src/computable/node/structural_analysis.rs'];
const added=['exp_relation.rs','exp_relation_tests.rs','exp_relation_reuse.rs','exp_relation_reuse_tests.rs']
  .map(f=>`src/computable/node/${f}`);
for(const f of baseline.files) if(!changed.includes(f.path))
  assert.equal(hash(`root-exp-reuse-trial-hyperreal/${f.path}`),f.sha256,f.path);
const sources=['sign-work-cost-experiment.json','flint-randtest-canonical-probe.c',
  'root-exp-opaque-query-bench.rs','run-root-exp-reuse-bench.mjs','run-root-exp-reuse-confirm-bench.mjs',
  'run-root-exp-reuse-alloc-bench.mjs',
  'bind-root-exp-reuse-experiment.mjs','verify-root-exp-reuse-checkpoint.mjs',
  ...['baseline','sign','reuse'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>`root-exp-opaque-${v}-bench/${f}`)),
  ...[...changed,...added].map(f=>`root-exp-reuse-trial-hyperreal/${f}`)];
const evidence=Object.keys(tags).flatMap(t=>['json','stdout','stderr'].map(e=>`results/${t}.${e}`))
  .concat(['root-exp-reuse-cpu','root-exp-reuse-confirm-cpu','root-exp-reuse-confirm-alloc']
    .flatMap(stem=>[`results/${stem}.jsonl`,`${stem}-summary.json`]));
const manifest={schema:1,recorded:new Date().toISOString(),
  status:'Isolated weak-cache v3: core tests pass; production retention awaits wider lifecycle, oracle, consumer, and memory/size qualification.',
  trial:{directory:'root-exp-reuse-trial-hyperreal',changed,added,unchangedBaselineFiles:174},
  sourceHashes:Object.fromEntries(sources.map(p=>[p,hash(p)])),
  evidenceHashes:Object.fromEntries(evidence.map(p=>[p,hash(p)])),
  expectedCaptureCodes:tags,
  coverage:{calcium:{complete:146,partial:1,lines:16890},flint:{complete:150,partial:6,lines:18376},
    scalarTopLevelC:{calcium:114,flint:108}},
  randtest:{samples:10000,noncanonical:2221,wrongOne:107,canonicalControlsPassed:10000},
  testsPerProfile:780,
  excludedCpuCampaign:'root-exp-reuse-cpu: overlapped deep fresh Memcheck; preserved but not used for performance conclusions.',
  acceptedCampaigns:['root-exp-reuse-confirm-cpu','root-exp-reuse-confirm-alloc'],
  limits:'No production source change. Prior v2 consumer and MPFR results are not new v3 qualification. Cache retains at most32 weak Node allocations per touched thread, not strong child graphs; this is not peak-heap or whole-query work bound. Cold TLS first touch, high thread counts, other graph sharing/collision patterns, numeric kernel benchmarks and consumer costs remain open. Fresh/retained benchmark processes all have preconditioning. CPU file-size deltas are not application-wide binary sizes. Full donor and ecosystem source audits remain incomplete.'};
writeFileSync(resolve(here,'root-exp-reuse-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({sources:sources.length,evidenceFiles:evidence.length,status:manifest.status}));
