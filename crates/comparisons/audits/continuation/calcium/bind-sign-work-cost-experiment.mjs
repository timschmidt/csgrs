import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const hash = p => createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const tags = ['sign-consumer-hypercurve-baseline-debug',
  'sign-consumer-hypercurve-baseline-debug-storage-retry', 'sign-consumer-hypercurve-sign-debug',
  ...['cpu','alloc'].flatMap(mode => [`run-root-exp-opaque-${mode}`,
    ...['baseline','sign'].map(v=>`root-exp-opaque-${v}-build-${mode}`)])];
for(const tag of tags) {
  const r=JSON.parse(readFileSync(resolve(here,`results/${tag}.json`)));
  assert.equal(r.code,tag==='sign-consumer-hypercurve-baseline-debug'?101:0); assert.equal(r.signal,null);
}
const sources=['sign-consumer-experiment.json','root-exp-opaque-query-bench.rs','run-root-exp-opaque-bench.mjs',
  'bind-sign-work-cost-experiment.mjs','verify-sign-work-cost-checkpoint.mjs',
  ...['baseline','sign'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>`root-exp-opaque-${v}-bench/${f}`))];
const evidence=tags.flatMap(t=>['json','stdout','stderr'].map(e=>`results/${t}.${e}`))
  .concat(['cpu','alloc'].flatMap(mode=>[`results/root-exp-opaque-${mode}.jsonl`,`root-exp-opaque-${mode}-summary.json`]));
const manifest={schema:1,recorded:new Date().toISOString(),
  status:'Sign candidate remains isolated pending repeated opaque-comparison reuse; no live production transfer.',
  hypercurve:{suites:45,passedPerVariant:1761,ignoredPerVariant:9},
  totalCompletedConsumerTestsPerVariant:3309,
  opaque:{depths:[1,8,32,128],groups:16,cpuObservations:768,allocationObservations:192},
  sourceHashes:Object.fromEntries(sources.map(p=>[p,hash(p)])),
  evidenceHashes:Object.fromEntries(evidence.map(p=>[p,hash(p)])),
  limits:'Consumer tests were not synchronized CPU benchmarks; nine pre-existing Hypercurve ignored tests remain unrun (six benchmark drivers and three long stroke regressions). No full CI or both-profile consumer qualification. Opaque benchmark uses independently reconstructed nested-sine arguments, warm operands or retained differences, not every DAG sharing pattern. Requested allocation bytes are cumulative, not peak heap. Collector128 visits and16 terms do not bound every opaque node-pair comparison or arbitrary-size atom payload. Full donor and ecosystem audit remains incomplete.'};
writeFileSync(resolve(here,'sign-work-cost-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({sources:sources.length,evidenceFiles:evidence.length,status:manifest.status}));
