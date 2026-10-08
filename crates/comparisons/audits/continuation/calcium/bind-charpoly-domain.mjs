import { readFileSync, writeFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft=process.argv.includes('--draft-charpoly-domain');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},gates:[],reads:[],binaries:{},
  status:'No production or donor transfer. All structured solve/characteristic controls pass. Generic CA typed-domain/property failures are reproduced; the negative-rational principal-argument defect is a reproduction of checkpoint six, not a newly discovered independent defect. Hyper public inverse-trig domain controls pass. Entire ecosystem audit incomplete; isolated rank v1 remains unselected.',
  limits:'Current FLINT executed; archived corresponding files independently read, not executed. Integer/scalar RHS and known-spectrum coefficient oracles avoid tested matrix kernels but share the scalar backend. Sampled dimensions, seeded outputs and valid whole aliases are not full branch/arbitrary-state coverage. Failed operation outputs are not inspected. Domain-row counts repeat values and lifecycles, not distinct bugs; a separate exact negative-pi value witness resolves algebraicity-predicate Unknown without treating Unknown as False or changing the correct positive-pi principal oracle. Hyper checks domain acceptance/rejection and refinability, not independent inverse-trig numerical accuracy; original/shared scalar inputs are not guaranteed cold. No new matched performance, allocation or binary-size comparison and no new production candidate. Focused clean memory controls do not supersede earlier mathematical failures or Rust-thread possible-loss reports.'};
const files=['bind-charpoly-domain.mjs','verify-charpoly-domain.mjs','capture.mjs','rank-cost-experiment.json',
  'record-charpoly-domain-reads.mjs','charpoly-domain-read-selection.json',
  'flint-matrix-solve-certificate-probe.c','flint-charpoly-certificate-probe.c',
  'flint-ca-domain-probe-initial.c','flint-ca-domain-probe.c','flint-ca-arg-witness-probe.c',
  'flint-ca-negative-arg-witness-probe.c','hyper-domain-probe.rs','hyper-domain-probe/Cargo.toml','hyper-domain-probe/Cargo.lock'];
const codes={
  'solve-certificate-native-compile':0,'solve-certificate-native':0,'solve-certificate-memcheck':0,
  'charpoly-certificate-native-compile':0,'charpoly-certificate-native':0,'charpoly-certificate-memcheck':0,
  'ca-domain-native-compile':1,'ca-domain-native-compile-corrected':0,'ca-domain-native':1,'ca-domain-memcheck':1,
  'ca-arg-witness-compile':0,'ca-arg-witness-native':1,'ca-arg-witness-memcheck':1,
  'ca-negative-arg-witness-compile':0,'ca-negative-arg-witness-native':0,'ca-negative-arg-witness-memcheck':0,
  'hyper-domain-debug':0,'hyper-domain-release':0,'hyper-domain-memcheck':0};
for(const [tag,code]of Object.entries(codes)) {
  const g=json(`results/${tag}.json`);assert.equal(g.code,code,tag);assert.equal(g.signal,null);
  manifest.gates.push({tag,code});for(const ext of ['json','stdout','stderr'])files.push(`results/${tag}.${ext}`);
}
for(const p of files)manifest.files[p]=sha(p);
const coverage=json('coverage.json');
manifest.reads=json('charpoly-domain-read-selection.json').map(e=>{
  const v=coverage.find(v=>v.repo===e.repo&&v.path===e.path);assert(v);return v;
});
assert.equal(manifest.reads.length,36);
const retained=json('retained-monic.json');
for(const [p,h]of Object.entries(retained.liveSources)) {
  assert.equal(sha(resolve(workspace,p)),h,`live ${p}`);
  assert.equal(sha(`${retained.frozenSnapshot}/${p}`),h,`qualified ${p}`);
}
manifest.hyperReadRanges={
  'hypersolve/src/bareiss.rs':[[260,341]],
  'hypersolve/src/algebraic_fiber.rs':[[2030,2195]],
  'hyperreal/src/real/arithmetic/facts.rs':[[1080,1135]],
  'hyperreal/src/real/arithmetic/elementary_functions.rs':[[2990,3163]]};
const root='/tmp/calcium-charpoly-solves.lbZlN8';
for(const p of ['flint-solve-certificate','flint-charpoly-certificate','flint-ca-domain',
  'flint-ca-arg-witness','flint-ca-negative-arg-witness','hyper-domain-debug','hyper-domain-release'])
  manifest.binaries[`${root}/${p}`]={sha256:sha(`${root}/${p}`),bytes:statSync(`${root}/${p}`).size};
for(const profile of ['debug','release'])assert.equal(
  sha(`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/${profile}/calcium-hyper-domain-probe`),
  manifest.binaries[`${root}/hyper-domain-${profile}`].sha256);
const prior=json('rank-cost-experiment.json');
manifest.nativeLibrary={path:prior.nativeLibrary.path,sha256:sha(prior.nativeLibrary.path)};
assert.deepEqual(manifest.nativeLibrary,prior.nativeLibrary);
if(!draft)writeFileSync(resolve(here,'charpoly-domain-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'charpoly domain binding',draft,files:files.length,gates:manifest.gates.length,
  reads:manifest.reads.length,binaries:Object.keys(manifest.binaries).length,
  binaryBytes:Object.values(manifest.binaries).reduce((n,b)=>n+b.bytes,0)}));
export {manifest};
