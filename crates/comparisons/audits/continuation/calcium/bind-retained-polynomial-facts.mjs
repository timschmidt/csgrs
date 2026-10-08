import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const json = p => JSON.parse(readFileSync(resolve(here, p)));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const files = {};
for (const path of ['polynomial-facts-experiment.json', 'reuse-consumers.json', 'retained-exp-proof.json',
  'bind-retained-polynomial-facts.mjs', 'verify-retained-polynomial-facts.mjs']) files[path] = sha(resolve(here, path));
const gates = ['debug', 'release', 'clippy', 'fmt', 'wasm', 'metadata'];
for (const gate of gates) {
  const tag = `retained-polynomial-facts-${gate}`, r = json(`results/${tag}.json`);
  assert.equal(r.code, 0, tag); assert.equal(r.cwd, resolve(workspace, 'hypersolve'));
  for (const ext of ['json', 'stdout', 'stderr']) {
    const path = `results/${tag}.${ext}`; files[path] = sha(resolve(here, path));
  }
}
const liveSources = {}, changed = [];
for (const { crate, files } of json('reuse-consumers.json').crates) for (const file of files) {
  const path = `${crate}/${file.path}`, hash = sha(resolve(here, 'polynomial-facts-trial', path));
  assert.equal(sha(resolve(workspace, path)), hash, path); liveSources[path] = hash;
  if (hash !== file.sha256) changed.push(path);
}
assert.deepEqual(changed, ['hypersolve/src/resultant.rs']);
for (const [file, hash] of Object.entries(json('retained-exp-proof.json').liveSourceHashes)) {
  const path = `hyperreal/${file}`;
  assert.equal(sha(resolve(workspace, path)), hash, path);
  assert.equal(sha(resolve(here, 'polynomial-facts-trial', path)), hash, path);
  liveSources[path] = hash;
}
assert.equal(Object.keys(liveSources).length, 953);
writeFileSync(resolve(here, 'retained-polynomial-facts.json'), JSON.stringify({ schema: 1, recorded: new Date().toISOString(),
  status: 'Fact-first nonzero-dominates-Unknown polynomial decision retained in live Hypersolve; full ecosystem audit remains incomplete.',
  files, gates, liveSources, changed, frozenSnapshot: 'polynomial-facts-trial',
  acceptedCosts: 'Same-outcome CPU paired medians range0.973–1.029 baseline on this host; newly decided cases do more work than baseline Unknown. Degree16 retained log-self4.549us versus67.286us v1, with13 versus1117 requested allocations. Stripped Hypercurve examples grow576/560 bytes. No new public scalar state, dependency or representation; production delta103 added/two removed source lines including three tests.',
  limits: '800 solver tests per profile; frozen identical downstream Hypercurve1761 passed/nine pre-existing ignored. Public630 and state1090 queries per profile, native4860 checks, CPU2592 and bounded allocation648 plus27 churn observations. All-feature Clippy, fmt and WASM compile-only live gates. Not full CI, whole-application size, concurrent performance, peak RSS or universal performance proof. Historical failed allocation assumption and rejected v1 schedule preserved. Optional --retained-live checks current live bytes without making historical verification fail after future authorized changes.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sourceFiles: Object.keys(liveSources).length, changed, gates: gates.length, boundFiles: Object.keys(files).length }));
