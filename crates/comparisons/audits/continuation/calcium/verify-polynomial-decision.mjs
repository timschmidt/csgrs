import './verify-retained-exp-proof.mjs';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = path => readFileSync(resolve(here, path), 'utf8');
const json = path => JSON.parse(read(path));
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const manifest = json('polynomial-decision-experiment.json');
for (const [path, hash] of Object.entries(manifest.files)) assert.equal(sha(resolve(here, path)), hash, path);
for (const [path, hash] of Object.entries(manifest.binaries)) assert.equal(sha(path), hash, path);
const sources = json('polynomial-decision-baseline-sources.json');
const changed = [];
for (const { crate, files } of sources.crates) for (const file of files) {
  const path = `${crate}/${file.path}`;
  assert.equal(sha(resolve(here, 'reuse-consumers', path)), file.sha256);
  if (process.argv.includes('--live')) assert.equal(sha(resolve(workspace, path)), file.sha256);
  if (sha(resolve(here, 'polynomial-decision-trial', path)) !== file.sha256) changed.push(path);
}
assert.deepEqual(changed, ['hypersolve/src/resultant.rs']);
const inventory = json('inventory.json'), coverage = json('coverage.json');
assert.equal(manifest.reads.length, 40);
let readLines = 0;
for (const entry of manifest.reads) {
  assert.deepEqual(coverage.find(c => c.repo === entry.repo && c.path === entry.path), entry);
  const source = inventory.sources.find(s => s.repo === entry.repo);
  const file = source.files.find(f => f.path === entry.path);
  assert.deepEqual(entry.ranges, [[1, file.lines]]);
  assert.equal(sha(resolve(workspace, 'exact-real-references', entry.repo, entry.path)), file.sha256);
  readLines += file.lines;
}
assert.equal(readLines, 4159);
const result = tag => json(`results/polynomial-decision-${tag}.json`);
for (const tag of manifest.gates) assert.equal(result(tag).code, 0, tag);
assert(!result('trial-tests-release').args.includes('--release'), 'mislabeled run must stay classified as debug');
assert(result('trial-tests-release-corrected').args.includes('--release'));

function corpus(tag, candidate) {
  const rows = read(`results/polynomial-decision-${tag}.stdout`).trim().split('\n');
  assert.equal(rows.shift(), 'kind,degree,index,relation,floor,coefficient_known,outcome,result_degree,steps');
  assert.equal(rows.length, 630);
  let next = 0, knownCount = 0;
  for (let kind = 0; kind < 7; kind++) for (let degree = 1; degree <= 4; degree++) {
    for (let index = 0; index < degree; index++) for (const relation of ['self', 'plus-one', 'unknown-leading-control']) {
      for (const floor of [-32, -128, -512]) {
        const m = /^(\d+),(\d+),(\d+),([^,]+),(-\d+),(true|false),(Known|Unknown:.*),(-?\d+),(\d+)$/.exec(rows[next++]);
        assert(m);
        assert.deepEqual(m.slice(1, 6), [kind, degree, index, relation, floor].map(String));
        const coefficientKnown = ![3, 6].includes(kind);
        assert.equal(m[6], String(coefficientKnown));
        const known = coefficientKnown || (candidate && relation !== 'unknown-leading-control');
        assert.equal(m[7] === 'Known', known);
        if (known) {
          const expectedDegree = relation === 'plus-one' || (kind === 0 && relation === 'unknown-leading-control') ? 0 : degree;
          assert.equal(Number(m[8]), expectedDegree);
          assert.equal(Number(m[9]), kind === 0 && relation === 'unknown-leading-control' ? 0 : 1);
          knownCount++;
        } else {
          assert.equal(m[7], `Unknown:UndecidedCoefficient { side: Left, index: ${relation === 'unknown-leading-control' ? degree : index} }`);
          assert.equal(m[8], '-1'); assert.equal(m[9], '0');
        }
      }
    }
  }
  assert.equal(knownCount, candidate ? 570 : 450);
  return rows;
}
assert.deepEqual(corpus('baseline-debug', false), corpus('baseline-release', false));
const trialRows = corpus('trial-debug-final', true);
assert.deepEqual(trialRows, corpus('trial-debug', true));
assert.deepEqual(trialRows, corpus('trial-release', true));
assert.deepEqual(trialRows, corpus('trial-probe-memcheck', true));

const tests = tag => [...read(`results/${tag}.stdout`).matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m => `${m[1]}:${m[2]}`).sort();
const baselineTests = tests('polynomial-decision-baseline-tests-debug');
assert.equal(baselineTests.length, 797);
assert.deepEqual(baselineTests, tests('reuse-consumer-hypersolve-debug'));
const addedTests = ['polynomial_zero_truth_lets_nonzero_dominate_unknown',
  'subresultant_chain_keeps_unknown_lower_coefficients', 'subresultant_chain_still_rejects_unknown_leading_degree']
  .map(name => `resultant::tests::${name}:ok`);
for (const tag of ['trial-tests-debug', 'trial-tests-release', 'trial-tests-release-corrected']) {
  const actual = tests(`polynomial-decision-${tag}`);
  assert.deepEqual(actual, [...baselineTests, ...addedTests].sort());
  const suites = [...read(`results/polynomial-decision-${tag}.stdout`).matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)];
  assert.equal(suites.length, 7);
  assert.equal(suites.reduce((n, m) => n + Number(m[1]), 0), 800);
  assert(suites.every(m => m[2] === '0' && m[3] === '0'));
}
const nativeText = read('results/polynomial-decision-native.stdout');
assert.equal(nativeText, read('results/polynomial-decision-native-memcheck.stdout'));
const native = nativeText.trim().split('\n');
assert.equal(native.shift(), 'code,length,zero,one,proper,monic,correct');
assert.deepEqual(JSON.parse(native.pop()), { suite: 'polynomial-decision', cases: 81, checks: 405, failed_cases: 0 });
assert.equal(native.length, 81);
native.forEach((line, code) => {
  const digits = Array.from({ length: 4 }, (_, i) => Math.floor(code / (3 ** i)) % 3);
  const length = digits.findLastIndex(d => d !== 0) + 1;
  const zero = digits.includes(1) ? 1 : digits.includes(2) ? 2 : 0;
  const one = digits[0] === 0 || digits.slice(1).includes(1) ? 1 : digits.includes(2) ? 2 : 0;
  assert.deepEqual(line.split(',').map(Number), [code, length, zero, one, Number(!digits.includes(2)), Number(length > 0 && digits[length - 1] === 1), 1]);
});
for (const tag of ['native-memcheck', 'trial-probe-memcheck']) {
  const stderr = read(`results/polynomial-decision-${tag}.stderr`);
  assert.match(stderr, /ERROR SUMMARY: 0 errors/);
  assert(!/definitely lost:\s+[1-9]|indirectly lost:\s+[1-9]|possibly lost:\s+[1-9]/.test(stderr));
}
assert.match(read('results/polynomial-decision-native-memcheck.stderr'), /All heap blocks were freed/);

const summary = json('polynomial-decision-cpu-summary.json');
const rows = read('results/polynomial-decision-cpu.jsonl').trim().split('\n').map(JSON.parse);
assert.equal(rows.length, 1728); assert.equal(summary.summaries.length, 36);
const median = input => { const a = [...input].sort((x, y) => x - y); return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2; };
let seed = 997, offset = 0;
const random = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
const expectedGroups = [];
for (const name of ['rational-self', 'radical-self', 'log-self', 'log-plus-one', 'tiny-self', 'unknown-leading'])
  for (const degree of [1, 8, 16]) for (const lifecycle of ['fresh', 'retained']) expectedGroups.push([name, degree, lifecycle]);
summary.summaries.forEach((s, group) => {
  assert.deepEqual([s.name, s.degree, s.lifecycle], expectedGroups[group]);
  assert.equal(s.iterations, Math.max(10, Math.min(10000, Math.ceil(1e7 / Math.max(...s.pilots.map(p => p.elapsed_ns / p.iterations))))));
  const selected = rows.slice(offset, offset += 48);
  for (let block = 0; block < 12; block++) {
    const order = block % 2 ? ['trial', 'baseline', 'baseline', 'trial'] : ['baseline', 'trial', 'trial', 'baseline'];
    selected.slice(block * 4, block * 4 + 4).forEach((r, i) => {
      assert.deepEqual([r.case, r.degree, r.lifecycle, r.block, r.variant, r.iterations], [s.name, s.degree, s.lifecycle, block, order[i], s.iterations]);
      const known = ['rational-self', 'radical-self'].includes(s.name) || (r.variant === 'trial' && s.name !== 'unknown-leading');
      assert.equal(r.known, known ? r.iterations : 0); assert(r.elapsed_ns > 0);
    });
  }
  const measurements = Object.fromEntries(['baseline', 'trial'].map(v => [v, median(selected.filter(r => r.variant === v).map(r => r.elapsed_ns / r.iterations))]));
  assert.deepEqual(s.measurements, measurements);
  const ratios = Array.from({ length: 12 }, (_, block) => {
    const time = v => median(selected.filter(r => r.block === block && r.variant === v).map(r => r.elapsed_ns));
    return time('trial') / time('baseline');
  });
  const boot = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[random(12)]))).sort((a, b) => a - b);
  assert.equal(s.pairedMedianRatio, median(ratios)); assert.deepEqual(s.pairedMedianBootstrap95, [boot[125], boot[4875]]);
});
const start = Date.parse(summary.started), finish = Date.parse(summary.finished);
const fmtFinish = Date.parse(result('trial-fmt').finished);
assert(fmtFinish > start); // Recorded overlap: exclude the entire first group.
const firstGroupMinimumElapsed = rows.slice(0, 48).reduce((n, r) => n + r.elapsed_ns / 1e6, 0);
assert(start + firstGroupMinimumElapsed - 1 > fmtFinish, 'all subsequent groups must begin after formatting');
for (const tag of manifest.gates.filter(t => t !== 'cpu-run' && t !== 'trial-fmt')) {
  const r = result(tag);
  assert(Date.parse(r.finished) <= start || Date.parse(r.started) >= finish, `CPU overlap: ${tag}`);
}
for (const binary of Object.values(summary.binaries)) assert.equal(sha(binary.path), binary.sha256);
console.log(JSON.stringify({ checkpoint: 'polynomial degree and three-valued zero decisions',
  readFiles: 40, readLines, publicCasesPerVariantPerProfile: 630, newDecisions: 120,
  preservedUnknownLeadingControls: 60, baselineTests: 797, candidateTestsPerProfile: 800,
  nativeTruthChecks: 405, cpuObservations: 1728, excludedFirstGroupObservations: 48,
  acceptedCPUObservations: 1680, status: 'Isolated candidate, not retained; full audit remains active.',
  limits: 'Repeated degree/position/budget cases are not distinct mathematical identities. First CPU group overlaps formatting and is excluded. Different proof outcomes are not equal-work speedups. No allocation campaign, broader downstream qualification, application-size or serialized/concurrent candidate qualification yet. Mislabeled initial release test is preserved as debug. Candidate needs reuse of already-certified nonzero facts before a retention decision.' }));
