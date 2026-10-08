import assert from 'node:assert/strict';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
const median = values => [...values].sort((a, b) => a - b)[1];
const ratio = (before, after) => before === 0 ? null : 100 * (after / before - 1);
function sample(version, repetition) {
  const log = `/tmp/hypercurve-quotient-projection-api-region-${version}-${repetition}.log`;
  const content = readFileSync(log, 'utf8');
  assert.match(content, /Exit status: 0\s*$/);
  const field = pattern => { const match = content.match(pattern); assert.ok(match); return match[1]; };
  const rows = [...content.matchAll(/^(\w+): (\d+) iterations in ([\d.]+)(ns|µs|ms|s) \([^\n]+\)(?:, checksum=(\d+))?$/gm)].map(match => ({
    name: match[1], iterations: Number(match[2]),
    nanoseconds_per_iteration: Number(match[3]) * ({ns: 1, 'µs': 1e3, ms: 1e6, s: 1e9}[match[4]]) / Number(match[2]),
    ...(match[5] === undefined ? {} : {checksum: Number(match[5])}),
  }));
  assert.equal(rows.length, 19);
  return {log, repetition, rows, exit_status: 0,
    wall_seconds: field(/Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)/).split(':').reduce((total, part) => total * 60 + Number(part), 0),
    user_seconds: Number(field(/User time \(seconds\): ([\d.]+)/)),
    system_seconds: Number(field(/System time \(seconds\): ([\d.]+)/)),
    maximum_rss_kib: Number(field(/Maximum resident set size \(kbytes\): (\d+)/)),
  };
}
const baseline = [1, 2, 3].map(i => sample('baseline', i));
const candidate = [1, 2, 3].map(i => sample('candidate', i));
const shape = rows => rows.map(({name, iterations, checksum}) => ({name, iterations, checksum}));
for (const sample of [...baseline, ...candidate]) assert.deepEqual(shape(sample.rows), shape(baseline[0].rows));
const summary = {};
for (const metric of ['wall_seconds', 'user_seconds', 'system_seconds', 'maximum_rss_kib']) {
  const before = median(baseline.map(sample => sample[metric]));
  const after = median(candidate.map(sample => sample[metric]));
  summary[metric] = {baseline: before, candidate: after, change_percent: ratio(before, after)};
}
const rows = baseline[0].rows.map((row, i) => {
  const before = median(baseline.map(sample => sample.rows[i].nanoseconds_per_iteration));
  const after = median(candidate.map(sample => sample.rows[i].nanoseconds_per_iteration));
  return {...shape([row])[0], baseline_nanoseconds: before, candidate_nanoseconds: after, change_percent: ratio(before, after)};
});
const artifacts = {};
for (const [version, path] of [['baseline', '/tmp/hypercurve-native-constant-sign-region-admitted-benchmark'], ['candidate', '/tmp/hypercurve-quotient-projection-api-region-benchmark']]) {
  artifacts[version] = {path, bytes: statSync(path).size, sha256: createHash('sha256').update(readFileSync(path)).digest('hex')};
}
console.log(JSON.stringify({baseline, candidate, median: summary, median_rows: rows, artifacts}, null, 2));
