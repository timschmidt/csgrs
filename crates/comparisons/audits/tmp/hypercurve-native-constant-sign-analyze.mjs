import assert from 'node:assert/strict';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';

const prefix = '/tmp/hypercurve-native-constant-sign-admitted';
const median = values => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
const read = path => readFileSync(path, 'utf8');
function sample(fixture, version, repetition) {
  const log = `${prefix}-${fixture}-${version}-${repetition}.log`;
  const content = read(log);
  assert.match(content, /Exit status: 0\s*$/);
  const field = pattern => {
    const match = content.match(pattern);
    assert.ok(match, `${log}: ${pattern}`);
    return match[1];
  };
  if (fixture !== 'region') assert.match(content, /test result: ok\. 1 passed; 0 failed;/);
  const result = {
    repetition, log,
    wall_seconds: field(/Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)/).split(':').reduce((total, part) => 60 * total + Number(part), 0),
    user_seconds: Number(field(/User time \(seconds\): ([\d.]+)/)),
    system_seconds: Number(field(/System time \(seconds\): ([\d.]+)/)),
    maximum_rss_kib: Number(field(/Maximum resident set size \(kbytes\): (\d+)/)),
    exit_status: 0,
  };
  if (fixture === 'region') {
    result.rows = [...content.matchAll(/^(\w+): (\d+) iterations in ([\d.]+)(ns|µs|ms|s) \([^\n]+\)(?:, checksum=(\d+))?$/gm)].map(match => ({
      name: match[1], iterations: Number(match[2]),
      nanoseconds_per_iteration: Number(match[3]) * ({ns: 1, 'µs': 1e3, ms: 1e6, s: 1e9}[match[4]]) / Number(match[2]),
      ...(match[5] === undefined ? {} : {checksum: Number(match[5])}),
    }));
    assert.equal(result.rows.length, 19);
  }
  return result;
}
const fixtures = {};
for (const fixture of ['selected', 'chord', 'radial', 'region']) {
  const baseline = [1, 2, 3].map(i => sample(fixture, 'baseline', i));
  const candidate = [1, 2, 3].map(i => sample(fixture, 'candidate', i));
  const summary = {};
  for (const key of ['wall_seconds', 'user_seconds', 'system_seconds', 'maximum_rss_kib']) {
    const before = median(baseline.map(s => s[key]));
    const after = median(candidate.map(s => s[key]));
    summary[key] = {baseline: before, candidate: after, change_percent: 100 * (after / before - 1)};
  }
  fixtures[fixture] = {baseline, candidate, median: summary};
  if (fixture === 'region') {
    const shape = rows => rows.map(({name, iterations, checksum}) => ({name, iterations, checksum}));
    for (const s of [...baseline, ...candidate]) assert.deepEqual(shape(s.rows), shape(baseline[0].rows));
    fixtures[fixture].median_rows = baseline[0].rows.map((row, index) => {
      const before = median(baseline.map(s => s.rows[index].nanoseconds_per_iteration));
      const after = median(candidate.map(s => s.rows[index].nanoseconds_per_iteration));
      return {...shape([row])[0], baseline_nanoseconds: before, candidate_nanoseconds: after, change_percent: 100 * (after / before - 1)};
    });
  }
}
const artifacts = {};
for (const version of ['baseline', 'admitted']) {
  for (const kind of ['test', 'region']) {
    const path = kind === 'test' ? `/tmp/hypercurve-native-constant-sign-${version}-test` : `/tmp/hypercurve-native-constant-sign-region-${version}-benchmark`;
    const artifact = {path, file_bytes: statSync(path).size, sha256: createHash('sha256').update(readFileSync(path)).digest('hex')};
    const [text, data, bss, combined] = execFileSync('size', [path], {encoding: 'utf8'}).trim().split('\n')[1].trim().split(/\s+/).map(Number);
    artifact.sections = {text, data, bss, combined};
    artifacts[`${version}_${kind}`] = artifact;
  }
}
console.log(JSON.stringify({fixtures, artifacts}, null, 2));
