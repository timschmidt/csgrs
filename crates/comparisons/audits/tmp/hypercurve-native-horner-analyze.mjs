import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const median = values => [...values].sort((a, b) => a - b)[1];
const change = (before, after) => before === 0 ? null : 100 * (after / before - 1);
const shape = rows => rows.map(({name, iterations, checksum}) => ({name, iterations, checksum}));
const result = {};
for (const fixture of ['selected', 'chord', 'radial', 'region', 'rational4', 'rational16', 'rational64']) {
    const samples = {};
    for (const version of ['baseline', 'candidate']) {
        samples[version] = [1, 2, 3].map(repetition => {
            const log = `/tmp/hypercurve-native-horner-${fixture}-${version}-${repetition}.log`;
            const text = readFileSync(log, 'utf8');
            assert.match(text, /Exit status: 0\s*$/);
            const field = expression => {
                const match = text.match(expression);
                assert.ok(match, `${log}: missing ${expression}`);
                return match[1];
            };
            const sample = {
                log, repetition, exit_status: 0,
                wall_seconds: field(/Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): (\S+)/).split(':').reduce((total, part) => total * 60 + Number(part), 0),
                user_seconds: Number(field(/User time \(seconds\): ([\d.]+)/)),
                system_seconds: Number(field(/System time \(seconds\): ([\d.]+)/)),
                maximum_rss_kib: Number(field(/Maximum resident set size \(kbytes\): (\d+)/)),
            };
            if ((fixture === 'region' || fixture.startsWith('rational'))) {
                sample.rows = [...text.matchAll(/^(\w+): (\d+) iterations in ([\d.]+)(ns|µs|ms|s) \([^\n]+\)(?:, checksum=(\d+))?$/gm)].map(match => ({
                    name: match[1], iterations: Number(match[2]),
                    nanoseconds_per_iteration: Number(match[3]) * ({ns: 1, 'µs': 1e3, ms: 1e6, s: 1e9}[match[4]]) / Number(match[2]),
                    ...(match[5] === undefined ? {} : {checksum: Number(match[5])}),
                }));
                assert.equal(sample.rows.length, fixture === 'region' ? 19 : 2, log);
            } else {
                assert.match(text, /test result: ok\. 1 passed; 0 failed; 0 ignored;/);
                sample.passed = 1;
            }
            return sample;
        });
    }
    const summary = {};
    for (const metric of ['wall_seconds', 'user_seconds', 'system_seconds', 'maximum_rss_kib']) {
        const baseline = median(samples.baseline.map(sample => sample[metric]));
        const candidate = median(samples.candidate.map(sample => sample[metric]));
        summary[metric] = {baseline, candidate, change_percent: change(baseline, candidate)};
    }
    result[fixture] = {...samples, median: summary};
    if ((fixture === 'region' || fixture.startsWith('rational'))) {
        const first = samples.baseline[0].rows;
        for (const sample of [...samples.baseline, ...samples.candidate]) assert.deepEqual(shape(sample.rows), shape(first));
        result[fixture].median_rows = first.map((row, index) => {
            const baseline = median(samples.baseline.map(sample => sample.rows[index].nanoseconds_per_iteration));
            const candidate = median(samples.candidate.map(sample => sample.rows[index].nanoseconds_per_iteration));
            return {...shape([row])[0], baseline_nanoseconds: baseline, candidate_nanoseconds: candidate, change_percent: change(baseline, candidate)};
        });
    }
}
console.log(JSON.stringify(result, null, 2));
