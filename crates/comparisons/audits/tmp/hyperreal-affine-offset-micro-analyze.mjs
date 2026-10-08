import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const median = values => [...values].sort((a, b) => a - b)[2];
const samples = {};
for (const variant of ['baseline', 'candidate']) {
    samples[variant] = [1, 2, 3, 4, 5].map(repetition => {
        const log = `/tmp/hyperreal-affine-offset-micro-${variant}-${repetition}.log`;
        const text = readFileSync(log, 'utf8');
        assert.match(text, /Exit status: 0\s*$/);
        const rows = [...text.matchAll(/^(\w+),(\d+),([\d.]+),(\d+)$/gm)].map(row => ({
            name: row[1], iterations: Number(row[2]),
            nanoseconds_per_iteration: Number(row[3]), checksum: row[4],
        }));
        assert.equal(rows.length, 4);
        return {log, repetition, rows};
    });
}
const shape = rows => rows.map(({name, iterations, checksum}) => ({name, iterations, checksum}));
const reference = shape(samples.baseline[0].rows);
for (const sample of [...samples.baseline, ...samples.candidate]) assert.deepEqual(shape(sample.rows), reference);
const median_rows = reference.map((row, index) => {
    const baseline_nanoseconds = median(samples.baseline.map(sample => sample.rows[index].nanoseconds_per_iteration));
    const candidate_nanoseconds = median(samples.candidate.map(sample => sample.rows[index].nanoseconds_per_iteration));
    return {...row, baseline_nanoseconds, candidate_nanoseconds, change_percent: 100 * (candidate_nanoseconds / baseline_nanoseconds - 1)};
});
console.log(JSON.stringify({...samples, median_rows}, null, 2));
