import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';

const path = '/home/tim/Documents/GitHub/workspace/hypercurve/benchmarks/checkpoints/2026-09-05-root-evidence-api.json';
const data = JSON.parse(readFileSync(path, 'utf8'));
for (const value of Object.values(data.commits)) assert.match(value, /^[a-f0-9]{40}$/);
assert.equal(data.change.hypersolve_production_line_delta + data.change.hypercurve_production_line_delta, data.change.production_line_delta);
const coverage = data.native_validation.coverage;
assert.equal(coverage.hit_lines + coverage.missed_lines, coverage.executable_lines);
assert.equal(Number((100 * coverage.hit_lines / coverage.executable_lines).toFixed(2)), coverage.percent);
assert.ok(coverage.percent >= coverage.threshold_percent);
let count = 0;
for (const [name, samples] of Object.entries(data.matched_comparison.measurements)) {
    for (const side of ['baseline', 'candidate']) {
        assert.equal(samples[side].length, 3);
        for (const sample of samples[side]) {
            count++;
            assert.equal(sample.exit_status, 0);
            assert.match(readFileSync(sample.log, 'utf8'), /Exit status: 0\s*$/);
            if (name !== 'region') assert.equal(sample.passed, 1);
        }
        for (const metric of ['wall_seconds', 'user_seconds', 'system_seconds', 'maximum_rss_kib']) {
            const values = samples[side].map(sample => sample[metric]).sort((a, b) => a - b);
            assert.equal(samples.median[metric][side], values[1]);
        }
    }
}
assert.equal(count, 24);
const region = data.matched_comparison.measurements.region;
const shape = rows => rows.map(({name, iterations, checksum}) => ({name, iterations, checksum}));
for (const sample of [...region.baseline, ...region.candidate]) {
    assert.equal(sample.rows.length, 19);
    assert.deepEqual(shape(sample.rows), shape(region.baseline[0].rows));
}
for (const side of ['baseline_region', 'candidate_region']) {
    const sections = data.frozen_control[side].sections;
    assert.equal(sections.text + sections.data + sections.bss, sections.total);
}
for (const source of data.source_hashes) {
    const actual = createHash('sha256').update(readFileSync(`/home/tim/Documents/GitHub/workspace/${source.path}`)).digest('hex');
    assert.equal(actual, source.live, source.path);
}
assert.equal(data.local_pages.report.passed, true);
assert.equal(data.local_pages.previous_screenshot_equal, true);
assert.equal(data.matched_comparison.final_workers.length, 0);
console.log('Verified commits, current source hashes, coverage/size arithmetic, 24 samples, medians, 19 output shapes and browser result.');
