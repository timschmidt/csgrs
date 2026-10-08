import assert from 'node:assert/strict';
import {readFileSync, statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {execFileSync} from 'node:child_process';

const workspace = '/home/tim/Documents/GitHub/workspace';
const c = JSON.parse(readFileSync(`${workspace}/hypercurve/benchmarks/checkpoints/2026-09-05-native-polynomial-kernel.json`, 'utf8'));
const hash = value => createHash('sha256').update(value).digest('hex');
const run = script => JSON.parse(execFileSync(process.execPath, [script], {encoding: 'utf8', maxBuffer: 5 * 1024 * 1024}));
const git = (repo, ...args) => execFileSync('git', ['-C', `${workspace}/${repo}`, ...args]);

assert.equal(c.change.source_files.length, 8);
assert.equal(c.change.removed_helpers.length, 6);
assert.equal(c.change.source_files.reduce((sum, row) => sum + row.production_lines.delta, 0), -56);
assert.equal(c.change.source_files.reduce((sum, row) => sum + row.total_lines.delta, 0), 29);
const laterEdits = [];
for (const row of c.change.source_files) {
    const [repo, ...parts] = row.file.split('/');
    const file = parts.join('/');
    assert.equal(hash(git(repo, 'show', `${c.commits[`${repo}_parent`]}:${file}`)), row.before_sha256, `parent ${row.file}`);
    assert.equal(hash(git(repo, 'show', `${c.commits[`${repo}_candidate`]}:${file}`)), row.candidate_sha256, `commit ${row.file}`);
    assert.equal(hash(readFileSync(`${c.measurement_environment.frozen_workspace}/${row.file}`)), row.candidate_sha256, `frozen ${row.file}`);
    if (hash(readFileSync(`${workspace}/${row.file}`)) !== row.candidate_sha256) laterEdits.push(row.file);
}
for (const repo of ['hyperreal', 'hypersolve', 'hypercurve']) {
    assert.equal(git(repo, 'branch', '--show-current').toString().trim(), 'main');
    git(repo, 'merge-base', '--is-ancestor', c.commits[`${repo}_candidate`], 'HEAD');
}
assert.deepEqual(c.validation.test_runs, run('/tmp/hypercurve-native-horner-qualification.mjs'));
assert.equal(Object.keys(c.validation.test_runs).length, 11);
for (const row of Object.values(c.validation.test_runs)) {
    assert.ok(row.passed > 0);
    assert.equal(row.failed, 0);
    assert.deepEqual(row.failures, []);
}

function inflate(results) {
    const x = structuredClone(results);
    for (const fixture of Object.values(x)) {
        for (const variant of ['baseline', 'candidate']) {
            for (const sample of fixture[variant]) {
                if (!sample.row_nanoseconds) continue;
                assert.equal(sample.row_nanoseconds.length, fixture.median_rows.length);
                sample.rows = fixture.median_rows.map((row, index) => ({
                    name: row.name,
                    iterations: row.iterations,
                    nanoseconds_per_iteration: sample.row_nanoseconds[index],
                    ...(row.checksum === undefined ? {} : {checksum: row.checksum}),
                }));
                delete sample.row_nanoseconds;
            }
        }
    }
    return x;
}
let invocations = 0;
for (const [stored, parser] of [
    [c.performance.broad.results, '/tmp/hypercurve-native-horner-analyze.mjs'],
    [c.performance.extended_rational.results, '/tmp/hypercurve-native-horner-extended-analyze.mjs'],
    [c.performance.rejected_intermediate.broad, '/tmp/hypercurve-native-eval-analyze.mjs'],
    [c.performance.rejected_intermediate.extended_rational, '/tmp/hypercurve-native-eval-extended-analyze.mjs'],
]) {
    assert.deepEqual(inflate(stored), run(parser), parser);
    for (const fixture of Object.values(stored)) invocations += fixture.baseline.length + fixture.candidate.length;
}
assert.equal(invocations, 144);
let artifactCount = 0;
for (const variant of Object.values(c.binary_sizes.artifacts)) {
    for (const artifact of Object.values(variant)) {
        assert.equal(statSync(artifact.path).size, artifact.bytes, artifact.path);
        assert.equal(hash(readFileSync(artifact.path)), artifact.sha256, artifact.path);
        artifactCount++;
    }
}
assert.equal(artifactCount, 9);
let heapCount = 0;
for (const fixture of Object.values(c.memory.controls)) {
    for (const variant of Object.values(fixture)) {
        assert.equal(hash(readFileSync(variant.recording)), variant.recording_sha256, variant.recording);
        heapCount++;
    }
}
assert.equal(heapCount, 4);
for (const [name, file] of [['region', 'bezier_region.rs'], ['rational', 'rational_bezier.rs']]) {
    assert.equal(hash(readFileSync(`${c.measurement_environment.frozen_workspace}/hypercurve/benches/${file}`)), c.measurement_environment.driver_sha256[name]);
}
assert.equal(hash(readFileSync(c.known_completeness_limit.source)), c.known_completeness_limit.source_sha256);
assert.deepEqual(readFileSync(c.known_completeness_limit.final_log, 'utf8').trim().split('\n'), c.known_completeness_limit.final_output);
console.log('Validated 8 committed/frozen source hashes, 11 test summaries, all 144 paired invocations, 9 binary artifacts, 4 heap recordings, and the known-limit probe.');
console.log(`Later live edits excluded from qualification: ${JSON.stringify(laterEdits)}`);
