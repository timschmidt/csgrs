import fs from 'node:fs';
import crypto from 'node:crypto';

const hash = path => crypto.createHash('sha256').update(fs.readFileSync(path)).digest('hex');
const artifacts = {};
for (const [variant, prefix] of [
    ['baseline', '/tmp/hypercurve-root-evidence-frozen-candidate'],
    ['rejected', '/tmp/hypercurve-native-eval-frozen-candidate'],
    ['candidate', '/tmp/hypercurve-native-horner-frozen-candidate'],
]) {
    artifacts[variant] = {};
    for (const kind of ['test', 'region', 'rational']) {
        const path = variant === 'baseline' && kind === 'rational'
            ? '/tmp/hypercurve-native-eval-frozen-baseline-rational'
            : `${prefix}-${kind}`;
        artifacts[variant][kind] = {path, bytes: fs.statSync(path).size, sha256: hash(path)};
    }
}
const allocation_controls = {};
for (const fixture of ['small', 'rational']) {
    allocation_controls[fixture] = {};
    for (const version of ['baseline', 'candidate']) {
        const prefix = `/tmp/hypercurve-native-horner-${version}-${fixture}`;
        const summary = fs.readFileSync(`${prefix}-heap-summary.log`, 'utf8');
        const field = pattern => {
            const match = summary.match(pattern);
            if (!match) throw new Error(`Missing heap field: ${pattern}`);
            return match[1];
        };
        const output = fs.readFileSync(`${prefix}-heaptrack.log`, 'utf8');
        if (fixture === 'small' && !output.includes('test result: ok. 1 passed; 0 failed;')) throw new Error('Small heap fixture failed');
        if (fixture === 'rational' && (output.match(/4 iterations/g) ?? []).length !== 2) throw new Error('Rational heap rows incomplete');
        allocation_controls[fixture][version] = {
            allocations: Number(field(/calls to allocation functions: (\d+)/)),
            temporary_allocations: Number(field(/temporary memory allocations: (\d+)/)),
            peak_heap_reported: field(/peak heap memory consumption: (\S+)/),
            end_live_heap_reported: field(/total memory leaked: (\S+)/),
            recording: `${prefix}.heap.zst`, recording_sha256: hash(`${prefix}.heap.zst`),
            summary: `${prefix}-heap-summary.log`, output: `${prefix}-heaptrack.log`,
        };
    }
}
console.log(JSON.stringify({artifacts, allocation_controls}));
