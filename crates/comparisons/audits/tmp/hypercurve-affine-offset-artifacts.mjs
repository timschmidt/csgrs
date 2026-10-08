import {readFileSync, statSync} from 'node:fs';
import {createHash} from 'node:crypto';

const hash = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const artifact = path => ({path, bytes: statSync(path).size, sha256: hash(path)});
const binaries = {};
for (const [variant, prefix] of [['baseline', 'native-horner'], ['candidate', 'affine-offset']]) {
    binaries[variant] = Object.fromEntries(['test', 'region', 'rational'].map(kind => [kind,
        artifact(`/tmp/hypercurve-${prefix}-frozen-candidate-${kind}`),
    ]));
}
function heap(prefix) {
    const summary = `${prefix}-heap-summary.log`;
    const text = readFileSync(summary, 'utf8');
    const number = expression => Number(text.match(expression)[1]);
    const recording = `${prefix}.heap.zst`;
    return {
        allocations: number(/calls to allocation functions: (\d+)/),
        temporary_allocations: number(/temporary memory allocations: (\d+)/),
        peak_heap_reported: text.match(/peak heap memory consumption: (\S+)/)[1],
        end_live_heap_reported: text.match(/total memory leaked: (\S+)/)[1],
        recording, recording_sha256: hash(recording), summary,
    };
}
const memory = {};
for (const fixture of ['small', 'rational']) {
    memory[fixture] = Object.fromEntries(['baseline', 'candidate'].map(variant => [variant,
        heap(`/tmp/hypercurve-affine-offset-${variant}-${fixture}`),
    ]));
}
const micro_memory = {};
for (const iterations of [4, 16, 64]) {
    micro_memory[iterations] = Object.fromEntries(['baseline', 'candidate'].map(variant => [variant,
        heap(`/tmp/hyperreal-affine-offset-${variant}-micro${iterations === 4 ? '' : iterations}`),
    ]));
}
console.log(JSON.stringify({binaries, memory, micro_memory}, null, 2));
