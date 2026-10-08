import {existsSync, readFileSync} from 'node:fs';

const result = {};
for (const [crate, suffixes] of [
    ['hyperreal', ['default', 'all-targets', 'release']],
    ['hypersolve', ['minimal', 'all-targets', 'release']],
    ['hypercurve', ['all-features', 'minimal', 'ignored-units', 'ignored-stroke', 'ui-tests']],
]) {
    for (const suffix of suffixes) {
        const phase = crate === 'hypercurve' ? 'affine-offset-final' : 'affine-offset';
        const path = `/tmp/${crate}-${phase}-${suffix}.log`;
        if (!existsSync(path)) continue;
        const text = readFileSync(path, 'utf8');
        const rows = [...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;[^\n]*finished in ([\d.]+)s/g)];
        result[`${crate}-${suffix}`] = {
            path, targets: rows.length,
            passed: rows.reduce((n, row) => n + Number(row[1]), 0),
            failed: rows.reduce((n, row) => n + Number(row[2]), 0),
            ignored: rows.reduce((n, row) => n + Number(row[3]), 0),
            failures: [...text.matchAll(/test result: FAILED[^\n]*/g)].map(row => row[0]),
            longest_targets: rows.map(row => ({passed: Number(row[1]), seconds: Number(row[4])})).sort((a, b) => b.seconds - a.seconds).slice(0, 6),
            tail: text.trimEnd().split('\n').slice(-3),
        };
    }
}
console.log(JSON.stringify(result, null, 2));
