import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';

const file = process.argv[2];
const before = fs.readFileSync(0, 'utf8');
const live = fs.readFileSync(file, 'utf8');
const frozen = fs.readFileSync(`/tmp/hypercurve-dyadic-bernstein-control.a5ZKq2/${file}`, 'utf8');
assert.equal(live, frozen, `${file}: frozen change differs from live source`);
const hash = source => crypto.createHash('sha256').update(source).digest('hex');
const lines = source => source.split('\n').length - 1;
const production = source => file.startsWith('hyperreal/') ? (file.endsWith('/tests.rs') ? '' : source) : file.endsWith('/bezier_algebraic_image.rs')
    ? source.replace(/\n#\[cfg\(test\)\][\s\S]*?\nfn compare_root_representation_to_real/, '\nfn compare_root_representation_to_real')
    : source.split('\n#[cfg(test)]')[0];
console.log(JSON.stringify({
    file, before_sha256: hash(before), candidate_sha256: hash(live),
    total_lines: {before: lines(before), after: lines(live), delta: lines(live) - lines(before)},
    production_lines: {before: lines(production(before)), after: lines(production(live)), delta: lines(production(live)) - lines(production(before))},
}));
