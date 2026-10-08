// Mechanical reuse of the already executed independent polynomial-ring oracle.
// The original checker and capture remain unchanged.
import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const source=readFileSync('check-power-sums-kernel.mjs','utf8');
const cut=source.indexOf('export function checkPowerSumsKernel(path)');assert(cut>0);
const derived=source.slice(0,cut)+'\nselfTest();\nexport {oracle};\n';
writeFileSync('power-sums-polynomial-oracle.mjs',derived,{flag:'wx'});
console.log(JSON.stringify({source:'check-power-sums-kernel.mjs',sourceSha256:createHash('sha256').update(source).digest('hex'),
 derived:'power-sums-polynomial-oracle.mjs',prefixBytes:Buffer.byteLength(source.slice(0,cut)),
 derivedBytes:Buffer.byteLength(derived),note:'Identical oracle core; only self-test call and export appended. No mathematical change.'}));
