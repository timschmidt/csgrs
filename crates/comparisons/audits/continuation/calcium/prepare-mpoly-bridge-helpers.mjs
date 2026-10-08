import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const c=readFileSync('flint-mpoly-rational-controls-v2.c','utf8');
const js=readFileSync('check-mpoly-rational.mjs','utf8');
const cEnd=c.indexOf('static void context('),jsEnd=js.indexOf('export function checkMpolyRational()');
assert(cEnd>0&&jsEnd>0);
writeFileSync('mpoly-bridge-helpers.h',c.slice(0,cEnd),{flag:'wx'});
writeFileSync('mpoly-bridge-math.mjs',js.slice(0,jsEnd)+
 'export {constant,add,mul,scale,factors,recipes,operation,decode,valueCheck,same};\n',{flag:'wx'});
console.log(JSON.stringify({cBytes:Buffer.byteLength(c.slice(0,cEnd)),mathPrefixBytes:Buffer.byteLength(js.slice(0,jsEnd)),
 derivation:'Exact prefixes of immutable checkpoint46 sources, plus explicit JS exports. No mathematical changes.'}));
