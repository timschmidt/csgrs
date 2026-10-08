import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha,json} from './point-qualified-sources.mjs';
sources();const b=json('point-qualified-platform-binaries.json');
assert.equal(sha('point-qualified-platform.rs'),b.harnessSha256);
const read=p=>readFileSync(p,'utf8'),files=[];
let harness=read('point-qualified-platform.rs');
for(const old of ['query(&a, &b, operation(op))','query(&a,&b,op)']){
 assert.equal(harness.split(old).length,2);
 const args=old.slice(6,-1);
 harness=harness.replace(old,'transform_algebraic_roots_binary('+args+',PredicatePolicy::APPROXIMATE_512)');
}
// No original source, binary, capture, constructor, wire or oracle is edited.
files.push(['point-qualified-approx.rs',harness]);
for(const variant of ['baseline','candidate'])files.push([
 'point-qualified-approx-'+variant+'/Cargo.toml',
 read('point-qualified-platform-'+variant+'/Cargo.toml').replaceAll('point-qualified-platform','point-qualified-approx')]);
files.push(['build-point-qualified-approx.mjs',read('build-point-qualified-platform.mjs')
 .replaceAll('point-qualified-platform','point-qualified-approx').replaceAll('point_qualified_platform','point_qualified_approx')]);
files.push(['run-point-qualified-approx-wasm.mjs',read('run-point-qualified-wasm.mjs')
 .replaceAll('point-qualified-platform-binaries','point-qualified-approx-binaries')]);
files.push(['run-point-qualified-approx.mjs',read('run-point-qualified-platform.mjs')
 .replaceAll('point-qualified-platform-binaries','point-qualified-approx-binaries')
 .replaceAll('point-qualified-public','point-qualified-approx-public')
 .replaceAll('run-point-qualified-wasm.mjs','run-point-qualified-approx-wasm.mjs')]);
console.log('*** Begin Patch\n'+files.map(([p,s])=>'*** Add File: '+resolve(p)+'\n'+s.trimEnd().split('\n').map(l=>'+'+l).join('\n')).join('\n')+'\n*** End Patch');
