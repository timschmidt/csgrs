import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sha} from './point-qualified-sources.mjs';
const variant=process.argv[2];assert(['baseline','candidate'].includes(variant));
const f=json('point-qualified-approx-binaries.json').artifacts.find(f=>f.variant===variant&&f.platform==='wasm');
assert.equal(sha(f.path),f.sha256);
const module=await WebAssembly.compile(readFileSync(f.path));
assert.deepEqual(WebAssembly.Module.imports(module),[]);
const {exports:e}=await WebAssembly.instantiate(module);
assert(e.memory instanceof WebAssembly.Memory);
assert.equal(typeof e.point_collect,'function');assert.equal(typeof e.point_output_ptr,'function');
const length=e.point_collect(),pointer=e.point_output_ptr();
assert(length>4000000&&length<5000000);assert(pointer>0&&pointer+length<=e.memory.buffer.byteLength);
const output=Buffer.from(new Uint8Array(e.memory.buffer,pointer,length));
const rows=output.toString('utf8').trim().split('\n');assert.equal(rows.length,6441);
assert.deepEqual(JSON.parse(rows.at(-1)),{type:'terminal',publicRows:6400,costCases:40});
process.stderr.write(JSON.stringify({variant,node:process.version,v8:process.versions.v8,
 imports:WebAssembly.Module.imports(module),exports:WebAssembly.Module.exports(module),
 moduleBytes:readFileSync(f.path).length,outputBytes:length,memoryBytesAfterCollection:e.memory.buffer.byteLength,
 limits:'One fresh instance, full-value corpus execution. Linear memory after JSON collection is not peak per-query demand or a CPU measurement.'})+'\n');
process.stdout.write(output);
