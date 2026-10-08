import assert from 'node:assert/strict';
import {wasmSources,sha,json} from './zero-factor-wasm-sources-v72.mjs';
export {sha,json};
export const sourceFiles=['zero-factor-wasm-cost-protocol-v73.md','zero-factor-wasm-cost-protocol-v73.mjs',
 'zero-factor-wasm-cost-worker-v73.mjs','zero-factor-wasm-cost-sources-v73.mjs',
 'run-zero-factor-wasm-cost-v73.mjs','zero-factor-wasm-cost-environment-v73.mjs','paired-statistics-v60.mjs'];
export function costSources(){
 const previous=json('zero-factor-wasm-v72-manifest.json');for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 const source=wasmSources();assert.deepEqual(source,json('zero-factor-wasm-origin-v72.json').source);
 const done=json('results/zero-factor-wasm-verify-v72.json');assert.equal(done.code,0);assert.equal(done.signal,null);
 return {checkpoint:73,previousSha256:sha('zero-factor-wasm-v72-manifest.json'),source,
  files:Object.fromEntries(sourceFiles.map(p=>[p,sha(p)])),scope:'Matched costs of unchanged checkpoint-72 rational modules. No source copy/build/live edit.'};
}
