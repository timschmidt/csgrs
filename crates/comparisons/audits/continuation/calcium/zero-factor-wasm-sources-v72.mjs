import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-cost-sources-v71.mjs';
export {sha,json};
export const files=['zero-factor-wasm-protocol-v72.md','zero-factor-wasm-v72.rs','zero-factor-wasm-input-v72.mjs','zero-factor-wasm-input-v72.json',
 'zero-factor-wasm-sources-v72.mjs','build-zero-factor-wasm-v72.mjs',
 'point-wasm-platform.rs','point-wasm-work.rs','point-history-work.rs','point-history-base.rs','power-sums-public.rs','zero-factor-cost-v71.rs',
 'zero-factor-cost-input-v71.json','zero-factor-cost-protocol-v71.mjs','check-zero-factor-v70.mjs','check-point-extended.mjs','point-extended-field.mjs',
 ...['baseline','candidate'].flatMap(v=>['rational','history'].map(k=>'zero-factor-wasm-'+v+'-'+k+'-v72/Cargo.toml'))];
export function wasmSources(){
 const prior=json('zero-factor-cost-v71-manifest.json');for(const[p,h]of Object.entries(prior.files))assert.equal(sha(p),h,p);
 const native=costSources();assert.deepEqual(native,json('zero-factor-native-origin-v71.json').source);
 const last=json('results/zero-factor-cost-verify-v71.json');assert.equal(last.code,0);assert.equal(last.signal,null);
 const cpu=readFileSync('zero-factor-cost-v71.rs','utf8'),wasm=readFileSync('zero-factor-wasm-v72.rs','utf8');
 assert.equal(cpu.slice(cpu.indexOf('    fn input_root('),cpu.indexOf('    fn wall(')).trimEnd(),
  wasm.slice(wasm.indexOf('    fn input_root('),wasm.indexOf('    struct Work {')).trimEnd());
 const history=readFileSync('point-history-work.rs','utf8');
 assert.equal(readFileSync('point-wasm-work.rs','utf8'),history.slice(history.indexOf('struct HistoryWork {'),history.indexOf('fn benchmark_main(')).trimEnd()+'\n');
 return {checkpoint:72,previousSha256:sha('zero-factor-cost-v71-manifest.json'),native,files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  scope:'Import-free rational/state collectors only; retained and isolated solver trees unchanged. No WASM timing or retention qualification follows from source identity.'};
}
