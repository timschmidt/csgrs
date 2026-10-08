import assert from 'node:assert/strict';
import {finalZeroSources,sha,json} from './zero-factor-final-sources-v70.mjs';
export {sha,json};
export const harnessFiles=['zero-factor-cost-protocol-v71.md','zero-factor-cost-input-v71.mjs','zero-factor-cost-input-v71.json',
 'zero-factor-cost-v71.rs','zero-factor-cpu-v71.rs','zero-factor-allocation-v71.rs','power-sums-public.rs','power-sums-allocation.rs',
 ...['baseline','candidate'].map(v=>'zero-factor-cost-'+v+'-app-v71/Cargo.toml'),
 'zero-factor-cost-sources-v71.mjs','zero-factor-cost-protocol-v71.mjs','build-zero-factor-cost-v71.mjs',
 'point-qualified-capture.mjs','capture.mjs','paired-statistics-v60.mjs'];
export function costSources(){
 const previous=json('zero-factor-v70-manifest.json');for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
 const source=finalZeroSources();assert.equal(sha('zero-factor-final-binding-v70.json'),previous.evidence.sourceBindingSha256);
 const final=json('results/zero-factor-verify-v70.json');assert.equal(final.code,0);assert.equal(final.signal,null);
 return {checkpoint:71,solver:source,previousSha256:sha('zero-factor-v70-manifest.json'),
  files:Object.fromEntries(harnessFiles.map(p=>[p,sha(p)]))};
}
