import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {checkPointImage} from './check-point-image.mjs';
import {json,sha} from './point-qualified-sources.mjs';
export function checkPublic(){
 const checks={};
 for(const platform of ['native','wasm']){
  const paths=['baseline','candidate'].map(v=>'results/point-qualified-public-'+v+'-'+platform+'.stdout');
  checks[platform]=checkPointImage(...paths);
 }
 for(const variant of ['baseline','candidate']){
  const native='results/point-qualified-public-'+variant+'-native.stdout';
  const wasm='results/point-qualified-public-'+variant+'-wasm.stdout';
  assert.equal(sha(native),sha(wasm),variant+' full-value cross-platform equality');
  const original=variant==='baseline'?'results/power-sums-public-baseline.stdout':'results/point-image-guard-public.stdout';
  assert.equal(sha(native),sha(original),variant+' checkpoint-52/53 collector equality');
  const metadata=JSON.parse(readFileSync(wasm.replace(/stdout$/,'stderr'),'utf8'));
  assert.equal(metadata.variant,variant);assert.deepEqual(metadata.imports,[]);
  assert.equal(metadata.outputBytes,readFileSync(wasm).length);
  assert(metadata.memoryBytesAfterCollection>=metadata.outputBytes);
  const artifact=json('point-qualified-platform-binaries.json').artifacts.find(f=>f.variant===variant&&f.platform==='wasm');
  assert.equal(metadata.moduleBytes,artifact.bytes);
 }
 return{status:'pass',rowsPerVariantPerPlatform:6441,checks,
  limits:'Two executions of the unchanged mathematical oracle. Its candidate self-pair checks are disclosed by checkpoint 53; separate baseline/candidate full-record checks prove exactly 825 gains and 5,616 unchanged records on each platform. Identical outputs are not additional independent mathematical oracles. No platform timings or nonrational endpoint coverage are claimed.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkPublic()));
