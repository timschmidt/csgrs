import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {checkPointImage} from './check-point-image.mjs';
import {sha,json} from './point-qualified-sources.mjs';
export function checkApproxPublic(){
 const checks={};
 for(const platform of ['native','wasm']){
  const paths=['baseline','candidate'].map(v=>'results/point-qualified-approx-public-'+v+'-'+platform+'.stdout');
  const c=checkPointImage(...paths);
  checks[platform]={status:c.status,improved:c.improved,unchanged:c.unchanged,
   counts:c.mathematical.counts,checks:c.mathematical.checks,totalChecks:c.mathematical.totalChecks,
   independentResultants:c.mathematical.independentResultants,failures:c.mathematical.failures};
  for(const variant of ['baseline','candidate']){
   const here='results/point-qualified-approx-public-'+variant+'-'+platform+'.stdout';
   const strict='results/point-qualified-public-'+variant+'-'+platform+'.stdout';
   assert.equal(sha(here),sha(strict),variant+' '+platform+' policy equality');
  }
 }
 for(const variant of ['baseline','candidate']){
  const prefix='results/point-qualified-approx-public-'+variant+'-wasm';
  const m=JSON.parse(readFileSync(prefix+'.stderr','utf8'));
  const f=json('point-qualified-approx-binaries.json').artifacts.find(f=>f.variant===variant&&f.platform==='wasm');
  assert.equal(m.variant,variant);assert.deepEqual(m.imports,[]);
  assert.equal(m.outputBytes,readFileSync(prefix+'.stdout').length);assert.equal(m.moduleBytes,f.bytes);
 }
 return{status:'pass',requestedPolicy:'APPROXIMATE_512',rowsPerVariantPerPlatform:6441,checks,
  limits:'Same authored exact-rational carriers and endpoints as the STRICT corpus, but both public call sites request APPROXIMATE_512. Exact rational comparisons remain available. This exercises the guarded approximate-policy point-image replay on both platforms; it is not unresolved or nonrational endpoint qualification. Candidate self-pair oracle checks are not independent baseline comparisons; separate paired comparisons prove 825 gains and 5,616 unchanged records. Repeated identical outputs are not new independent mathematical oracles or timing observations.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkApproxPublic()));
