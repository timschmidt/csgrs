import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {sha} from './point-demand-sources.mjs';
import {demandBindings} from './point-demand-bindings.mjs';
import {lines} from './point-history-protocol.mjs';
import {checkPointImage} from './check-point-image.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
export function checkDemandPublic(){
 demandBindings();const publicChecks={};
 for(const mode of ['public','approx']){
  const p='results/point-demand-'+mode+'.stdout';
  const prior='results/point-qualified-'+(mode==='approx'?'approx-':'')+'public-candidate-native.stdout';
  assert.equal(sha(p),sha(prior),'complete eager/demand public '+mode);
  publicChecks[mode]=checkPointImage('results/power-sums-public-baseline.stdout',p);
 }
 const p='results/point-demand-extended.stdout';assert.equal(sha(p),sha('results/point-extended-public-candidate.stdout'));
 const rows=lines(p),checks={};assert.equal(rows.length,385);
 assert.deepEqual(rows.at(-1),{type:'terminal',cases:48,histories:4,policies:2,rows:384});
 let i=0;const statuses={};
 for(let which=0;which<48;which++)for(let policy=0;policy<2;policy++)for(let history=0;history<4;history++){
  const r=rows[i++];assert.equal(r.type,'query');assert.equal(r.case,which);assert.equal(r.policy,policy);assert.equal(r.history,history);
  const result=validateExtendedObservation(r);assert.deepEqual(result.failures,[]);
  for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
  statuses[r.report.status]=(statuses[r.report.status]??0)+1;
 }
 const totalChecks=Object.values(checks).reduce((a,b)=>a+b,0);assert.equal(totalChecks,11248);
 return{status:'pass',publicChecks,extended:{rows:384,checks,totalChecks,statuses},
  limits:'Complete output equality against the guarded variant plus independent rational/radical/serialized-value mathematical checks. Repeated policies and earlier corpora are not new independent defect counts. The large rational checker contains candidate self-pairs; separate baseline/demand comparisons establish the same 825 gains and 5616 unchanged records. No new WASM, full consumer or timing qualification implied.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkDemandPublic()));
