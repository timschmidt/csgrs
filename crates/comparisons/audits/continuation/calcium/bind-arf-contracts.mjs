import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkArfContracts} from './check-arf-contracts.mjs';
const tags=['compile','native','memcheck','linked-libraries','output-check'].map(t=>'arf-contract-'+t);
const files=['flint-arf-contract-controls.c','check-arf-contracts.mjs','record-arf-contract-reads.mjs',
 'arf-contract-read-records.json','record-arf-contract-support.mjs','arf-contract-support-read-records.json',
 'arf-contract-findings.md','bind-arf-contracts.mjs','verify-arf-contracts.mjs','e-qualified-experiment.json',
 'e-qualified-retention-verification.json','capture.mjs'];
const gates=tags.map(tag=>{const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
 return{tag,code:g.code,signal:g.signal,cwd:g.cwd,command:g.command,args:g.args};});
const live=json('e-qualified-experiment.json').liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const linked=readFileSync('results/arf-contract-linked-libraries.stdout','utf8'),previous=json('mag-series-experiment.json');
const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));
assert.deepEqual(libraries,previous.libraries);for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const hyperReadRanges={'hyperreal/src/real/arithmetic/representation.rs':[[315,441]],
 'hyperreal/src/computable/approximation/exp_sqrt.rs':[[121,265]],
 'hyperreal/src/real/arithmetic/elementary_functions.rs':[[312,405]],
 'hyperreal/src/rational/arithmetic/queries_conversion.rs':[[327,441]],
 'hyperreal/tests/public_numeric_semantics.rs':[[1,115]],'hyperreal/src/rational/arithmetic/tests.rs':[[2350,2415]]};
for(const[p,ranges]of Object.entries(hyperReadRanges)) {
 const s=readFileSync(resolve('../../../..',p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 assert(live[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
const binary='/tmp/calcium-arf-contracts.PNgYf4/controls',checks=checkArfContracts();
assert.deepEqual(checks,json('results/arf-contract-output-check.stdout'));
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,
 liveSources:live,libraries,configurationFiles:previous.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},
 readRecords:[...json('arf-contract-read-records.json'),...json('arf-contract-support-read-records.json')],
 newDonorLines:5849,arfReadLines:5783,supportReadLines:66,newFullFiles:47,
 completedExistingPaths:['src/arf.h','doc/source/arf.rst'],newPartialFiles:2,completeArfScopeFiles:104,
 coverageAtBinding:effectiveSummary(),hyperReadRanges,checks,
 status:'ARF implementation/tests/header/manual source-complete at the pinned inventory; recursive support and the full exact-real ecosystem remain incomplete. Bounded finite add/sub/div/root numerical and independent full-value checks pass.',
 production:'No production or donor change, new transfer, cleanup, deletion, commit, push or external report. All956 retained source hashes match checkpoint43; its five continuation improvements and all pre-existing user changes remain intact. One28392-byte executable reuses the existing native libraries; no new Rust or whole-native build.',
 findings:'The two main add/sub tests override all random modes with truncation. Signed-add wrapper aliasing is unreachable because n_randint(state,1) returns zero. First/third binary64 conversion switches have unreachable nearest-even defaults. Root references share MPFR and omit nearest-even. Approximate-dot reference error pairings use revx for both vectors even when revy differs. These are source-level qualification gaps, not newly proved library output defects. New finite exact-oracle checks cover518490 complete outputs/flags,316260 whole-object aliases and64518 input-preservation checks;64860 exact outputs and890 repeated ties. Independent BigInt regenerates40446 rows and202230 primary power certificates, checking all alias outputs in full.',
 comparison:'Hyper already separates finite rational facts from partial computable-real decisions, tests certified positive/negative ties-away rounding, provides total multivalued near-integer choice, uses guarded integer/Newton square roots and integer-power enclosures for bounded nth roots, and compresses dyadic conversion tails with sticky round-to-odd. Finite ARF nearest-even/ordering/root domains are not replacements for those contracts. No additional worthwhile architecture or performance transfer is established.',
 limits:checks.limits+' Source completeness is limited to the104-file ARF slice, not recursively called MPFR/GMP/integer support or all original references. Scalar conversion/comparison and approximate-dot oracle gaps remain separate. No full upstream ARF test executable is newly run. No new Hyper CPU/allocation/size benchmark is warranted without a production candidate; inherited checkpoint43 regression gates remain bound to unchanged current sources. Paired numerical logs total36592572 bytes in the workspace; /tmp receives only the28392-byte native executable for this checkpoint.',
 followup:'Qualify remaining finite conversion/integer-rounding/comparison and approximate-dot contracts with independent oracles where prior evidence is insufficient, finish recursive scalar/high-product helpers and generic field/matrix/algebraic support, then every original formal/symbolic/historical reference and full inventory reconciliation. Preserve all older numerical/FENV/LLL/thread-memory failures and open transfer experiments.'};
assert.equal(files.length,27);assert.equal(m.readRecords.length,51);assert.equal(m.binary.bytes,28392);
writeFileSync('arf-contract-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:files.length,gates:gates.length,liveFiles:Object.keys(live).length,
 readRecords:m.readRecords.length,newDonorLines:m.newDonorLines,coverage:m.coverageAtBinding,binaryBytes:m.binary.bytes,checks}));
