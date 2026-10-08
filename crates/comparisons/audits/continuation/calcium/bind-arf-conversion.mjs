import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkArfConversion} from './check-arf-conversion.mjs';
const tags=['arf-conversion-compile','arf-conversion-native','arf-conversion-linked-libraries',
 'arf-conversion-v2-compile','arf-conversion-v2-native','arf-conversion-v2-linked-libraries',
 'arf-conversion-v2-memcheck','arf-conversion-output-check'];
const files=['flint-arf-conversion-controls.c','flint-arf-conversion-v2-controls.c',
 'prepare-arf-conversion-v2.mjs','check-arf-conversion.mjs','record-arf-conversion-reads.mjs',
 'arf-conversion-read-records.json','arf-conversion-findings.md','bind-arf-conversion.mjs',
 'verify-arf-conversion.mjs','arf-contract-experiment.json','capture.mjs',
 ...['json','stdout','stderr'].map(ext=>'results/arf-contract-verify-full.'+ext)];
const gates=tags.map(tag=>{const g=json('results/'+tag+'.json');
 assert.equal(g.code,tag==='arf-conversion-native'?1:0);assert.equal(g.signal,null);
 for(const ext of ['json','stdout','stderr'])files.push('results/'+tag+'.'+ext);
 return{tag,code:g.code,signal:g.signal,cwd:g.cwd,command:g.command,args:g.args};});
const previous=json('arf-contract-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
for(const tag of ['arf-conversion','arf-conversion-v2']) {
 const linked=readFileSync('results/'+tag+'-linked-libraries.stdout','utf8');
 const libraries=Object.fromEntries([...linked.matchAll(/=> (\/[^ ]+) \(/g)].map(m=>[m[1],sha(m[1])]));
 assert.deepEqual(libraries,previous.libraries);
}
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const binaries=['controls','controls-v2'].map(name=>{const path='/tmp/calcium-arf-conversion.ZnmLMf/'+name;
 return{path,bytes:statSync(path).size,sha256:sha(path)};});
assert.deepEqual(binaries.map(b=>b.bytes),[28664,28672]);
const checks=checkArfConversion();assert.deepEqual(checks,json('results/arf-conversion-output-check.stdout'));
const m={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,
 liveSources:live,libraries:previous.libraries,configurationFiles:previous.configurationFiles,binaries,
 readRecords:json('arf-conversion-read-records.json'),newDonorLines:238,newCompleteFiles:1,
 coverageAtBinding:effectiveSummary(),
 hyperReadRanges:{'hyperreal/src/rational/arithmetic/queries_conversion.rs':[[320,445]],
  'hyperreal/tests/public_numeric_semantics.rs':[[1,115]]},checks,
 status:'Finite conversion/integer numerical qualification passes after one preserved harness contract correction. One238-line support source is newly complete; the full exact-real ecosystem remains incomplete.',
 production:'No production or donor edits, new transfer, cleanup, deletion, commit, push or external report. All956 live source hashes match retained checkpoint43; all five continuation transfers remain intact. Reused native libraries; no new Rust/whole-native build.',
 findings:'Independent exact GMP grid and BigInt bit-order search/certificate oracles pass117705 assertions:12005 binary64 outputs including2401 nearest-even,11281 finite imports,12005 integer outputs,7985 proved-in-range signed conversions,14406 integral-operation outputs and9604 public aliases. Binary results include1848 subnormals,460 negative zeros,724 overflow infinities and164 midpoint fixtures. Decomposition, integrality, magnitude bounds, signed/absolute power/previous-value comparisons and input preservation also pass.',
 preservedFailure:'Original harness exit1 with exactly one bad zero-bound expectation; source/manual specify -ARF_PREC_EXACT for signed bound, and leave zero fmpz bounds unspecified. Original source/binary/gate retained. Mechanically derived v2 skips the two unspecified zero calls, checks the documented sentinel and preserves its exact decimal text. Every non-bound numerical output equals v1; no donor bug inferred.',
 comparison:'Hyper already separates exact dyadic export from lossy/fallback conversion, has sticky round-to-odd normal compression, certified ties-away integer rounding, total multivalued near-integer choice and partial computable-real decisions. Donor finite nearest-even and total-order contracts are not replacements; no nonredundant worthwhile production candidate or new performance claim.',
 limits:checks.limits+' Source completeness is limited to the existing104-file ARF scope and newly read fmpz/set.c, not recursive support. Memory includes oracle/setup and is not peak/RSS or donor-only cost. Three numerical logs total8340310 workspace bytes; both preserved executables total57336 /tmp bytes. Inherited checkpoint43 regression gates remain bound to unchanged current sources.',
 followup:'Qualify add_si public alias and approximate-dot reference pairing with independent bounded oracles as warranted, fixed-grid wrappers and other remaining scalar contracts; continue recursive scalar/high-product, generic field/matrix/algebraic and all original formal/symbolic/historical references, unresolved transfers and full inventory reconciliation. Preserve older failed numerical/FENV/LLL/thread-memory evidence.'};
assert.equal(files.length,38);assert.equal(Object.keys(live).length,956);
writeFileSync('arf-conversion-experiment.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({files:files.length,gates:gates.length,successfulGates:7,preservedFailedGates:1,
 liveFiles:956,newDonorLines:238,coverage:m.coverageAtBinding,binaryBytes:57336,checks}));
