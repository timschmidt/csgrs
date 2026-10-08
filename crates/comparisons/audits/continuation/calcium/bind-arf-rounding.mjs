import {readFileSync,writeFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkArfRoundingControls} from './check-arf-rounding-controls.mjs';
const tags=['compile','native','memcheck','linked-libraries','bigint-oracle','output-check','inventory'].map(t=>'arf-rounding-'+t);
const files=['flint-arf-rounding-controls.c','arf-rounding-bigint-oracle.mjs','check-arf-rounding-controls.mjs',
 'record-arf-rounding-reads.mjs','arf-rounding-read-selection.json','bind-arf-rounding.mjs',
 'verify-arf-rounding.mjs','high-product-experiment.json','capture.mjs'];
for(const tag of tags){const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 for(const e of ['json','stdout','stderr'])files.push('results/'+tag+'.'+e);}
const previous=json('high-product-experiment.json'),live=previous.liveSources;
for(const[p,h]of Object.entries(live))assert.equal(sha(resolve('../../../..',p)),h,p);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
const readRecords=json('arf-rounding-read-selection.json').map(s=>{
 const r=coverage.find(r=>r.repo===s.repo&&r.path===s.path&&JSON.stringify(r.ranges)===JSON.stringify(s.newRanges));assert(r);return r;
});
const libraryPaths=[...readFileSync('results/arf-rounding-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]);
const libraries=Object.fromEntries(libraryPaths.map(p=>[p,sha(p)]));assert.deepEqual(libraries,previous.libraries);
for(const[p,h]of Object.entries(previous.configurationFiles))assert.equal(sha(p),h,p);
const binary='/tmp/calcium-arf-rounding.9JsRt1/arf-rounding-controls';
const manifest={schema:1,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 gates:tags.map(tag=>({tag,code:0,signal:null})),liveSources:live,libraries,configurationFiles:previous.configurationFiles,
 binary:{path:binary,bytes:statSync(binary).size,sha256:sha(binary)},readRecords,coverageAtBinding:effectiveSummary(),
 hyperReadRanges:{'hyperreal/AGENTS.md':[[1,14]],'hyperreal/src/computable/approximation/arithmetic_kernels.rs':[[104,240]],
  'hyperlattice/src/complex.rs':[[1,195],[235,360]],'hyperlattice/src/kernels.rs':[[110,255]],'hyperlattice/src/vector.rs':[[240,280]]},
 checks:checkArfRoundingControls(),
 status:'Correctly-rounded finite ARF scalar controls pass native and Memcheck, independently checked by GMP integer quotient/remainder with raw result decoding. All 7200 group fingerprints and rounding decision counts also match independent JavaScript BigInt reconstruction. Earlier high-product bound discrepancies remain preserved, not disproved by ARF success.',
 production:'No new production/donor edits or fifth continuation transfer. All 955 live source/support hashes match checkpoint 33. Existing builds and earlier evidence are preserved.',
 limits:'612000 outputs: 18 limb-length pairs through 1001 limbs, ten deterministic operand patterns, four sign forms, 17 precision positions (including duplicate values where positions coincide), five modes and ten routes. There are 61200 outputs per route; this counts related/repeated arithmetic, not 612000 distinct mathematical cases. Inputs include finite zero, powers of two, all ones, sparse/tie patterns and deterministic dense words, with small ordinary exponents. Routes are set/neg rounding with disjoint and supported in-place outputs, public/swapped/in-place multiply, explicit MPFR multiply, public square and MPFR square. All values and exactness flags are compared with full GMP integers; the BigInt cross-check reconstructs 244800 arithmetic roundings and compact fingerprints, not collision-free certificates. Donor and oracle share GMP integer machinery, but not ARF rounding; MPFR is not the oracle. No infinity/NaN sweep, big-fmpz exponent, concurrent/TLS lifecycle, low-level ui/uiui direct API, fused/complex, ARM/32-bit/FFT or every-branch qualification. All public output seeding/inputs are valid. Memcheck allocation totals include setup/oracles, not donor-only cost or peak RSS. No Hyper implementation candidate means no matched CPU/allocation/binary benchmark is claimed.',
 findings:'ARF full-product rounding retains sticky/tie/carry information; the demand-sensitive MPFR bridge preserves this finite rounding contract. Complex/fused source retains exact intermediate products and delays final rounding, with balanced-size three-product crossovers. Hyper already plans constructive approximations from magnitude facts, uses exact BigInt products before final scaling and shares square work. Hyperlattice already has cache-evidence-gated exact-rational three-product complex multiplication, fused cold exact-rational complex products, sparse signed product-sums and known-exact shared-scale dot dispatch. Donor limb thresholds are not a new general algorithm and do not justify displacing these existing choices. Constructive absolute-error approximations are not the same API as correctly-rounded finite ARF values.',
 followup:'Qualify finite fused/complex cancellation cases with independent exact integer/rational arithmetic as needed, read the remaining ARF/addition/scratch/generic matrix/field support and original references. Investigate demand-sized multiplication only with certified omitted-tail/error handling and matched Hyper workloads. Reconcile all original inventory before completion; no scope narrowing.'};
writeFileSync('arf-rounding-experiment.json',JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'specialised high products and ARF rounding binding',files:files.length,gates:tags.length,
 liveFiles:Object.keys(live).length,readRecords:readRecords.length,readLines:7570,coverage:manifest.coverageAtBinding,
 binaryBytes:manifest.binary.bytes,summary:manifest.checks.summary,status:manifest.status}));
