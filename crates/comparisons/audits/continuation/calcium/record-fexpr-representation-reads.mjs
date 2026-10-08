import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const workspace=resolve('../../../..'),inv=json('inventory.json'),prior=effectiveCoverage();
const before=effectiveSummary(),records=[],donors={};
assert.deepEqual(before,[{repo:'calcium',reviewed:422,complete:421,partial:1,readLines:41035},
 {repo:'flint',reviewed:756,complete:735,partial:21,readLines:104761}]);
const notes={
 'fexpr.h':'Entire public representation and inline operations: flat tagged words, owned capacity, borrowed views, indexed general calls, builtin alias wrappers, vector lifecycle and API declarations. No DAG sharing or numerical approximation cache.',
 'fexpr.rst':'Entire manual: formal syntax and structural equality, ownership/views/alias restrictions, compact representations, numeric conversion, printing, formal normalization and vector contracts. Source/documentation distinctions are in findings; existing current490..585 credit excluded.',
 'arg.c':'Owned argument copy and in-bounds view lookup using small-call skips or every-four-argument index; no newly exercised invalid indexes or overlap.',
 'func.c':'Owned function extraction and borrowed function view for small/general call layouts; fast builtin recognition.',
 'call0.c':'Zero-argument generic call concatenates header and copied function with output distinct from inputs.',
 'call1.c':'One-argument call layout and builtin unary wrapper with temporary/swap for whole-output alias.',
 'call2.c':'Two-argument call layout and builtin binary wrapper with temporary/swap for whole-output alias.',
 'call3.c':'Three-argument flat constructor copies all children; generic nonoverlap precondition.',
 'call4.c':'Four-argument flat constructor copies all children; generic nonoverlap precondition.',
 'call_vec.c':'Dispatch to small constructors or generic argument-count/function-offset/every-four-argument index layout; all subexpressions copied.',
 'arithmetic.c':'Pos/Neg/Add/Sub/Mul/Div/Pow convenience constructors use builtin alias-supporting wrappers, without semantic normalization.',
 'cmp_fast.c':'Representation-first total ordering over encoded expressions, not mathematical real comparison.',
 'hash.c':'Complete structural word-array hash; not semantic equality or a studied adversarial/hash-performance claim.',
 'contains.c':'Structural whole/subexpression search, including function heads and equality/size shortcuts.',
 'depth.c':'Recursive maximum expression depth including function head and arguments; atoms terminate.',
 'num_leaves.c':'Recursive atom count including function heads, independent of semantic arithmetic complexity.',
 'equal_si.c':'Compact signed-integer representation comparison, not approximate numerical equality.',
 'equal_ui.c':'Compact unsigned-integer representation comparison, not approximate numerical equality.',
 'is_neg_integer.c':'Tag/sign structural recognition for small and multiword integer atoms.',
 'is_builtin_call.c':'Fast structural builtin-head comparison across call layouts; not arithmetic arity/domain validation.',
 'is_any_builtin_call.c':'Recognizes any builtin head in call layouts, distinct from evaluation support.',
 'inlines.c':'Translation-unit emission of inline definitions.',
 'vec_sort_fast.c':'qsort wrapper over vector entries using structural fast ordering.',
 'set_si.c':'Compact signed integer construction and multiword fallback; source generation differences only, no old signed-boundary reproduction.',
 'set_ui.c':'Compact unsigned integer construction and multiword fallback.',
 'set_fmpz.c':'Exact immediate/multiword integer encoding, fitting storage then copying magnitude limbs.',
 'get_fmpz.c':'Exact structural integer decoding and failure status for nonintegers; manual failure wording differs. No invalid-input probe.',
 'set_fmpq.c':'Canonical rational constructor: unit denominator integer route, small direct Div encoding or owned numerator/denominator construction.',
 'set_arf.c':'Exact mantissa/exponent constructor, compact integer/rational exponent ranges and general power-of-two structure; symbolic nonfinite atoms.',
 'set_d.c':'Exact binary64-to-ARF construction and explicit complex imaginary-unit syntax; no decimal-intent inference.',
 'set_symbol_str.c':'Builtin lookup and compact/long symbol storage for documented valid names; no new malformed-name tests.',
 'get_symbol_str.c':'Builtin/small/long symbol extraction into caller-freed text, distinct from semantic evaluation.',
 'set_string.c':'Compact or padded multiword string atom construction.',
 'get_string.c':'String atom extraction into owned text.',
 'replace.c':'Simultaneous structural replacement, first-rule priority, function-head recursion, borrowed unchanged branches and temporary changed owners; documented whole-expression alias route.',
 'print.c':'Structural plain printing and vector printing, integer/symbol/string/call routes; not general escaped serialization.',
 'numerical_enclosure.c':'Entire fixed-precision Acb interpreter and finite capped decimal retry. Evaluation success and requested accuracy are distinct; output suppresses radius. All special-function dispatch read, not newly numerically qualified.',
 'arithmetic_nodes.c':'Archived recursive formal-leaf collection, arithmetic/integer-power traversal and structural unique insertion; current counterpart already47.',
 'expanded_normal_form.c':'Archived rational-function normalization pipeline and preserve-original failure fallback; current counterpart already47. Does not prove leaf independence or domains.',
 'is_arithmetic_operation.c':'Archived structural arithmetic-head recognition, distinct from arity and domain checks; current counterpart already47.',
 'set_fmpz_mpoly.c':'Archived expanded polynomial-to-expression conversion, term factor lists and exact constants; current counterpart already47.',
 'main.c':'Current fexpr test registration: five included tests and common driver. Read only; no suite execution this checkpoint.',
 't-builtins.c':'All builtin index/order/lookup consistency tests and named absence controls; source-read only.',
 't-call_vec.c':'Generated0..19-argument calls, exact integer children and owned/borrowed function/argument comparisons; shared constructors, source-read only.',
 't-set_fmpz.c':'Repeated bounded generated integer roundtrips through set/get, sharing integer backend; source-read only.',
 't-replace.c':'Entire generated expression/rule test and naive recursive reference, contains and documented in-place replacement checks. Shared structural backend, source-read only.',
 't-write_latex.c':'Arbitrary-syntax generator and string-return smoke test, not mathematical/display correctness; source-read only, not run.'
};
for(const s of inv.sources)for(const f of s.files){
 const selected=f.path==='doc/source/fexpr.rst'||/^(src\/)?fexpr\.h$/.test(f.path)||
  (/^(src\/)?fexpr\//.test(f.path)&&!f.path.endsWith('/write_latex.c'));
 if(!selected)continue;
 const p=prior.find(r=>r.repo===s.repo&&r.path===f.path);
 const read=p?.ranges.reduce((n,[a,b])=>n+b-a+1,0)??0;
 if(read===f.lines)continue;
 const note=notes[f.path.split('/').at(-1)];assert(note,f.path);
 const ranges=p?[[1,489],[586,620]]:[[1,f.lines]];
 if(p){assert.equal(s.repo,'flint');assert.equal(f.path,'doc/source/fexpr.rst');assert.deepEqual(p.ranges,[[490,585]]);}
 assert(f.text);const path=resolve(workspace,'exact-real-references',s.repo,f.path);
 assert.equal(sha(path),f.sha256,path);donors[s.repo+':'+f.path]=f.sha256;
 records.push({repo:s.repo,path:f.path,ranges,note:'Checkpoint48: '+note});
}
const lines=records.reduce((n,r)=>n+r.ranges.reduce((n,[a,b])=>n+b-a+1,0),0);
assert.equal(records.length,88);assert.equal(lines,8321);
const previous=json('mpoly-bridge-experiment.json');
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
for(const[p,h]of Object.entries(previous.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const[p,h]of Object.entries({...previous.libraries,...previous.configurationFiles}))assert.equal(sha(p),h,p);
for(const b of previous.binaries)assert.equal(sha(b.path),b.sha256,b.path);
const terminal=json('results/mpoly-bridge-verify-full.json');assert.equal(terminal.code,0);assert.equal(terminal.signal,null);
assert.equal(readFileSync('results/mpoly-bridge-verify-full.stderr').length,0);
const hyperReadRanges={'hyperreal/src/computable/node/representation.rs':[[1,330]],
 'hyperreal/src/computable/README.md':[[1,170]]};
for(const p of Object.keys(hyperReadRanges))assert(previous.liveSources[p],p);
const files=['record-fexpr-representation-reads.mjs','verify-fexpr-representation.mjs',
 'fexpr-representation-findings.md','fexpr-representation-read-records.json','inventory.json',
 'effective-coverage.mjs','mpoly-bridge-experiment.json',
 ...['json','stdout','stderr'].map(ext=>'results/mpoly-bridge-verify-full.'+ext)];
// Preflight every input before generating any new evidence or appending coverage.
for(const p of files.filter(p=>p!=='fexpr-representation-read-records.json'))sha(p);
writeFileSync('fexpr-representation-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
const m={schema:1,checkpoint:48,recorded:new Date().toISOString(),
 files:Object.fromEntries(files.map(p=>[p,sha(p)])),donorSources:donors,
 readRecords:records,newDonorLines:lines,newFiles:87,completedPartialFiles:1,
 coverageBefore:before,coverageAtBinding:effectiveSummary(),hyperReadRanges,
 previousManifest:'mpoly-bridge-experiment.json',previousLiveFiles:956,
 qualification:'Source and recorded-evidence integrity only. No new numerical tests, Memcheck, benchmark or complete historical chain rerun.',
 production:'No production/donor edit, new retained transfer, /tmp allocation, build, cleanup/deletion, commit, push or external report.',
 scope:'Both fexpr headers/manuals and both directories except write_latex.c are source-read. Builtin headers/tables, LaTeX implementations, recursive support and full original ecosystem remain incomplete.'};
writeFileSync('fexpr-representation-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:48,files:files.length,readRecords:records.length,newDonorLines:lines,
 coverage:m.coverageAtBinding,qualification:m.qualification}));
