import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkFexprFormattingMetadata} from './check-fexpr-formatting-metadata.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const workspace=resolve('../../../..'),inv=json('inventory.json'),prior=effectiveCoverage();
const before=effectiveSummary(),records=[],donors={};
assert.deepEqual(before,[{repo:'calcium',reviewed:467,complete:466,partial:1,readLines:45368},
 {repo:'flint',reviewed:798,complete:778,partial:20,readLines:108749}]);
const notes={
 'fexpr_builtin.h':'Entire alphabetical symbol enum, table record layout, display callback pointer, inline name/length and packed arithmetic identifiers. Vocabulary is not implemented semantic support.',
 'fexpr_builtin.rst':'Entire builtin manual: binding forms, logic/equality/cases, exact and extended number vocabulary, declaration-only calculus/special functions, nonsemantic display markup and explicit normalization-cost warning.',
 'table.c':'Every474 table rows read, including display text and callback mapping. Lookup and presentation share one static metadata table; no measured binary-size claim.',
 'lookup.c':'Entire alphabetically indexed binary string search returning-1 for absent names. Immutable metadata, no retained mutable lookup cache.',
 'inlines.c':'Complete translation-unit inline-definition emission.',
 'write_latex.c':'Entire formatter: structural precedence/signs/powers, calculus/logic/collections/matrices, endpoint substitutions, decimal text, polynomial/root notation, explicit normal-form evaluation, symbol and generic dispatch, optional derivatives and stream entry points. Display is not proof/serialization; source ownership/text-output concerns unexecuted.',
 'calcium.h':'Complete archived global/version, stream lifecycle, three-valued truth, function-code enum and integer hash header. Previously read76..108 excluded from new credit.',
 'write.c':'File-backed formatting API versus geometrically growing literal string buffer; do not transplant as arbitrary-text-safe output. No runtime issue reproduction.',
 'write_acb.c':'Real/imaginary display and sign extraction, exact/NO_RADIUS distinctions. Archived compatibility text rewrite versus current direct Arb string API; no new numerical or formatting accuracy gate.',
 'write_si.c':'Machine-integer file formatting and22-byte stack temporary for string route, current WORD_FMT versus archived size dispatch.',
 'io.c':'Current file-stream initialization with opaque FILE pointer and null string owner.',
 'func_name.c':'Entire function-code/name switch and unknown fallback; separate from the474-entry expression vocabulary.',
 'test_multiplier.c':'Archived lazy environment test multiplier, accepted0..1000 range and default1 fallback. Source-read only; no tests newly executed.',
 'version.c':'Archived version string access; header patchlevel and version text differ, no runtime compatibility claim.',
 'fmpz_hash.c':'Current immediate/least-limb signed integer hash; no semantic or adversarial hash-performance claim.'
};
for(const s of inv.sources)for(const f of s.files){
 const selected=/^(src\/)?fexpr_builtin(?:\.h|\/)/.test(f.path)||f.path==='doc/source/fexpr_builtin.rst'||
  /^(src\/)?fexpr\/write_latex\.c$/.test(f.path)||/^(src\/)?calcium(?:\.h|\/)/.test(f.path);
 if(!selected)continue;
 const p=prior.find(r=>r.repo===s.repo&&r.path===f.path),read=p?.ranges.reduce((n,[a,b])=>n+b-a+1,0)??0;
 if(read===f.lines)continue;
 const note=notes[f.path.split('/').at(-1)];assert(note,f.path);assert(f.text);
 const ranges=p?[[1,75],[109,183]]:[[1,f.lines]];
 if(p){assert.equal(s.repo,'calcium');assert.equal(f.path,'calcium.h');assert.deepEqual(p.ranges,[[76,108]]);}
 assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,f.path)),f.sha256,f.path);
 donors[s.repo+':'+f.path]=f.sha256;
 records.push({repo:s.repo,path:f.path,ranges,note:'Checkpoint49: '+note});
}
assert.equal(records.length,25);
const lines=records.reduce((n,r)=>n+r.ranges.reduce((n,[a,b])=>n+b-a+1,0),0);assert.equal(lines,13667);
const previous=json('fexpr-representation-manifest.json');
for(const[p,h]of Object.entries(previous.files))assert.equal(sha(p),h,p);
const live=json('mpoly-bridge-experiment.json');
for(const[p,h]of Object.entries(live.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
const checks=checkFexprFormattingMetadata();
const auxiliaryReadRanges={
 'exact-real-references/flint/src/config.h':[[201,227]],
 'exact-real-references/flint/src/config.h.in':[[200,226]]};
const auxiliarySources=Object.fromEntries(Object.keys(auxiliaryReadRanges).map(p=>[p,sha(resolve(workspace,p))]));
const hyperReadRanges={'hyperreal/src/computable/format.rs':[[1,330]],
 'hyperreal/src/real/arithmetic/format_parse.rs':[[1,106]]};
for(const p of Object.keys(hyperReadRanges))assert(live.liveSources[p],p);
const files=['record-fexpr-formatting-reads.mjs','verify-fexpr-formatting.mjs',
 'check-fexpr-formatting-metadata.mjs','fexpr-formatting-findings.md',
 'fexpr-formatting-read-records.json','fexpr-representation-manifest.json','inventory.json','effective-coverage.mjs',
 ...['json','stdout','stderr'].map(ext=>'results/fexpr-representation-verify.'+ext)];
for(const p of files.filter(p=>p!=='fexpr-formatting-read-records.json'))sha(p);
const terminal=json('results/fexpr-representation-verify.json');assert.equal(terminal.code,0);assert.equal(terminal.signal,null);
assert.equal(readFileSync('results/fexpr-representation-verify.stderr').length,0);
writeFileSync('fexpr-formatting-read-records.json',JSON.stringify(records,null,2)+'\n',{flag:'wx'});
writeFileSync('coverage-extensions.json',JSON.stringify([...json('coverage-extensions.json'),...records],null,2)+'\n');
const m={schema:1,checkpoint:49,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
 donorSources:donors,readRecords:records,newDonorLines:lines,newFiles:24,completedPartialFiles:1,
 coverageBefore:before,coverageAtBinding:effectiveSummary(),auxiliarySources,auxiliaryReadRanges,hyperReadRanges,
 previousManifest:'fexpr-representation-manifest.json',previousLiveFiles:956,checks,
 qualification:'Static metadata and source/recorded-evidence integrity only. No new donor runtime, numerical, Memcheck, benchmark or full47 historical chain rerun.',
 production:'No production/donor edit, new retained transfer, /tmp build or file, cleanup/deletion, commit, push or external report.',
 scope:'Both fexpr and fexpr_builtin directories, their headers/manuals and both calcium support directories/headers source-read. Recursive support, algebraic paths and full original ecosystem remain incomplete.'};
writeFileSync('fexpr-formatting-manifest.json',JSON.stringify(m,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:49,files:files.length,readRecords:records.length,newDonorLines:lines,
 coverage:m.coverageAtBinding,checks,qualification:m.qualification}));
