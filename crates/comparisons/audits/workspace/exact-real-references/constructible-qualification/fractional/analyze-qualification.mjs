import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
const root=import.meta.dirname,workspace=resolve(root,'../../..');
const assert=(ok,message)=>{if(!ok)throw Error(message);};
const read=path=>readFileSync(path,'utf8');
const hash=path=>createHash('sha256').update(readFileSync(path)).digest('hex');
for(const [path,want] of Object.entries({
 'hyperreal/src/computable/node/structural_analysis.rs':'930e28d3400a59a6764402261f09e1e3c687c1f9b2166fbad8161b97c565163e',
 'hyperreal/src/computable/node.rs':'81578c53491c2c75cf445d8c7a9394ff8928a49bcbcfb069a02e4cd84b0bab5b',
 'hyperreal/src/computable/node/fractional_separation_tests.rs':'a6aafd580f1afd4de6bd5e26a7f741a5bda9f3148cca61f7254c00737e883e26',
 'hypercurve/src/bezier_offset.rs':'f37d3795ad1bead2de3e4bf7c20fb3d6fee60526c961c3d7978c1768b406af9f',
 'hypercurve/src/bezier_region.rs':'81f75094cf225e904f53463d649ecf87971a7c97d2eba3bf25ef61406e0c7ee4',
 'hypercurve/src/policy.rs':'6f9ede2c7e288e38b0fff1ff56e3d1fcf32e16fa980ba39e682480c8046f3d56',
 'hyperreal/benchmarks.md':'b99ad5623ac1b50a7c8c3a1b00627c08b5f1b0643499b93a479ec39ab1b52b12',
 'hyperreal/dispatch_trace.md':'ef7c18ed21b815a3077a2a01d31e4d661fef6dd01069e42d23d107cad3a29c3c',
 '.audit-fractional-control.pXHyjP/hyperreal/src/computable/node/structural_analysis.rs':'92d2e3c7fb956d3ee81a2f33243b1934e40440f04ed73e34254fef887ed81346',
 '.audit-constructible-build.l4UDoe/fractional-field-retained-debug':'cc56da89151781bcb2f1464197c2fde1e66acbe4082491cf5bc73746d38067d9',
 '.audit-constructible-build.l4UDoe/fractional-field-retained-release':'629aae20d1f530bf2c79c0e6336f2a8101ff5201e8b61bb3f8f139effc42182a',
}))assert(hash(workspace+'/'+path)===want,'changed qualification snapshot: '+path);
const tests={};
for(const [file,expected,ignored] of [
 ['retained-full-debug.log',750,0],['retained-full-release-all-features.log',857,0],
 ['hyperlattice-current.log',202,0],['hyperlimit-current.log',348,0],['hypertri-current.log',7,0],['hypersolve-current.log',796,0],['hypercurve-current.log',902,1],
]){
 const text=read(root+'/'+file),summaries=[...text.matchAll(/test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored/g)];
 assert(summaries.length>0&&!/^error:/m.test(text),'missing/failed suite '+file);
 assert(summaries.every(m=>m[1]==='ok'&&+m[3]===0),'test failures '+file);
 const passed=summaries.reduce((n,m)=>n+Number(m[2]),0),skipped=summaries.reduce((n,m)=>n+Number(m[4]),0);
 assert(passed===expected&&skipped===ignored,'test count '+file);tests[file]={passed,ignored:skipped};
}
const all=read(root+'/retained-all-targets-all-features.log'),summary=[...all.matchAll(/test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored/g)];
assert(summary.length>0&&summary.every(m=>m[1]==='ok'&&+m[3]===0)&&!/^error:/m.test(all),'all-target suite failed');
const declared=[...read(workspace+'/hyperreal/Cargo.toml').matchAll(/\[\[bench\]\]\s+name = "([^"]+)"/g)].map(m=>m[1]).sort();
const ran=[...all.matchAll(/Running benches\/([^ .]+)\.rs /g)].map(m=>m[1]).sort();
assert(JSON.stringify(declared)===JSON.stringify(ran),'missing benchmark executable');
const attempted=all.split('\n').filter(s=>s.startsWith('Testing ')).length,successes=all.split('\n').filter(s=>s==='Success').length;
assert(attempted>0&&attempted===successes,'benchmark smoke mismatch');
assert(all.trimEnd().endsWith('test result: ok. 2 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s'),'missing final example test result');
for(const file of ['retained-clippy.log','retained-wasm-check.log','retained-fuzz-check-approved.log']){
 const text=read(root+'/'+file);assert(/Finished `dev` profile/.test(text)&&!/^error:/m.test(text),'check failed '+file);
}
const corpus=read(root+'/../field-corpus.tsv').trimEnd().split('\n').slice(1).map(r=>r.split('\t'));
for(const [file,passed,unknown] of [['before-field-release-512.log',1296,28],['retained-field-debug-512.log',1324,0],['retained-field-release-512.log',1324,0]]){
 const lines=read(root+'/'+file).trimEnd().split('\n');assert(lines.pop()===`SUMMARY\t${passed}\t${unknown}\t0\t1324`,'field summary '+file);assert(lines.length===1324,'field coverage');
 let count=0;
 lines.forEach((line,i)=>{const r=line.split('\t'),c=corpus[i];assert(r[1]===c[0]&&r[2]===c[1],'field order');if(r[0]==='PASS'){assert(r[3]===c[2],'wrong proof');count++;}else assert(r[0]==='UNKNOWN'&&r[3]==='UNKNOWN','field error');});
 assert(count===passed,'field pass count');
}
console.log(JSON.stringify({status:'qualified-retained',tests,allTargets:{passed:summary.reduce((n,m)=>n+Number(m[2]),0),benchmarkExecutables:ran,benchmarkSmokeCases:successes},bounded512:{before:{passed:1296,unknown:28},after:{passed:1324,unknown:0}},reportsRestored:true,fullEcosystemGoal:'ACTIVE/OPEN'},null,2));
