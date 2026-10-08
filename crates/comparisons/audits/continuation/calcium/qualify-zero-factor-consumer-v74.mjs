import {readFileSync,writeFileSync,copyFileSync,constants,mkdtempSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {consumerSources,crate,baseline,sha,json} from './zero-factor-consumer-sources-v74.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=consumerSources(),manifest=crate+'/Cargo.toml';
writeFileSync('zero-factor-consumer-run-origin-v74.json',JSON.stringify({checkpoint:74,source,scriptSha256:sha('qualify-zero-factor-consumer-v74.mjs'),
 environmentScriptSha256:sha('point-qualified-environment.mjs'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
await captured('zero-factor-consumer-environment-before-v74','.','node',['point-qualified-environment.mjs']);
for(const [variant,path]of [['baseline',baseline+'/Cargo.toml'],['candidate',manifest]])
 await captured('zero-factor-consumer-'+variant+'-metadata-v74','.','env',[...cargoEnv,'cargo','metadata','--offline','--locked','--all-features','--format-version','1','--manifest-path',path]);
const base=json('results/zero-factor-consumer-baseline-metadata-v74.stdout'),candidate=JSON.parse(readFileSync('results/zero-factor-consumer-candidate-metadata-v74.stdout','utf8')
 .replaceAll(resolve(crate),resolve(baseline)).replaceAll(resolve('zero-factor-candidate-v70/hypersolve'),resolve('point-demand-candidate/hypersolve')));
assert.deepEqual(candidate,base);assert.equal(base.packages.length,187);assert.equal(base.resolve.nodes.length,187);
assert.equal(sha(crate+'/Cargo.lock'),sha(baseline+'/Cargo.lock'));
await captured('zero-factor-consumer-release-v74','.','env',[...cargoEnv,'cargo','test','--offline','--locked','--manifest-path',manifest,
 '--release','--all-features','--no-fail-fast','--','--test-threads=2']);
await captured('zero-factor-consumer-fmt-v74','.','env',[...cargoEnv,'cargo','fmt','--manifest-path',manifest,'--','--check']);
await captured('zero-factor-consumer-clippy-v74','.','env',[...cargoEnv,'cargo','clippy','--offline','--locked','--manifest-path',manifest,
 '--all-targets','--all-features','--','-D','warnings']);
await captured('zero-factor-consumer-wasm-v74','.','env',[...cargoEnv,'cargo','build','--offline','--locked','--manifest-path',manifest,
 '--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
const root=mkdtempSync('/tmp/calcium-zero-consumer.'),prior=json('point-consumer-apps-v66.json'),artifacts=[];
writeFileSync('zero-factor-consumer-app-origin-v74.json',JSON.stringify({checkpoint:74,root,recorded:new Date().toISOString(),
 sourceBindingSha256:sha('zero-factor-consumer-binding-v74.json'),baselineAppsSha256:sha('point-consumer-apps-v66.json')},null,2)+'\n',{flag:'wx'});
await captured('zero-factor-consumer-app-build-v74',crate,'env',[...cargoEnv,'cargo','build','--offline','--locked','--release','--example','basic','--example','arrangement']);
for(const example of ['basic','arrangement']){
 const old=prior.artifacts.find(a=>a.variant==='demand'&&a.example===example&&a.crate==='hypercurve');assert(old);
 for(const f of old.files){assert.equal(sha(f.path),f.sha256);assert.equal(statSync(f.path).size,f.bytes);}
 const reference={...old,variant:'baseline',reusedFrom:'point-consumer-apps-v66.json'},path=root+'/candidate-'+example,stripped=path+'.stripped';
 copyFileSync(target+'/release/examples/'+example,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 await captured('zero-factor-consumer-'+example+'-strip-v74','.','strip',['--strip-all','-o',stripped,path]);
 const current={variant:'candidate',crate:'hypercurve',example,files:[path,stripped].map(path=>({path,bytes:statSync(path).size,sha256:sha(path)}))};
 for(const a of [reference,current])await captured('zero-factor-consumer-'+example+'-'+a.variant+'-run-v74','.',a.files[1].path,[]);
 assert.equal(sha('results/zero-factor-consumer-'+example+'-baseline-run-v74.stdout'),sha('results/zero-factor-consumer-'+example+'-candidate-run-v74.stdout'));
 await captured('zero-factor-consumer-'+example+'-size-v74','.','size',[...reference.files,...current.files].map(f=>f.path));
 artifacts.push(reference,current);
}
await captured('zero-factor-consumer-environment-after-v74','.','node',['point-qualified-environment.mjs']);
assert.deepEqual(consumerSources(),source);assert.equal(sha('qualify-zero-factor-consumer-v74.mjs'),json('zero-factor-consumer-run-origin-v74.json').scriptSha256);
const result={checkpoint:74,status:'consumer-qualification-terminal',root,artifacts,sourceBindingSha256:sha('zero-factor-consumer-binding-v74.json'),
 baselineAppsSha256:sha('point-consumer-apps-v66.json'),dedicatedBytes:artifacts.filter(a=>a.variant==='candidate').flatMap(a=>a.files).reduce((n,f)=>n+f.bytes,0),
 limits:'Full consumer regression and representative default-feature examples; not a timing campaign or proof that these simple examples execute divisor deflation. Build paths/layout/dead-code selection affect sizes. No live adoption.'};
writeFileSync('zero-factor-consumer-apps-v74.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
