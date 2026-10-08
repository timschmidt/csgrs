import {sources} from './twelfth-revision-sources-v79.mjs';
import {workspace} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const b=sources(),root=b.root+'/hyperreal';
await captured('twelfth-fmt-v79',root,'cargo',['fmt','--all','--','--check']);
await captured('twelfth-private-fmt-v79',root,'rustfmt',['--edition','2024','--check',
 'src/computable/node/twelfth_relation.rs','src/computable/node/twelfth_relation_tests.rs','src/computable/node/twelfth_relation_reference_tests.rs']);
await captured('twelfth-clippy-v79',root,'env',[...cargoEnv,'cargo','clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
for(const variant of ['candidate','baseline']){
 const cwd=variant==='candidate'?root:workspace+'/hyperreal';
 for(const profile of ['debug','release'])for(const features of ['default','all']){
  await captured('twelfth-'+variant+'-'+features+'-'+profile+'-v79',cwd,'env',[...cargoEnv,'cargo','test','--locked','--offline',
   ...(profile==='release'?['--release']:[]),...(features==='all'?['--all-features']:[]),'--lib','--tests']);sources();
 }
 await captured('twelfth-metadata-'+variant+'-v79',cwd,'cargo',['metadata','--locked','--offline','--all-features','--format-version','1']);
}
await captured('twelfth-wasm-build-v79',root,'env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--all-features','--lib','--target','wasm32-unknown-unknown']);
await captured('twelfth-capability-lock-v79','twelfth-capability-v79','env',[...cargoEnv,'cargo','generate-lockfile','--offline']);
for(const profile of ['debug','release']){
 await captured('twelfth-capability-'+profile+'-v79','twelfth-capability-v79','env',[...cargoEnv,'cargo','run','--locked','--offline',...(profile==='release'?['--release']:[])]);sources();
}
for(const[tool,args]of [['rustc',['--version','--verbose']],['cargo',['--version']]])await captured('twelfth-'+tool+'-version-v79','.',tool,args);
sources();console.log(JSON.stringify({checkpoint:79,status:'revision-regression-captures-complete',retained:false,candidateFiles:184}));
