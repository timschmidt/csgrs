import {resolve} from 'node:path';
export function specifications(retained=false) {
 const list=[],base='derivative-demand-candidate',cand='e-plan-qualified-candidate',apps='/tmp/calcium-e-qualified-apps.AHiwfn';
 const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
 const add=(tag,cwd,command,args)=>list.push({tag:'e-qualified-'+tag,cwd:resolve(cwd),command,args});
 const cargo=(tag,cwd,args)=>add(tag,cwd,'env',[...env,'cargo',...args]);
 cargo('focused-debug',cand+'/hyperreal',['test','--locked','--offline','--all-features','--lib','e_plan_tests','--','--test-threads=4']);
 for(const v of ['baseline','candidate'])for(const profile of ['debug','release'])
  cargo(v+'-default-'+profile,(v==='baseline'?base:cand)+'/hyperreal',['test','--locked','--offline','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
 for(const profile of ['debug','release'])cargo('candidate-all-'+profile,cand+'/hyperreal',['test','--locked','--offline','--all-features','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
 cargo('candidate-doc',cand+'/hyperreal',['test','--locked','--offline','--all-features','--doc']);
 cargo('candidate-fmt',cand+'/hyperreal',['fmt','--all','--','--check']);
 cargo('candidate-clippy-all',cand+'/hyperreal',['clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
 cargo('candidate-check-default',cand+'/hyperreal',['check','--locked','--offline','--all-targets']);
 cargo('candidate-fuzz',cand+'/hyperreal',['check','--locked','--offline','--manifest-path','fuzz/Cargo.toml','--bins']);
 for(const v of ['baseline','candidate']) {
  const root=v==='baseline'?base:cand;
  cargo(v+'-wasm-scalar',root+'/hyperreal',['build','--locked','--offline','--release','--lib','--all-features','--target','wasm32-unknown-unknown']);
  cargo(v+'-wasm-consumer',root+'/hypercurve',['build','--locked','--offline','--release','--lib','--features','triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
  for(const crate of ['hypersolve','hypercurve'])cargo(v+'-'+crate+'-release',root+'/'+crate,
   ['test','--locked','--offline','--release','--all-features','--no-fail-fast','--','--test-threads=2']);
  cargo('wasm-build-'+v,'.',['build','--offline','--release','--target','wasm32-unknown-unknown','--manifest-path','e-qualified-wasm-'+v+'/Cargo.toml','--lib']);
  for(const[crate,examples]of [['hyperreal',['readme_quickstart']],['hypercurve',['basic','arrangement']]]) {
   cargo('app-'+v+'-'+crate+'-build',root+'/'+crate,['build','--locked','--offline','--release',...examples.flatMap(e=>['--example',e])]);
   for(const e of examples) {
    const tag='app-'+v+'-'+crate+'-'+e.replaceAll('_','-'),path=apps+'/'+v+'-'+crate+'-'+e,stripped=path+'.stripped';
    add(tag+'-strip','.','strip',['--strip-all','-o',stripped,path]);add(tag+'-size','.','size',[path,stripped]);add(tag+'-run','.',stripped,[]);
   }
  }
 }
 cargo('candidate-consumer-clippy',cand+'/hypercurve',['clippy','--locked','--offline','--all-targets','--all-features','--','-D','warnings']);
 add('wasm-check','.','node',['run-e-qualified-wasm.mjs','check']);
 add('app-size-resume','.','node',['measure-e-qualified-app-size.mjs']);
 add('controls','.','node',['run-e-qualified-controls.mjs']);
 add('wasm-cpu','.','taskset',['-c','6','node','--expose-gc','run-e-qualified-wasm-costs.mjs']);
 add('output-check','.','node',['check-e-qualified.mjs','--e-qualified-summary']);
 if(retained) {
  for(const profile of ['debug','release']) {
   cargo('retained-focused-'+profile,'../../../../hyperreal',['test','--locked','--offline','--all-features','--lib','e_plan_tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
   cargo('retained-all-'+profile,'../../../../hyperreal',['test','--locked','--offline','--all-features','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
  }
  cargo('retained-fmt','../../../../hyperreal',['fmt','--all','--','--check']);
 }
 return list;
}
