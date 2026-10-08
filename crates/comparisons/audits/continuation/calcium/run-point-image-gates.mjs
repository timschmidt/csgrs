import {spawn} from 'node:child_process';
import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const mode=process.argv[2],gates=[];
const env=['env','CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
 'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
if(mode==='public'){
 const b=JSON.parse(readFileSync('point-image-binaries.json'));
 gates.push(['public-candidate',b.binaries.cpu.path,'check']);
 gates.push(['public-check','node','check-point-image.mjs','results/power-sums-public-baseline.stdout','results/point-image-public-candidate.stdout']);
}else if(mode==='regression'){
 for(const [variant,profile]of [['baseline','debug'],['baseline','release'],['candidate','release']]){
  const source=variant==='baseline'?'e-plan-qualified-candidate':'point-image-candidate';
  gates.push([variant+'-'+profile,...env,'cargo','test','--offline','--manifest-path',source+'/hypersolve/Cargo.toml',
   ...(profile==='release'?['--release']:[]),'--all-features','--lib','--tests']);
 }
 gates.push(['candidate-doc',...env,'cargo','test','--offline','--manifest-path','point-image-candidate/hypersolve/Cargo.toml','--all-features','--doc']);
 gates.push(['candidate-clippy',...env,'cargo','clippy','--offline','--manifest-path','point-image-candidate/hypersolve/Cargo.toml',
  '--all-features','--all-targets','--','-D','warnings']);
 gates.push(['candidate-fmt','rustfmt','--edition','2024','--check','point-image-candidate/hypersolve/src/algebraic_binary.rs']);
 gates.push(['consumer-debug',...env,'cargo','test','--offline','--manifest-path','point-image-candidate/hypercurve/Cargo.toml',
  '--all-features','--lib','--tests']);
}else if(mode==='memory'){
 const b=JSON.parse(readFileSync('point-image-binaries.json'));
 for(const [v,path]of [['baseline',b.baselineBinaries['baseline-cpu'].path],['candidate',b.binaries.cpu.path]])
  gates.push([v+'-memcheck','valgrind','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',
   '--error-exitcode=99',path,'check']);
}else throw Error('Expected public, regression or memory');
for(const [tag,...command]of gates){
 const args=['capture.mjs','point-image-'+tag,'.',...command];
 const child=spawn(process.execPath,args,{stdio:'inherit'});
 const result=await new Promise((ok,fail)=>{child.on('error',fail);child.on('close',(code,signal)=>ok({code,signal}));});
 assert.equal(result.code,0,tag);assert.equal(result.signal,null,tag);
}
console.log(JSON.stringify({mode,gates:gates.map(g=>'point-image-'+g[0]),status:'pass'}));
