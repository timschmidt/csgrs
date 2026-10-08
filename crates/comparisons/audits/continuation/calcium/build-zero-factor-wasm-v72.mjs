import {mkdtempSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import assert from 'node:assert/strict';
import {wasmSources,sha} from './zero-factor-wasm-sources-v72.mjs';
import {captured,cargoEnv,target} from './point-qualified-capture.mjs';
const source=wasmSources(),root=mkdtempSync('/tmp/calcium-zero-wasm.'),artifacts=[];
writeFileSync('zero-factor-wasm-origin-v72.json',JSON.stringify({checkpoint:72,source,root,recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
for(const variant of ['baseline','candidate'])for(const kind of ['rational','history']){
 const app='zero-factor-wasm-'+variant+'-'+kind+'-v72',tag='zero-factor-wasm-'+variant+'-'+kind;
 for(const [name,args]of [['lock',['generate-lockfile','--offline']],['metadata',['metadata','--offline','--locked','--format-version','1']],
  ['build',['build','--offline','--locked','--release','--target','wasm32-unknown-unknown','--lib']],
  ['clippy',['clippy','--offline','--locked','--target','wasm32-unknown-unknown','--lib','--','-D','warnings']]])
  await captured(tag+'-'+name+'-v72',app,'env',[...cargoEnv,'cargo',...args]);
 const name=kind==='rational'?'zero_factor_wasm_v72':'zero_factor_history_v72',path=root+'/'+variant+'-'+kind+'.wasm';
 copyFileSync(target+'/wasm32-unknown-unknown/release/'+name+'.wasm',path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
 artifacts.push({variant,kind,path,bytes:statSync(path).size,sha256:sha(path)});
}
await captured('zero-factor-wasm-fmt-v72','.','rustfmt',['--check','--edition','2024','--config','skip_children=true','zero-factor-wasm-v72.rs']);
assert.deepEqual(wasmSources(),source);
writeFileSync('zero-factor-wasm-binaries-v72.json',JSON.stringify({checkpoint:72,root,artifacts,originSha256:sha('zero-factor-wasm-origin-v72.json'),recorded:new Date().toISOString()},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:72,status:'wasm-builds-terminal',artifacts,limits:'Compilation/lint only. Full runtime qualification and later costs/consumer/size decisions remain open.'}));
