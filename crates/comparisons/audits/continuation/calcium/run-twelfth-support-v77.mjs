import assert from 'node:assert/strict';
import {retainedSources,sha,json,workspace} from './zero-factor-retained-sources-v75.mjs';
import {captured,cargoEnv} from './point-qualified-capture.mjs';
const b=json('twelfth-final-binding-v77.json');
const sources=()=>{assert.deepEqual(retainedSources(),b.current);for(const[p,h]of Object.entries(b.sources))assert.equal(sha(b.root+'/'+p),h,p);};
sources();
for(const variant of ['candidate','baseline']){
 await captured('twelfth-metadata-'+variant+'-v77',variant==='candidate'?b.root+'/hyperreal':workspace+'/hyperreal','cargo',['metadata','--locked','--offline','--all-features','--format-version','1']);sources();
}
for(const[command,args]of [['rustc',['--version','--verbose']],['cargo',['--version']],['valgrind',['--version']]])await captured('twelfth-'+command+'-version-v77','.',command,args);
await captured('twelfth-wasm-build-v77',b.root+'/hyperreal','env',[...cargoEnv,'cargo','build','--locked','--offline','--release','--all-features','--lib','--target','wasm32-unknown-unknown']);sources();
console.log(JSON.stringify({checkpoint:77,status:'metadata-toolchain-wasm-compilation-finished',retained:false,wasmExecution:false}));
