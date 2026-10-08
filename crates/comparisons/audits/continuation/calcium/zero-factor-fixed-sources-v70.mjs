import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {zeroSources,sha,json} from './zero-factor-sources-v70.mjs';
export {sha,json};
export function fixedZeroSources(){
 const s=zeroSources(),initial=json('zero-factor-binding-v70.json'),file=s.added;
 for(const[p,h]of Object.entries(initial.source))if(p!==file)assert.equal(s.source[p],h,p);
 assert.equal(sha('zero-factor-tests-initial-v70.rs'),initial.source[file]);
 const old=readFileSync('zero-factor-tests-initial-v70.rs','utf8'),from='carrier(&[0, 1], 1));',to='carrier(&[0, 1], scale.signum()));';
 assert.equal(old.split(from).length-1,1);
 assert.equal(readFileSync(s.root+'/'+file,'utf8'),old.replace(from,to));
 const gate=json('results/zero-factor-unit-v70.json');assert.equal(gate.code,101);assert.equal(gate.signal,null);
 assert(readFileSync('results/zero-factor-unit-v70.stdout','utf8').includes('6 passed; 1 failed; 0 ignored'));
 return {...s,initialBindingSha256:sha('zero-factor-binding-v70.json'),initialTestsSha256:sha('zero-factor-tests-initial-v70.rs'),
  amendment:'Correct one new-result test expectation to preserve input scale sign after existing monic-GCD division. Independent determinant confirms both signs. Algorithm unchanged; initial 6-pass/1-fail run retained.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const s=fixedZeroSources();if(process.argv.includes('--record'))writeFileSync('zero-factor-fixed-binding-v70.json',JSON.stringify(s,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(s,json('zero-factor-fixed-binding-v70.json'));
 console.log(JSON.stringify({checkpoint:70,amendedTestsOnly:true,files:Object.keys(s.source).length,liveFiles:s.current.liveFiles}));
}
