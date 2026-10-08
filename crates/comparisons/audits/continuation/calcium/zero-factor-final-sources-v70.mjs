import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {zeroSources,sha,json} from './zero-factor-sources-v70.mjs';
export {sha,json};
export function finalZeroSources(){
 const s=zeroSources(),initial=json('zero-factor-binding-v70.json'),signed=json('zero-factor-fixed-binding-v70.json'),file=s.added;
 for(const[p,h]of Object.entries(initial.source))if(p!==file){assert.equal(s.source[p],h,p);assert.equal(signed.source[p],h,p);}
 assert.equal(sha('zero-factor-tests-initial-v70.rs'),initial.source[file]);
 assert.equal(sha('zero-factor-tests-signed-v70.rs'),signed.source[file]);
 const old=readFileSync('zero-factor-tests-initial-v70.rs','utf8'),fixed=readFileSync('zero-factor-tests-signed-v70.rs','utf8');
 assert.equal(fixed,old.replace('carrier(&[0, 1], 1));','carrier(&[0, 1], scale.signum()));'));
 assert.equal(readFileSync(s.root+'/'+file,'utf8'),fixed.replace(
  '            assert_eq!(root.polynomial_coefficients, carrier(&[0, 1], scale.signum()));',
  '            assert_eq!(\n                root.polynomial_coefficients,\n                carrier(&[0, 1], scale.signum())\n            );'));
 for(const [tag,code]of [['unit',101],['fmt',1],['gates',1]]){
  const gate=json('results/zero-factor-'+tag+'-v70.json');assert.equal(gate.code,code);assert.equal(gate.signal,null);
 }
 return {...s,initialBindingSha256:sha('zero-factor-binding-v70.json'),signedBindingSha256:sha('zero-factor-fixed-binding-v70.json'),
  initialTestsSha256:sha('zero-factor-tests-initial-v70.rs'),signedTestsSha256:sha('zero-factor-tests-signed-v70.rs'),
  amendment:'One signed new-result test expectation corrected, then rustfmt wrapped that assertion. All failed captures and both test versions preserved. Production algorithm unchanged throughout.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const s=finalZeroSources();if(process.argv.includes('--record'))writeFileSync('zero-factor-final-binding-v70.json',JSON.stringify(s,null,2)+'\n',{flag:'wx'});
 else assert.deepEqual(s,json('zero-factor-final-binding-v70.json'));
 console.log(JSON.stringify({checkpoint:70,amendedTestsOnly:true,files:Object.keys(s.source).length,liveFiles:s.current.liveFiles}));
}
