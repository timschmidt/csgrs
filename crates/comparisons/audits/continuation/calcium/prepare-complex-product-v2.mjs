import {readFileSync,mkdirSync,copyFileSync,constants,writeFileSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),read=p=>readFileSync(resolve(here,p),'utf8');
const old=JSON.parse(read('complex-product-experiment.json'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const patch=(name,content)=>'*** Begin Patch\n*** Add File: '+resolve(here,name)+'\n'+content.trimEnd().split('\n').map(l=>'+'+l).join('\n')+'\n*** End Patch\n';
if(process.argv.includes('--cost-patch')) {
 const name='run-complex-product-costs.mjs';assert.equal(sha(name),old.files[name]);
 let s=read(name),a=s.indexOf("if(mode==='freeze') {"),b=s.indexOf('} else {',a);
 assert(a>0&&b>a);
 s=s.slice(0,a)+'{'+s.slice(b+'} else {'.length);
 s=s.replace("['freeze','cpu','allocation']","['cpu','allocation']");
 s=s.replaceAll('complex-product-','complex-product-v2-');
 console.log(patch('run-complex-product-v2-costs.mjs',s));
} else if(process.argv.includes('--check-patch')) {
 const name='check-complex-product.mjs';assert.equal(sha(name),old.files[name]);
 let s=read(name).replaceAll('complex-product-','complex-product-v2-');
 s=s.replace('import {json,sources}', 'import {json as sourceJson,sources}');
 const marker="const read=p=>readFileSync(new URL(p,import.meta.url),'utf8');";
 assert(s.includes(marker));
 s=s.replace(marker,
  "const evidencePath=p=>p.replace(/^results\\/complex-product-v2-(check|memcheck|trace)-baseline/, 'results/complex-product-$1-baseline');\n"+
  "const json=p=>sourceJson(evidencePath(p));\n"+
  "const read=p=>readFileSync(new URL(evidencePath(p),import.meta.url),'utf8');");
 s=s.replaceAll('r.bits<=128','r.bits<256').replaceAll('r.bits>128','r.bits>=256');
 console.log(patch('check-complex-product-v2.mjs',s));
} else if(process.argv.includes('--app-patch')) {
 const name='complex-product-app-candidate/Cargo.toml';assert.equal(sha(name),old.files[name]);
 const s=read(name).replaceAll('complex-product-candidate','complex-product-v2-candidate');
 console.log(patch('complex-product-v2-app-candidate/Cargo.toml',s));
} else {
 const destination='complex-product-v2-candidate',origin='complex-product-candidate';
 mkdirSync(resolve(here,destination));let bytes=0;
 for(const[p,h]of Object.entries(old.sourceMap.candidate)) {
  assert.equal(sha(origin+'/'+p),h,p);const target=resolve(here,destination,p);
  mkdirSync(dirname(target),{recursive:true});
  copyFileSync(resolve(here,origin,p),target,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  assert.equal(sha(target),h,p);bytes+=readFileSync(target).length;
 }
 writeFileSync(resolve(here,'complex-product-v2-origin.json'),JSON.stringify({
  recorded:new Date().toISOString(),origin,destination,sourceHashes:old.sourceMap.candidate,
  baselineHashes:old.liveSources,copiedFiles:955,copiedBytes:bytes,
  note:'Isolated v2 copy. Frozen v1 and all live files unchanged. Existing baseline executables and source are reused.'
 },null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({copiedFiles:955,copiedBytes:bytes,origin,destination}));
}
