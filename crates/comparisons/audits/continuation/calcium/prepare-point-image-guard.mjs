import {readFileSync,writeFileSync,mkdirSync,copyFileSync,constants,statSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {checkPointImageSources,sha} from './point-image-sources.mjs';
const hashes=checkPointImageSources(),selected=Object.keys(hashes).filter(p=>p.startsWith('hypersolve/'));
const source='point-image-candidate',destination='point-image-guard';mkdirSync(destination);let bytes=0;
for(const p of selected){const to=destination+'/'+p;mkdirSync(dirname(to),{recursive:true});
 copyFileSync(source+'/'+p,to,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);assert.equal(sha(to),hashes[p]);bytes+=statSync(to).size;}
// Mechanical relocation: all scalar dependencies still use the already frozen
// point-image source tree; no dependency or version is changed.
const manifest=destination+'/hypersolve/Cargo.toml';let text=readFileSync(manifest,'utf8');
for(const name of ['hyperreal','hyperlattice','hyperlimit']){
 const old='path = "../'+name+'"';assert.equal(text.split(old).length,2);
 text=text.replace(old,'path = "../../point-image-candidate/'+name+'"');
}
writeFileSync(manifest,text);
copyFileSync(source+'/hypersolve/src/algebraic_binary.rs','point-image-v2-algebraic_binary.rs',constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
writeFileSync('point-image-guard-origin.json',JSON.stringify({checkpoint:53,source,destination,sourceHashes:hashes,
 copiedFiles:selected.length,copiedBytes:bytes,relocatedManifestSha256:sha(manifest),
 note:'Small solver-only proof-control copy while the unchanged v2 consumer suite runs. Scalar/dependency sources and shared cache reused; no production or original candidate edit.'},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({source,destination,files:selected.length,bytes}));
