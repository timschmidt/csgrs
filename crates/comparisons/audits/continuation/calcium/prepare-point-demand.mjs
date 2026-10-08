import {existsSync,mkdirSync,copyFileSync,constants,readFileSync,writeFileSync,statSync} from 'node:fs';
import {dirname} from 'node:path';
import assert from 'node:assert/strict';
import {sources,sha} from './point-qualified-sources.mjs';
const o=sources(),root='point-demand-candidate';assert(!existsSync(root));
const sourceFiles=Object.keys(o.candidateSources).filter(p=>p.startsWith('hypersolve/'));
let bytes=0;
for(const p of sourceFiles){
 const dest=root+'/'+p;mkdirSync(dirname(dest),{recursive:true});
 copyFileSync(o.candidate+'/'+p,dest,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);bytes+=statSync(dest).size;
}
// Mechanical path relocation only. All reused dependency sources are checked
// against the retained 956-file map; no snapshot is edited through these paths.
const manifest=root+'/hypersolve/Cargo.toml';
let text=readFileSync(manifest,'utf8');
for(const crate of ['hyperreal','hyperlattice','hyperlimit']){
 const old='path = "../'+crate+'"';assert.equal(text.split(old).length,2);
 text=text.replace(old,'path = "../../'+o.baseline+'/'+crate+'"');
}
writeFileSync(manifest,text);
const origin={schema:1,checkpoint:57,recorded:new Date().toISOString(),root,template:o.candidate,
 templateOriginSha256:sha('point-qualified-origin.json'),predecessor:'point-history-manifest.json',
 predecessorSha256:sha('point-history-manifest.json'),sourceFiles,copiedBytes:bytes,
 originalSources:Object.fromEntries(sourceFiles.map(p=>[p,o.candidateSources[p]]))};
writeFileSync('point-demand-origin.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({root,sourceFiles:sourceFiles.length,copiedBytes:bytes,dependencies:o.baseline,status:'isolated-solver-copy-ready'}));
