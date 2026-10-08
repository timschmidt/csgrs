import {readFileSync,readdirSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {verifyInventory as verifyInitial} from './prototype-statistics-v63-inventory.mjs';
const patterns={interval:/bootstrap|confidence/i,randomness:/Math\.random|randomBytes|randomInt|xorshift/i,seed:/\bseed\b/};
function inspect(path){
 const source=readFileSync(path,'utf8'),lines=source.split('\n'),matches=[];
 for(const[i,text]of lines.entries()){
  const kinds=Object.entries(patterns).filter(([,re])=>re.test(text)).map(([k])=>k);
  if(kinds.length)matches.push({line:i+1,kinds,text});
 }
 return{path,sha256:sha(path),lines:lines.length-Number(source.endsWith('\n')),matches};
}
export function included(name){
 const version=name.match(/-v(\d+)\.mjs$/);
 return name.endsWith('.mjs')&&(!version||Number(version[1])<60);
}
export function verifyInventory(snapshot){
 assert.equal(snapshot.schema,2);assert.equal(snapshot.root,resolve('.'));
 assert.equal(snapshot.initialSha256,sha('prototype-statistics-v63-inventory.json'));
 verifyInitial(json('prototype-statistics-v63-inventory.json'));
 assert.deepEqual(snapshot.files.map(f=>f.path),[...new Set(snapshot.files.map(f=>f.path))].sort());
 for(const f of snapshot.files){assert(included(f.path));assert.deepEqual(inspect(f.path),f);}
 const known=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path);
 assert.deepEqual(snapshot.knownLcgMatches,known);
 for(const p of known)assert(snapshot.files.some(f=>f.path===p&&f.matches.length),p);
 const added=snapshot.files.filter(f=>!json('prototype-statistics-v63-inventory.json').files.some(o=>o.path===f.path)).map(f=>f.path);
 assert.deepEqual(added.filter(p=>known.includes(p)),['check-complex-product-v2.mjs','snapshot-v43-check-complex-product-v2.mjs']);
 assert.equal(added.length,14);assert(added.every(p=>/-v\d+\.mjs$/.test(p)));
 assert.deepEqual(snapshot.otherPotentialFiles,snapshot.files.filter(f=>f.matches.length&&!known.includes(f.path)).map(f=>f.path));
 assert.equal(snapshot.matchingFiles,snapshot.files.filter(f=>f.matches.length).length);
 assert(included('check-complex-product-v2.mjs'));assert(!included('reanalyse-point-statistics-v60.mjs'));
 assert(!included('nested.txt'));assert(included('future-v3.mjs'));
 return{files:snapshot.files.length,matchingFiles:snapshot.matchingFiles,knownLcgMatches:known.length,
  otherPotentialFiles:snapshot.otherPotentialFiles.length,recoveredHistoricalFiles:added,
  scope:'Frozen top-level .mjs membership, excluding correction files ending -v60.mjs or later. The initial snapshot excluded 14 historical scripts, including two known legacy v2 statistical checkers; it remains preserved as an incomplete inventory. Text matches are not semantic estimator classification, and no subdirectory or workspace-wide closure is claimed.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const files=readdirSync('.',{withFileTypes:true}).filter(f=>f.isFile()&&included(f.name)).map(f=>f.name).sort().map(inspect),
  known=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path);
 const result={schema:2,scanned:new Date().toISOString(),root:resolve('.'),initialSha256:sha('prototype-statistics-v63-inventory.json'),
  files,knownLcgMatches:known,matchingFiles:files.filter(f=>f.matches.length).length,
  otherPotentialFiles:files.filter(f=>f.matches.length&&!known.includes(f.path)).map(f=>f.path)};
 const summary=verifyInventory(result);
 writeFileSync('prototype-statistics-v63-inventory-corrected.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify(summary));
}
