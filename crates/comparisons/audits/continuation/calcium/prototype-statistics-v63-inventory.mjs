import {readFileSync,readdirSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
const patterns={interval:/bootstrap|confidence/i,randomness:/Math\.random|randomBytes|randomInt|xorshift/i,seed:/\bseed\b/};
function inspect(path){
 const source=readFileSync(path,'utf8'),lines=source.split('\n'),matches=[];
 for(const[i,text]of lines.entries()){
  const kinds=Object.entries(patterns).filter(([,re])=>re.test(text)).map(([k])=>k);
  if(kinds.length)matches.push({line:i+1,kinds,text});
 }
 return{path,sha256:sha(path),lines:lines.length-Number(source.endsWith('\n')),matches};
}
export function verifyInventory(snapshot){
 assert.equal(snapshot.schema,1);assert.equal(snapshot.root,resolve('.'));
 assert.deepEqual(snapshot.files.map(f=>f.path),[...new Set(snapshot.files.map(f=>f.path))].sort());
 for(const f of snapshot.files)assert.deepEqual(inspect(f.path),f);
 const known=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path);
 assert.deepEqual(snapshot.knownLcgMatches,known);
 assert.deepEqual(snapshot.otherPotentialFiles,snapshot.files.filter(f=>f.matches.length&&!known.includes(f.path)).map(f=>f.path));
 assert.equal(snapshot.matchingFiles,snapshot.files.filter(f=>f.matches.length).length);
 return{files:snapshot.files.length,matchingFiles:snapshot.matchingFiles,knownLcgMatches:known.length,
  otherPotentialFiles:snapshot.otherPotentialFiles.length,
  scope:'Frozen membership of top-level continuation .mjs files, excluding version-suffixed correction tools. Broad text matches, not an exhaustive workspace scan or semantic estimator classification. Future new files do not retroactively alter this snapshot.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)){
 const files=readdirSync('.',{withFileTypes:true}).filter(f=>f.isFile()&&f.name.endsWith('.mjs')&&!/-v\d+\.mjs$/.test(f.name)).map(f=>f.name).sort().map(inspect),
  known=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path);
 const result={schema:1,scanned:new Date().toISOString(),root:resolve('.'),files,knownLcgMatches:known,
  matchingFiles:files.filter(f=>f.matches.length).length,
  otherPotentialFiles:files.filter(f=>f.matches.length&&!known.includes(f.path)).map(f=>f.path),
  exclusions:'Subdirectories, non-.mjs files, and files ending -v<digits>.mjs are outside this inventory. Matches include report/checker descriptions and seeded test generation, not just performance estimators.'};
 const summary=verifyInventory(result);writeFileSync('prototype-statistics-v63-inventory.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify(summary));
}
