import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';

// Check only the static metadata that was read. Never execute donor formatting,
// arbitrary syntax, generated C, or a reproduction of a memory/output failure.
export function checkFexprFormattingMetadata(){
 const roots=[['calcium',''],['flint','src/']],rows=[];
 for(const[repo,prefix]of roots){
  const base=resolve('../../../../exact-real-references',repo);
  const read=p=>readFileSync(resolve(base,p),'utf8');
  const header=read(prefix+'fexpr_builtin.h');
  const enumBody=header.slice(header.indexOf('typedef enum'),header.indexOf('FEXPR_BUILTIN_LENGTH'));
  const names=[...enumBody.matchAll(/^\s*FEXPR_(\w+),?\s*$/gm)].map(m=>m[1]);
  assert.equal(names.length,474);
  const table=read(prefix+'fexpr_builtin/table.c');
  const lines=table.split('\n').filter(s=>/^\s*\{ FEXPR_/.test(s));
  assert.equal(lines.length,names.length);
  const entries=lines.map((s,i)=>{
   const m=s.match(/^\s*\{ FEXPR_(\w+), "([A-Za-z0-9_]+)", "((?:\\.|[^"\\])*)", (NULL|[a-zA-Z0-9_]+),? \},?\s*$/);
   assert(m,repo+':table-row-'+i);assert.equal(m[1],names[i]);assert.equal(m[2],names[i]);
   if(i)assert(names[i-1]<names[i],'ASCII lexical order');
   return{id:i,name:m[1],latexSource:m[3],writer:m[4]};
  });
  const writers=[...new Set(entries.map(e=>e.writer).filter(w=>w!=='NULL'))].sort();
  const latex=read(prefix+'fexpr/write_latex.c');
  for(const w of writers){
   const pattern=new RegExp('\\bvoid\\s+'+w+'\\s*\\([^;{}]*\\)\\s*\\{','g');
   assert.equal([...latex.matchAll(pattern)].length,1,repo+':'+w);
  }
  const docs=read('doc/source/fexpr_builtin.rst');
  const documented=[...docs.matchAll(/^\.\. macro:: (\w+)\s*$/gm)].map(m=>m[1]);
  assert.equal(new Set(documented).size,documented.length);
  for(const name of documented)assert(names.includes(name),repo+':documented-'+name);
  rows.push({repo,entries,documented,writers});
 }
 assert.deepEqual(rows[0].entries,rows[1].entries);
 assert.deepEqual(rows[0].documented,rows[1].documented);
 const sha=createHash('sha256').update(JSON.stringify(rows[0].entries)).digest('hex');
 return{kind:'static-source-metadata-consistency',repositories:2,symbolsPerRepository:474,
  exactPairedTableEntries:474,explicitLatexCallbacks:rows[0].writers.length,
  callbackRows:rows[0].entries.filter(e=>e.writer!=='NULL').length,
  documentedSymbolsPerRepository:rows[0].documented.length,
  normalizedTableSha256:sha,
  limits:'Checks enum/table names and indexes, lexical order, paired metadata identity, documented-name membership and callback definition presence. Does not test numerical support, formatting output, memory safety, linkage size or runtime.'};
}
if(process.argv.includes('--summary'))console.log(JSON.stringify(checkFexprFormattingMetadata()));
