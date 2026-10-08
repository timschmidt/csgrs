import {readFileSync,readlinkSync,writeFileSync}from 'node:fs';
import {execFileSync}from 'node:child_process';
import {createHash}from 'node:crypto';
import assert from 'node:assert/strict';
const root='/home/tim/Documents/GitHub/workspace/exact-real-references/coq-aern';
const sha=b=>createHash('sha256').update(b).digest('hex'),git=args=>execFileSync('git',args,{cwd:root,encoding:'utf8'});
assert.equal(git(['status','--short','--untracked-files=no']),'');
const commit=git(['rev-parse','HEAD']).trim();assert.equal(commit,'bc11353f450cf866b47c3985eee6150a5f99cf00');
const files=git(['ls-files','-s','-z']).split('\0').filter(Boolean).map(row=>{
 const [meta,path]=row.split('\t'),[mode,gitObject,stage]=meta.split(' ');assert.equal(stage,'0');assert(['100644','100755','120000'].includes(mode));
 const buffer=mode==='120000'?Buffer.from(readlinkSync(root+'/'+path)):readFileSync(root+'/'+path);let text=null;
 if(!buffer.includes(0)){try{text=new TextDecoder('utf-8',{fatal:true}).decode(buffer);}catch{}}
 return{path,mode,gitObject,bytes:buffer.length,sha256:sha(buffer),kind:mode==='120000'?'symlink':text===null?'binary':path.endsWith('paper-full.min.js')?'vendored-minified-javascript':path.endsWith('.csv')?'historical-data':'text',
  lines:text===null?null:text.length?text.split('\n').length-(text.endsWith('\n')?1:0):0,...(mode==='120000'?{target:buffer.toString()}:{} )};
});
const result={repository:'https://github.com/holgerthies/coq-aern',commit,commitDate:git(['show','-s','--format=%cI','HEAD']).trim(),root,files,
 totals:files.reduce((a,f)=>{a.files++;a.bytes+=f.bytes;a[f.kind]=(a[f.kind]??0)+1;if(f.kind==='text')a.textLines+=f.lines;return a;},{files:0,bytes:0,textLines:0})};
if(process.argv.includes('--record'))writeFileSync('inventory-v85.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
else assert.deepEqual(JSON.parse(readFileSync('inventory-v85.json')),result);
console.log(JSON.stringify({status:process.argv.includes('--record')?'inventoried-not-read':'inventory-verified',commit,totals:result.totals}));
