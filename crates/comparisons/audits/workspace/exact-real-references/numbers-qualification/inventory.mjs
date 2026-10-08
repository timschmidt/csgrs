import {readFileSync, writeFileSync, mkdirSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve, relative} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname, refs=resolve(dir,'..');
const pin='0ab9e067e10a2e097239f73bdcbbffeae09790b0';
const files=['.gitignore','.travis.yml','Data/Number/Abstract.hs','Data/Number/BigFloat.hs',
  'Data/Number/CReal.hs','Data/Number/Dif.hs','Data/Number/Fixed.hs','Data/Number/FixedFunctions.hs',
  'Data/Number/Interval.hs','Data/Number/Natural.hs','Data/Number/Symbolic.hs','Data/Number/Vectorspace.hs',
  'LICENSE','README.md','Setup.hs','Test/Data/Number/BigFloat.hs','TestSuite.hs','default.nix','numbers.cabal'];
const hash=b=>createHash('sha256').update(b).digest('hex');
const rows=files.map(path=>{
  const b=readFileSync(resolve(refs,'numbers',path));
  const lines=b.toString().split('\n').length-(b.at(-1)===10?1:0);
  return [path,b.length,lines,hash(b),'1-'+lines,'independently read',pin].join('\t');
});
const cat=readFileSync(resolve(refs,'haskell-numbers-inventory/README.md'));
rows.push(['../haskell-numbers-inventory/README.md',cat.length,54,hash(cat),'1-54',
  'independently read; catalogue only','366f5c8f4fa02243241060469b04ed4119f2f154'].join('\t'));
const manifest=['path\tbytes\tlines\tsha256\tread_ranges\tstatus\tpin',...rows].join('\n')+'\n';
const out=resolve(refs,'NUMBERS_READ_COVERAGE.tsv');
if(process.argv.includes('--record')) writeFileSync(out,manifest,{flag:'wx'});
else assert.equal(readFileSync(out,'utf8'),manifest,'source/read manifest changed');
console.log(JSON.stringify({sourceFiles:19,sourceLines:rows.slice(0,19).reduce((a,r)=>a+Number(r.split('\t')[2]),0),
  catalogueFiles:1,catalogueLines:54,scope:'Actual independent reading recorded; hashes alone do not establish reading.'},null,2));
if(process.argv.includes('--bridge')) {
  const src=readFileSync(resolve(refs,'numbers/Data/Number/CReal.hs'),'utf8');
  const old='module Data.Number.CReal(CReal, showCReal) where';
  assert.equal(src.split(old).length,2);
  const path=resolve(refs,'../.audit-numbers.rjcbha/AuditCReal.hs');
  const bridge=src.replace(old,'module AuditCReal where');
  writeFileSync(path,bridge);
  assert.equal(readFileSync(path,'utf8').replace('module AuditCReal where',old),src);
  console.log(JSON.stringify({visibilityOnlyBridge:relative(refs,path),sha256:hash(bridge)}));
  const fixed=readFileSync(resolve(refs,'numbers/Data/Number/Fixed.hs'),'utf8');
  const original='newtype Fixed e = F Rational deriving (Eq, Ord, Enum, Real, RealFrac)';
  const replacement='newtype Fixed e = F Rational deriving (Eq, Ord, Enum)\n'
    +'deriving instance Epsilon e => Real (Fixed e)\n'
    +'deriving instance Epsilon e => RealFrac (Fixed e)';
  assert.equal(fixed.split(original).length,2);
  const compat='{-# LANGUAGE StandaloneDeriving #-}\n'+fixed.replace(original,replacement);
  assert.equal(compat.replace('{-# LANGUAGE StandaloneDeriving #-}\n','').replace(replacement,original),fixed);
  const fixedPath=resolve(refs,'../.audit-numbers.rjcbha/compat/Data/Number/Fixed.hs');
  mkdirSync(resolve(fixedPath,'..'),{recursive:true});
  writeFileSync(fixedPath,compat);
  console.log(JSON.stringify({derivingContextOnlyBridge:relative(refs,fixedPath),sha256:hash(compat)}));
}
