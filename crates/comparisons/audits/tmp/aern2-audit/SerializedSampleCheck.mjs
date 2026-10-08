// Inspect original generated fixtures as data, not executable Haskell.
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import crypto from 'node:crypto';
const root='/home/tim/Documents/GitHub/workspace/exact-real-references/aern2';
const inv=JSON.parse(fs.readFileSync('/tmp/aern2-artifact-inventory.json','utf8'));
const hash=s=>crypto.createHash('sha256').update(s).digest('hex');
const summaries=[];
for(const file of inv.files.filter(f=>f.kind==='gz')) {
  const text=zlib.gunzipSync(fs.readFileSync(path.join(root,file.file))).toString('utf8');
  const lines=text.trimEnd().split('\n'), unique=new Set(), pairs=new Set(), failures=[];
  const [,degreeText,countText]=file.file.match(/deg(\d+)-(\d+)x/), degree=+degreeText;
  let exactCoefficients=0, otherCoefficients=0, parsed=0, omittedZeroCoefficients=0;
  for(let i=0;i<lines.length;i++) {
    const line=lines[i]; unique.add(hash(line));
    if(i%2===0) pairs.add(hash(line+'\n'+lines[i+1]));
    const prefix='(ChPoly (Interval (dyadic (0)) (dyadic (1)){--}) (Poly (terms_fromList [';
    const suffix=']{--})) Nothing)';
    if(!line.startsWith(prefix)||!line.endsWith(suffix)) {failures.push({line:i+1,reason:'unrecognized envelope'});continue;}
    const terms=line.slice(prefix.length,-suffix.length);
    const termRe=/\((\d+),Interval \(dyadic \((-?\d+(?:\*0\.5\^-?\d+)?)\)\) \(dyadic \((-?\d+(?:\*0\.5\^-?\d+)?)\)\)\)/gy;
    let pos=0, lastIndex=-1, termCount=0, valid=true;
    while(pos<terms.length) {
      termRe.lastIndex=pos; const m=termRe.exec(terms);
      if(!m||+m[1]<=lastIndex||+m[1]>degree) {valid=false;break;}
      if(m[2]===m[3]) exactCoefficients++; else otherCoefficients++;
      lastIndex=+m[1];termCount++;pos=termRe.lastIndex;
      if(pos<terms.length) {if(terms[pos]!==',') {valid=false;break;}pos++;}
    }
    if(!valid||lastIndex!==degree) failures.push({line:i+1,reason:'coefficient grammar or degree',parsedTerms:termCount,tail:terms.slice(pos,pos+80)});
    else {parsed++;omittedZeroCoefficients+=degree+1-termCount;}
  }
  summaries.push({file:file.file,degree,expectedPairs:+countText,lines:lines.length,
    uniquePolynomials:unique.size,uniquePairs:pairs.size,parsed,exactCoefficients,otherCoefficients,omittedZeroCoefficients,
    failureCount:failures.length,examples:failures.slice(0,3)});
  console.error(JSON.stringify(summaries.at(-1)));
}
console.log(JSON.stringify(summaries,null,2));
