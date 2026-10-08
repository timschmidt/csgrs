import {readFileSync,writeFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
const root=import.meta.dirname;
const text=p=>readFileSync(root+'/'+p,'utf8');
const rows=p=>text(p).trimEnd().split('\n').map(s=>s.split('\t'));
const hash=p=>createHash('sha256').update(readFileSync(root+'/'+p)).digest('hex');
const runs=JSON.parse(text('shared-runs.json'));assert.equal(runs.length,8);
for(const r of runs) {
 assert.equal(hash(r.mode+'-'+r.test+'.log'),r.sha256);
 if(r.test==='binary-gcd-zero') {assert.equal(r.error,'ETIMEDOUT');assert.equal(r.signal,'SIGKILL');assert.equal(text(r.mode+'-'+r.test+'.log'),'ENTER\tbinary-gcd-zero\n');}
 else {assert.equal(r.status,r.test==='junit'?1:0);assert.equal(r.error,null);assert.equal(r.signal,null);}
}
const summaries={};
for(const mode of ['jit','interpreter']) {
 const junit=text(mode+'-junit.log');assert(junit.includes('Tests run: 11,  Failures: 1'));assert(junit.includes('1) countQuadraticResidues(AlgorithmsTests)'));assert(junit.includes('n must be a positive odd number but was 2.'));
 let boundedPositivePass=0,boundedPositiveFail=0,negativeMultipleFail=0,otherPass=0,otherFail=0;
 const shared=rows(mode+'-shared.log');assert.equal(shared.length,2014);
 const logging=shared.filter(r=>r[0]==='LOGGING');assert.equal(logging.length,2);
 const unit=1n<<128n,near=1n<<111n;
 assert.deepEqual(logging[0],['LOGGING','add',(unit+near).toString(),unit.toString()]);
 assert.deepEqual(logging[1],['LOGGING','mul',(near*2n).toString(),'0']);
 for(const [kind,mText,nText,gotText] of shared.filter(r=>r[0]==='BARRETT')) {
  const m=BigInt(mText),n=BigInt(nText),got=BigInt(gotText),want=((n%m)+m)%m,pass=got===want;
  const fastLimit=1n<<BigInt(2*m.toString(2).length);
  if(n>=0n&&n<fastLimit) {if(pass)boundedPositivePass++;else boundedPositiveFail++;}
  else if(n<0n&&(-n)%m===0n&&-n<fastLimit) {assert(!pass);negativeMultipleFail++;}
  else if(pass)otherPass++;else otherFail++;
 }
 assert.equal(boundedPositiveFail,0);
 const quotient=shared.filter(r=>r[0]==='QUOTIENT');assert.equal(quotient.length,5);for(const r of quotient)assert.equal(r[2],'false');
 assert.deepEqual(shared.slice(-2),[['FRACTION','invert-zero','1','0'],['STACK','bitlength-one']]);
 const dag=rows(mode+'-dag.log');assert.equal(dag.length,8);
 for(const [kind,dText,len,before,after,coarse] of dag) {
  assert.equal(kind,'DAG');const d=Number(dText);assert.equal(Number(len),4*2**d-3);assert.equal(Number(before),d+1);assert.equal(Number(after),d+2);assert.equal(coarse,after);
 }
 summaries[mode]={junit:{pass:10,fail:1},barrett:{boundedPositivePass,boundedPositiveFail,negativeMultipleFail,otherPass,otherFail},quotientZeroEquivalenceFailures:5,loggingSemanticChanges:2,invalidZeroInverse:'1/0',bitLengthOne:'StackOverflowError',binaryGcdZero:'bounded timeout; source fixed point (0,1)',dag:{depths:8,maxDepth:20,maxRootStringCharacters:4194301,estimatorCallsAtMaxDepth:{constructed:21,refined:22,after100CachedReads:22}}};
}
assert.deepEqual(summaries.jit,summaries.interpreter);
writeFileSync(root+'/shared-analysis.json',JSON.stringify(summaries,null,2)+'\n');console.log(JSON.stringify(summaries.jit,null,2));
