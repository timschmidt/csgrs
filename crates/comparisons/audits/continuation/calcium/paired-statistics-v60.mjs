import {createHash,createCipheriv} from 'node:crypto';
import assert from 'node:assert/strict';

export const config={replicates:20000,percentileIndices:[500,19500],
 sampler:'AES-256-CTR deterministic pseudorandom words, rejection to unbiased integer range',
 keyDomain:'hyper-audit-paired-bootstrap-v60',medianConfidence:[95,100]};
export const median=xs=>{
 assert(xs.length>0&&xs.every(Number.isFinite));const x=[...xs].sort((a,b)=>a-b);
 return(x[(x.length-1)>>1]+x[x.length>>1])/2;
};
// Reject the incomplete final residue interval. This is unbiased for uniform
// 32-bit words, rather than deriving indices from the short-cycle LCG low bits.
export function boundedInteger(nextWord,n){
 assert(Number.isSafeInteger(n)&&n>0&&n<=0x100000000);const limit=Math.floor(0x100000000/n)*n;
 for(;;){const word=nextWord();assert(Number.isInteger(word)&&word>=0&&word<=0xffffffff);if(word<limit)return word%n;}
}
export function generator(label){
 const key=createHash('sha256').update(config.keyDomain+'\0'+label).digest();
 const cipher=createCipheriv('aes-256-ctr',key,Buffer.alloc(16)),zero=Buffer.alloc(65536);
 let bytes=Buffer.alloc(0),offset=0,draws=0;
 return{word(){if(offset===bytes.length){bytes=cipher.update(zero);offset=0;}
   const word=bytes.readUInt32LE(offset);offset+=4;draws++;return word;},
  draws(){return draws;},keySha256:key.toString('hex')};
}
export function bootstrapMedian(xs,label,replicates=config.replicates){
 assert(xs.length>0&&xs.every(Number.isFinite));assert(Number.isInteger(replicates)&&replicates>=200);
 const rng=generator(label),sample=new Array(xs.length),medians=new Array(replicates);
 for(let b=0;b<replicates;b++){
  for(let i=0;i<sample.length;i++)sample[i]=xs[boundedInteger(()=>rng.word(),xs.length)];
  medians[b]=median(sample);
 }
 medians.sort((a,b)=>a-b);
 return{interval:[medians[Math.floor(replicates*0.025)],medians[Math.floor(replicates*0.975)]],
  replicates,draws:rng.draws(),keySha256:rng.keySha256};
}
// Distribution-free order-statistic interval for the population median,
// conditional on independent observations from a common distribution. With
// ties this is conservative; independence is NOT proved by timing collection.
export function medianOrderInterval(xs){
 assert(xs.length>=6&&xs.every(Number.isFinite));const sorted=[...xs].sort((a,b)=>a-b),n=xs.length;
 const denominator=1n<<BigInt(n);let choose=1n,tail=0n,best;
 for(let k=1;k<=Math.floor((n+1)/2);k++){
  tail+=choose;const numerator=denominator-2n*tail;
  if(numerator*100n>=95n*denominator)best={interval:[sorted[k-1],sorted[n-k]],
   lowerRank:k,upperRank:n-k+1,coverageNumerator:numerator.toString(),coverageDenominator:denominator.toString(),
   nominalCoverage:Number(numerator)/Number(denominator)};
  choose=choose*BigInt(n-k+1)/BigInt(k);
 }
 assert(best);return best;
}
export function correctedPairedStatistics(ratios,label){
 assert(ratios.every(x=>Number.isFinite(x)&&x>0));
 return{pairedBlockRatios:ratios,pairedMedianRatio:median(ratios),
  bootstrap:bootstrapMedian(ratios,label),orderStatistic:medianOrderInterval(ratios)};
}

export function statisticsSelfTest(){
 assert.equal(median([4,1,3,2]),2.5);assert.equal(median([2,1,3]),2);
 const order=medianOrderInterval(Array.from({length:12},(_,i)=>i+1));
 assert.deepEqual(order,{interval:[3,10],lowerRank:3,upperRank:10,coverageNumerator:'3938',coverageDenominator:'4096',nominalCoverage:3938/4096});
 assert.deepEqual(medianOrderInterval(Array(12).fill(2)).interval,[2,2]);
 let i=0;const words=[0xffffffff,0xfffffffe,0xfffffffc,0xfffffffb];
 assert.equal(boundedInteger(()=>words[i++],12),11);assert.equal(i,4);
 const a=generator('reproducible'),b=generator('reproducible');
 assert.deepEqual(Array.from({length:20000},()=>a.word()),Array.from({length:20000},()=>b.word()));
 // A four-point population gives an exhaustive finite bootstrap oracle.
 // For [0,0,0,1], 189 samples have median 0, 54 have .5, and 13 have 1.
 const exact=[];for(let mask=0;mask<256;mask++){
  let m=mask;const sample=[];for(let j=0;j<4;j++){sample.push(m%4===3?1:0);m=Math.floor(m/4);}exact.push(median(sample));
 }
 const counts=[0,.5,1].map(x=>exact.filter(y=>x===y).length);assert.deepEqual(counts,[189,54,13]);
 exact.sort((a,b)=>a-b);assert.deepEqual([exact[6],exact[249]],[0,1]);
 let seed=532825;const oldIndices=[],oldMedians=[];
 for(let sample=0;sample<5000;sample++){
  const indices=Array.from({length:12},()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%12;});
  for(let residue=0;residue<4;residue++)assert.equal(indices.filter(i=>i%4===residue).length,3);
  if(sample<3)oldIndices.push(indices);
 }
 seed=532825;
 for(let sample=0;sample<5000;sample++)oldMedians.push(median(Array.from({length:4},()=>{
  seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%4===3?1:0;})));
 assert(oldMedians.every(x=>x===0));
 const corrected=bootstrapMedian([0,0,0,1],'exhaustive-four-point-check',50000);assert.deepEqual(corrected.interval,[0,1]);
 const varied=generator('residue-test');let nonbalanced=0;
 for(let sample=0;sample<100;sample++){
  const counts=[0,0,0,0];for(let i=0;i<12;i++)counts[boundedInteger(()=>varied.word(),12)%4]++;
  if(counts.some(n=>n!==3))nonbalanced++;
 }
 assert(nonbalanced>80);
 return{status:'pass',orderStatistic12:order,rejectionTestWords:words,rejectionConsumed:i,
  exhaustiveFourPointCounts:counts,exhaustive95:[0,1],oldFourPoint5000Interval:[0,0],correctedFourPoint:corrected,
  oldTwelveIndexExamples:oldIndices,oldResidueCounts:[3,3,3,3],correctedNonbalancedOf100:nonbalanced,
  algebraicProof:'1664525 == 1 mod 4; 1013904223 == 3 mod 4; reduction modulo 2^32 preserves modulo 4. For n divisible by four, seed % n retains those forced residue classes.',
  limits:'Tests establish implementation controls and a concrete failure of the historical resampler. Cryptographic pseudorandom sampling is deterministic, not a proof of statistical independence of recorded timing blocks; both new intervals remain conditional on sampling assumptions.'};
}
