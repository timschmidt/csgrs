import {generator,boundedInteger} from './paired-statistics-v60.mjs';
export {groups,key,plan,rows,validate} from './twelfth-cost-protocol-v78.mjs';
export const config={checkpoint:79,cpu:2,cpuTriples:24,allocationTriples:6,pilotIterations:[1,16],allocationIterations:[1,16],
 targetNs:2000000,maxIterations:2000,seed:'twelfth-revision-native-v79'};
export function shuffle(list,label){const a=[...list],rng=generator(config.seed+':'+label);for(let i=a.length-1;i>0;i--){const j=boundedInteger(()=>rng.word(),i+1);[a[i],a[j]]=[a[j],a[i]];}return a;}
export function orders(n,label){
 if(n%6!==0)throw Error('triples must balance all six permutations');
 const p=[['baseline','prior','candidate'],['baseline','candidate','prior'],['prior','baseline','candidate'],
  ['prior','candidate','baseline'],['candidate','baseline','prior'],['candidate','prior','baseline']];
 return shuffle(Array.from({length:n},(_,i)=>p[i%6]),label);
}
