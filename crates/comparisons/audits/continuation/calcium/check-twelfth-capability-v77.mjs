import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {hyperEvidence} from './check-qqbar-trig-hyper-v76.mjs';
function check(text,baseline){
 const rows=text.trimEnd().split('\n'),old=baseline.trimEnd().split('\n');
 assert.equal(rows.shift(),'k,shift,op,precision,control,outcome');assert.equal(old.shift(),'k,shift,op,precision,control,outcome');
 const counts={},changed=new Set();let index=0,improvements=0;
 for(let k=0;k<24;k++)for(let shift=0;shift<3;shift++)for(const op of ['sin','cos','tan','cot'])for(const precision of [-64,-256])for(const control of ['radical','periodic','perturbed']){
  const prefix=[k,shift,op,precision,control].join(',')+',';
  const pole=(op==='tan'&&(k===6||k===18))||(op==='cot'&&(k===0||k===12));
  const expected=pole?'Pole':control==='perturbed'?'NotEqual':'Equal';
  assert.equal(rows[index],prefix+expected);
  const previouslyUnknown=!pole&&control==='radical'&&k%2!==0&&k%3!==0;
  assert.equal(old[index],prefix+(previouslyUnknown?'Unknown':expected));
  if(previouslyUnknown){improvements++;changed.add(k+':'+op);}else assert.equal(rows[index],old[index]);
  counts[control+':'+expected]=(counts[control+':'+expected]??0)+1;index++;
 }
 assert.equal(rows.length,index);assert.equal(old.length,index);assert.equal(index,1728);assert.equal(improvements,192);assert.equal(changed.size,32);
 return {records:index,counts,improvedRows:192,improvedIdentities:[...changed].sort(),unchangedRows:1536,unknownRows:0};
}
export function capability(){
 const previous=hyperEvidence(),baseline=readFileSync('results/qqbar-trig-hyper-debug-fixed-v76.stdout','utf8');
 const debug=readFileSync('results/twelfth-capability-debug-final-v77.stdout','utf8'),release=readFileSync('results/twelfth-capability-release-final-v77.stdout','utf8');
 assert.equal(debug,release);const evidence=check(debug,baseline);let rejected=0;
 for(const mutate of [s=>s.replace(',radical,Equal',',radical,Unknown'),s=>s.replace(',NotEqual',',Equal'),s=>s.replace(',Pole',',Equal'),
  s=>s.replace(',periodic,Equal',',periodic,NotEqual'),s=>s.split('\n').slice(0,-2).join('\n'),
  s=>{const rows=s.split('\n');rows[2]=rows[1];return rows.join('\n');}]){
  assert.throws(()=>check(mutate(debug),baseline));rejected++;
 }
 return {checkpoint:77,...evidence,profiles:['debug','release'],profilesByteIdentical:true,corruptionControlsRejected:rejected,
  priorUnresolvedIdentities:previous.unresolvedIdentities,retained:false,
  limits:'Initial candidate capability, not a universal closure claim or performance/retention result. Exact pi/12 formulas were independently qualified in checkpoint 76; all original source and baseline records remain unchanged.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(capability()));
