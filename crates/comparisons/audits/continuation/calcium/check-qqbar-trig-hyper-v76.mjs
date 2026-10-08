import {readFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
function check(text){
 const rows=text.trimEnd().split('\n');assert.equal(rows.shift(),'k,shift,op,precision,control,outcome');
 const counts={},identities=new Set();let index=0;
 for(let k=0;k<24;k++)for(let shift=0;shift<3;shift++)for(const op of ['sin','cos','tan','cot'])for(const precision of [-64,-256])for(const control of ['radical','periodic','perturbed']){
  const prefix=[k,shift,op,precision,control].join(',')+',';const row=rows[index++];assert(row?.startsWith(prefix));
  const outcome=row.slice(prefix.length);const pole=(op==='tan'&&(k===6||k===18))||(op==='cot'&&(k===0||k===12));
  const expected=pole?'Pole':control==='perturbed'?'NotEqual':control==='radical'&&k%2!==0&&k%3!==0?'Unknown':'Equal';
  assert.equal(outcome,expected,'Recorded bounded capability '+prefix);
  counts[control+':'+outcome]=(counts[control+':'+outcome]??0)+1;
  if(outcome==='Unknown')identities.add(k+':'+op);
 }
 assert.equal(rows.length,index);assert.equal(index,1728);assert.equal(identities.size,32);
 return {records:index,counts,unresolvedIdentities:[...identities].sort(),unresolvedRepeats:192,
  poles:72,periodicEqualities:552,unequalControls:552,radicalEqualities:360};
}
export function hyperEvidence(){
 const debug=readFileSync('results/qqbar-trig-hyper-debug-fixed-v76.stdout','utf8');
 const release=readFileSync('results/qqbar-trig-hyper-release-fixed-v76.stdout','utf8');
 assert.equal(debug,release);const evidence=check(debug);let rejected=0;
 for(const mutate of [s=>s.replace(',Unknown',',NotEqual'),s=>s.replace(',NotEqual',',Equal'),s=>s.replace(',Pole',',Equal'),
  s=>s.replace(',periodic,Equal',',periodic,Unknown'),s=>s.split('\n').slice(0,-2).join('\n'),
  s=>{const a=s.split('\n');a[2]=a[1];return a.join('\n');}]){
  assert.throws(()=>check(mutate(debug)));rejected++;
 }
 return {checkpoint:76,status:'bounded-hyper-capability-verified',...evidence,profiles:['debug','release'],profilesByteIdentical:true,
  corruptionControlsRejected:rejected,
  limits:'Expected radical identities come from exact pi/12 formulas independently qualified in the donor field corpus; this records current bounded predicate capability, not a claim that Unknown means inequality, no other identity is provable, or every Hyper value/feature was tested. No candidate or benchmark.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(hyperEvidence()));
