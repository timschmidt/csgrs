import {readFileSync,writeFileSync,copyFileSync,constants,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft=process.argv.includes('--draft-matrix-pivots');
const manifest={schema:1,recorded:new Date().toISOString(),files:{},binaries:{},reads:[],gates:[],
  status:'Matrix source/capability checkpoint and completed monic downstream; monic live retention recorded separately. Full ecosystem incomplete.',
  limits:'Native determinant oracle covers finite diagonal, row-swapped and repeated-row matrices; no general matrix or archived runtime qualification. Generic same-scalar oracles are not independent scalar backends. Hyper determinant construction and decidable equality are separate capabilities. Standalone Faddeev/Berkowitz probes are not production algorithms or matched benchmarks. Berkowitz gains eight comparisons but loses two: not selected as a direct replacement; alternate scheduling remains open. Nine pre-existing Hypercurve ignored tests remain unrun.'};
const files=['bind-matrix-pivots.mjs','verify-matrix-pivots.mjs','capture.mjs','record-matrix-pivot-reads.mjs',
  'matrix-pivot-read-selection.json','flint-matrix-pivot-probe.c','matrix-pivot-hyper.rs','matrix-methods-probe.rs',
  'monic-state-experiment.json','retained-polynomial-facts.json'];
for(const pkg of ['matrix-pivot-hyper','matrix-methods-probe'])for(const file of ['Cargo.toml','Cargo.lock'])files.push(`${pkg}/${file}`);
const tags=['matrix-pivot-native-compile','matrix-pivot-native','matrix-pivot-native-memcheck',
  'matrix-pivot-hyper-debug','matrix-pivot-hyper-release','matrix-pivot-hyper-memcheck','matrix-pivot-donor-tests',
  'matrix-methods-debug','matrix-methods-release','matrix-methods-memcheck','monic-qualified-consumer-hypercurve-debug'];
for(const tag of tags){const g=json(`results/${tag}.json`);assert.equal(g.code,0,tag);assert.equal(g.signal,null);
  manifest.gates.push(tag);for(const ext of ['json','stdout','stderr'])files.push(`results/${tag}.${ext}`);}
for(const p of files)manifest.files[p]=sha(p);
const coverage=json('coverage.json');
manifest.reads=json('matrix-pivot-read-selection.json').map(e=>{const v=coverage.find(c=>c.repo===e.repo&&c.path===e.path);assert(v);return v;});
assert.equal(manifest.reads.length,47);
const root='/tmp/calcium-matrix-pivots.X3ulwj';
manifest.binaries[`${root}/flint-matrix-pivots`]=sha(`${root}/flint-matrix-pivots`);
for(const binary of ['calcium-matrix-pivot-hyper','calcium-matrix-methods-probe'])for(const profile of ['debug','release']){
  const original=`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/${profile}/${binary}`;
  const path=draft?original:`${root}/${binary}-${profile}`;
  if(!draft)copyFileSync(original,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  assert.equal(sha(path),sha(original));manifest.binaries[path]=sha(path);
}
const original=resolve(workspace,'exact-real-references/flint/build/ca_mat/test/main');
const path=draft?original:`${root}/flint-ca-mat-tests`;
if(!draft)copyFileSync(original,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
assert.equal(sha(path),sha(original));manifest.binaries[path]=sha(path);
if(!draft)writeFileSync(resolve(here,'matrix-pivot-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'matrix pivot binding',draft,files:files.length,gates:tags.length,
  reads:manifest.reads.length,binaries:Object.keys(manifest.binaries).length,
  binaryBytes:Object.keys(manifest.binaries).reduce((n,p)=>n+statSync(p).size,0)}));
export {manifest};
