// Recheck hashes, exact corpus membership and identical completed test sets.
import './verify-root-exp-sign-checkpoint.mjs';
import { readFileSync, realpathSync, readdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = json('sign-consumer-experiment.json');
for (const [p, expected] of Object.entries({ ...manifest.sourceHashes, ...manifest.evidenceHashes }))
  assert.equal(hash(p), expected, p);
function capture(tag, code=0) {
  const r = json(`results/${tag}.json`);
  assert.equal(r.code, code, tag); assert.equal(r.signal, null, tag);
  return read(`results/${tag}.stdout`);
}
const snapshot = json('sign-consumers.json');
assert.equal(snapshot.crates.length, 5);
assert.equal(manifest.filesPerVariant, 773); assert.equal(manifest.bytesPerVariant, 40117830);
function filesBelow(root) {
  const paths = [];
  function walk(dir) { for (const e of readdirSync(dir, { withFileTypes: true })) {
    const p = resolve(dir, e.name);
    if (e.isDirectory()) walk(p); else { assert(e.isFile(), p); paths.push(relative(root, p)); }
  } }
  walk(root); return paths.sort();
}
for (const v of ['baseline', 'sign']) {
  const root = `sign-consumers/${v}`;
  assert.equal(realpathSync(resolve(here, root, 'hyperreal')), snapshot.scalarSources[v]);
  for (const crate of snapshot.crates) {
    const dir = resolve(here, root, crate.crate);
    assert.deepEqual(filesBelow(dir), crate.files.map(f => f.path).sort());
    for (const f of crate.files) {
      assert.equal(hash(`${root}/${crate.crate}/${f.path}`), f.sha256, `${v}/${crate.crate}/${f.path}`);
      assert.equal(readFileSync(resolve(dir, f.path)).length, f.bytes);
    }
  }
  const metadata = JSON.parse(capture(`sign-consumer-metadata-all-features-${v}`));
  for (const name of ['hyperreal', ...snapshot.crates.map(c => c.crate)]) {
    const packages = metadata.packages.filter(p => p.name === name);
    assert.equal(packages.length, 1, `${v}/${name}`);
    assert.equal(realpathSync(packages[0].manifest_path), realpathSync(resolve(here, root, name, 'Cargo.toml')));
  }
  const failedBuild = capture(`sign-consumer-probe-${v}-build`, 101);
  assert.equal(failedBuild, '');
  assert(read(`results/sign-consumer-probe-${v}-build.stderr`).includes('package collision in the lockfile'));
  capture(`sign-consumer-probe-${v}-build-unified-path`);
  capture(`sign-consumer-hyperlattice-${v}-release`, 101);
  assert(read(`results/sign-consumer-hyperlattice-${v}-release.stderr`).includes('ccache: error: Read-only file system'));
}
function suites(text) {
  const result = []; let current;
  for (const line of text.split('\n')) {
    const start = /^running (\d+) tests?$/.exec(line);
    if (start) { assert(!current); current = { expected: +start[1], names: [] }; }
    const test = /^test (.+) \.\.\. (ok|ignored)(?:,.*)?$/.exec(line);
    if (test) { assert(current); current.names.push([test[1], test[2]]); }
    const end = /^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/.exec(line);
    if (end) {
      assert(current); const [passed, failed, ignored, measured, filtered] = end.slice(1).map(Number);
      assert.equal(failed, 0); assert.equal(measured, 0); assert.equal(filtered, 0);
      assert.equal(current.expected, passed+ignored); assert.equal(current.names.length, current.expected);
      assert.equal(current.names.filter(n=>n[1]==='ok').length, passed);
      assert.equal(new Set(current.names.map(n=>n[0])).size, current.expected);
      result.push({ passed, ignored, names: current.names.sort((a,b)=>a[0].localeCompare(b[0])) }); current=undefined;
    }
  }
  assert(!current); assert(result.length); return result;
}
const totals = { hyperlimit: 361, hypersolve: 797, hypertri: 187, hyperlattice: 203 };
for (const [crate, total] of Object.entries(totals)) {
  const gates = manifest.gates.filter(g=>g.crate===crate);
  assert.equal(gates.length, 2); const results = gates.map(g=>suites(capture(g.tag)));
  assert.deepEqual(results[0], results[1], `${crate} test membership changed`);
  assert.equal(results[0].reduce((n,s)=>n+s.passed,0), total);
  assert.equal(results[0].reduce((n,s)=>n+s.ignored,0), 0);
}
function consumerCorpus(tag, variant) {
  const rows = capture(tag).trim().split('\n');
  assert.equal(rows.shift(), 'case,delta,reversed,consumer,result,stage');
  const map = new Map(rows.map(r=>r.split(',')).map(r=>[r.slice(0,4).join(','),r]));
  assert.equal(rows.length,224); assert.equal(map.size,224); let decided=0, beyond=0;
  for (let c=0;c<4;c++) for (const delta of [0,-767,767,-999,999,-2048,2048])
    for (const reversed of [false,true]) for (const consumer of ['sign','ordering','orientation','solver']) {
      const r=map.get([c,delta,reversed,consumer].join(',')); assert(r);
      const known = variant==='baseline' ? c===0 && delta===0 : Math.abs(delta)<=999;
      const direction = delta===0 ? 0 : (delta<0)!==reversed ? 1 : -1;
      const value = !known ? 'Unknown' : consumer==='ordering' ? ['Less','Equal','Greater'][direction+1]
        : consumer==='solver' ? ['Violated','Boundary','Satisfied'][direction+1] : ['Negative','Zero','Positive'][direction+1];
      assert.equal(r[4],value,tag); decided+=known;
      if (Math.abs(delta)===2048) { assert.equal(value,'Unknown'); beyond++; }
      const stage = consumer==='solver' ? 'Structural' : !known ? 'Undecided'
        : consumer==='orientation' && c===0 && delta===0 ? 'Exact' : 'Structural';
      assert.equal(r[5],stage,tag);
    }
  assert.equal(decided,variant==='baseline'?8:160); assert.equal(beyond,64);
}
consumerCorpus('sign-consumer-probe-baseline-release','baseline');
consumerCorpus('sign-consumer-probe-sign-release','sign');
consumerCorpus('sign-consumer-probe-sign-memcheck','sign');
const mem = read('results/sign-consumer-probe-sign-memcheck.stderr');
for (const s of ['ERROR SUMMARY: 0 errors','definitely lost: 0 bytes','indirectly lost: 0 bytes','possibly lost: 0 bytes']) assert(mem.includes(s));
capture('expression-roundtrip-compile',1);
assert(read('results/expression-roundtrip-compile.stderr').includes('Disk quota exceeded'));
const unbuilt=json('results/expression-roundtrip-native.json');
assert.equal(unbuilt.code,null); assert(unbuilt.error.includes('ENOENT'));
capture('expression-roundtrip-compile-workspace-tmp');
const expression=capture('expression-roundtrip-native-built',1);
assert.equal(expression,capture('expression-roundtrip-memcheck',1));
let lines=expression.trim().split('\n');
assert.equal(lines.shift(),'input,operation,serialization,source_unknown,parsed,restored_unknown,equality');
assert.deepEqual(JSON.parse(lines.pop()),{suite:'expression-roundtrip',cases:48,parse_failed:0,lost_value:6,unequal:0});
let map=new Map(lines.map(l=>l.split(',')).map(r=>[r.slice(0,3).join(','),r]));
assert.equal(lines.length,48); assert.equal(map.size,48);
for (const input of ['two-thirds','sqrt-two','pi','one-plus-i','pi-plus-i','one-plus-pi-i'])
  for (const op of ['identity','arg','csgn','exp']) for (const mode of [0,1]) {
    const r=map.get([input,op,mode].join(',')); assert(r);
    const lost=op==='arg' && ['pi','pi-plus-i','one-plus-pi-i'].includes(input);
    assert.deepEqual(r.slice(3),['0','1',lost?'1':'0',lost?'Unknown':'Equal']);
  }
capture('factor-association-compile');
const factors=capture('factor-association-native',1);
assert.equal(factors,capture('factor-association-memcheck',1));
lines=factors.trim().split('\n'); assert.equal(lines.shift(),'permutation,index,delta,initial_equal,updated_equal');
assert.deepEqual(JSON.parse(lines.pop()),{suite:'factor-association',cases:90,initial_failed:0,updated_failed:48});
map=new Map(lines.map(l=>l.split(',')).map(r=>[r.slice(0,3).join(','),r]));
assert.equal(lines.length,90); assert.equal(map.size,90);
for (let p=0;p<6;p++) for(let i=0;i<3;i++) for(let d=-2;d<=2;d++)
  assert.deepEqual(map.get([p,i,d].join(','))?.slice(3),['1',i===0 || d===0?'1':'0']);
for (const tag of ['expression-roundtrip-memcheck','factor-association-memcheck']) {
  const text=read(`results/${tag}.stderr`);
  assert(text.includes('ERROR SUMMARY: 0 errors') && text.includes('All heap blocks were freed'));
}
const coverage=json('coverage.json'),inventory=json('inventory.json');
for(const source of inventory.sources) for(const stem of ['set_fexpr','get_fexpr','clear','set','transfer','hash_repr',
  'factor_init','factor_clear','factor_one','factor_insert','factor_get_ca','is_gen_as_ext','is_cyclotomic_nf_elem',
  'ctx_get_field_const','ctx_get_field_fx','ctx_get_field_fxy','check_is_number','check_is_imaginary',
  'check_is_negative_real','check_is_one','check_is_neg_one','check_is_i','check_is_neg_i']) {
  const path=`${source.repo==='flint'?'src/':''}ca/${stem}.c`;
  const file=source.files.find(f=>f.path===path);
  assert.deepEqual(coverage.find(c=>c.repo===source.repo && c.path===path)?.ranges,[[1,file.lines]]);
}
console.log(JSON.stringify({checkpoint:'sign-consumers',pairedGates:4,testsPerVariant:1548,
  filesPerVariant:773,consumerQueries:224,newExactDecisions:152,roundTripLosses:6,factorFailures:48,
  hypercurve:'ongoing; not bound by this checkpoint',status:'verified partial qualification; no production transfer'}));
