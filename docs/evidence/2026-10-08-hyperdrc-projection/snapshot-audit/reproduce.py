"""Replay all changed snapshot identities against the pinned old snapshots."""
import json,pathlib,subprocess,sys
root=pathlib.Path.cwd();source=pathlib.Path(__file__).parent
out=pathlib.Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
baseline=out/'baseline';baseline.mkdir()
revision='d171b377755dcbd6b1a1dfe371f1739bee8ee52c'
for slug in ['nano','uno','esp32','esp32s3','rp2040','stm32f103']:
 path=f'crates/hypercircuit/tests/fixtures/easyduino/fast-drc/{slug}.json'
 (baseline/f'{slug}.json').write_bytes(subprocess.check_output(['git','show',revision+':'+path]))
cmd=['cargo','build','--locked','--release','-p','hypercircuit','--all-features','--lib','--message-format=json']
with (out/'build.jsonl').open('w') as stdout,(out/'build.log').open('w') as stderr:subprocess.run(cmd,stdout=stdout,stderr=stderr,check=True)
names={'hypercircuit','hyperdrc','hyperlimit','hypercurve','serde_json'};libs={}
for line in (out/'build.jsonl').read_text().splitlines():
 x=json.loads(line)
 if x.get('reason')=='compiler-artifact' and x['target']['name'] in names:
  for p in x['filenames']:
   if p.endswith('.rlib'):libs[x['target']['name']]=p
assert libs.keys()==names
cmd=['rustc','--edition=2024','-O',str(source/'audit.rs'),'-L',f'dependency={root / "target/release/deps"}','-o',str(out/'audit')]
for name,path in sorted(libs.items()):cmd+=['--extern',name+'='+path]
subprocess.run(cmd,check=True)
subprocess.run([str(out/'audit'),str(out/'details'),str(baseline)],check=True)
for p in (out/'details').glob('*-audit.json'):
 x=json.loads(p.read_text());assert not x['unmatched'];groups={}
 for row in x['rows']:groups.setdefault(row['evidence_id'],[]).append(row)
 for rows in groups.values():
  expected=rows[0]['change']=='added'
  assert any(t['native_below'] for row in rows for t in row['thresholds'])==expected
  assert any(t['chords_below'] for row in rows for t in row['thresholds'])!=expected
print('Every changed finding identity agrees with the unpruned native oracle and disagrees with the report-chord reference.')
