from pathlib import Path
import hashlib, io, json, os, shutil, subprocess, tarfile, time

w = Path('/home/tim/Documents/GitHub/workspace')
a = w / 'hypercurve-api-review-2026-09-12'
r = Path('/tmp/hypercurve-mapped-point-qualification')
r.mkdir(exist_ok=True)
for name in ['hypercurve', 'hyperbrep']:
    destination = r / name
    destination.mkdir(exist_ok=True)
    archive = subprocess.check_output(['git', 'archive', 'HEAD'], cwd=w/name)
    with tarfile.open(fileobj=io.BytesIO(archive)) as source:
        source.extractall(destination, filter='data')
for name in ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypersolve', 'hypertri']:
    if not (r/name).is_symlink():
        (r/name).symlink_to(Path('/tmp/hypercurve-pruning-qualification')/name)
source = r/'hypercurve/src/bezier_offset.rs'
text = source.read_text()
marker = '    fn finite_circle_component_inverse_charts('
assert text.count(marker) == 1
source.write_text(text.replace(marker, (a/'finite-point-inverse-regression.rs').read_text()+'\n'+marker))
artifacts = a/'mapped-point-inverse-baseline-artifacts'
artifacts.mkdir()
shutil.copy2(source, artifacts/source.name)
settings = json.loads((a/'finite-point-inverse-domains-build-settings.json').read_text())
env = dict(os.environ, **settings)
p = 'mapped-point-inverse-baseline'
cmd = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'test', '--release', '--all-features', '--locked', '--offline', '--lib', '--no-run', '--message-format=json']
start = time.monotonic()
with (a/(p+'-build.log')).open('w') as err, (a/(p+'-build.jsonl')).open('w') as out:
    build = subprocess.run(cmd, cwd=r/'hypercurve', env=env, stdout=out, stderr=err, timeout=500)
record = dict(command=cmd, returncode=build.returncode, elapsed_seconds=time.monotonic()-start)
(a/(p+'-build.json')).write_text(json.dumps(record, indent=2)+'\n')
assert build.returncode == 0, (a/(p+'-build.log')).read_text()[-6000:]
entries = [json.loads(line) for line in (a/(p+'-build.jsonl')).read_text().splitlines()]
bins = [entry['executable'] for entry in entries if entry.get('executable') and entry.get('target', {}).get('kind') == ['lib']]
assert len(bins) == 1, bins
binary = Path(bins[0])
shutil.copy2(binary, artifacts/binary.name)
cmd = [str(binary), 'finite_point_inverse_replays_nonrepresented_center_cuts', '--nocapture', '--test-threads=1']
start = time.monotonic()
with (a/(p+'.log')).open('w') as out:
    result = subprocess.run(['timeout', '--kill-after=5', '120', *cmd], env=env, stdout=out, stderr=subprocess.STDOUT)
record = dict(command=cmd, returncode=result.returncode, elapsed_seconds=time.monotonic()-start,
              parent=subprocess.check_output(['git','rev-parse','HEAD'],cwd=w/'hypercurve',text=True).strip(),
              source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), binary=str(binary))
(a/(p+'.json')).write_text(json.dumps(record, indent=2)+'\n')
print(json.dumps(record, indent=2), flush=True)
print((a/(p+'.log')).read_text()[-6000:])
