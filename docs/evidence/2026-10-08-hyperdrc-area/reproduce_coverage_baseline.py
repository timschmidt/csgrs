"""Replay the retained staged coverage failures in a clean base-revision checkout.
Usage: python3 reproduce_coverage_baseline.py /path/to/clean/checkout /tmp/area-coverage-baseline
"""
import hashlib,json,pathlib,re,subprocess,sys
record=pathlib.Path(__file__).resolve().parent / 'coverage-baseline'
manifest=json.loads((record/'manifest.json').read_text())
checkout=pathlib.Path(sys.argv[1]).resolve();out=pathlib.Path(sys.argv[2]).resolve()
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=checkout,text=True).strip()==manifest['base_revision']
assert not subprocess.check_output(['git','status','--porcelain'],cwd=checkout,text=True).strip()
out.mkdir(parents=True,exist_ok=False)
patch=(record/'stage.patch').read_bytes()
paths=re.findall(r'^\+\+\+ b/(.+)$',patch.decode(),re.M)
saved={checkout/p:(checkout/p).read_bytes() for p in paths}
tests=checkout/'crates/hyperdrc/src/region/area_regressions.rs';assert not tests.exists()
try:
 subprocess.run(['git','apply','-'],cwd=checkout,input=patch,check=True)
 tests.write_bytes((record/'regressions.rs').read_bytes())
 for p,h in manifest['source_hashes'].items():assert hashlib.sha256((checkout/p).read_bytes()).hexdigest()==h,p
 with (out/'test.log').open('w') as log:result=subprocess.run(manifest['command'],cwd=checkout,stdout=log,stderr=subprocess.STDOUT)
 assert result.returncode==101
 assert '10 passed; 4 failed;' in (out/'test.log').read_text()
finally:
 for p,data in saved.items():p.write_bytes(data)
 tests.unlink(missing_ok=True)
print('Reproduced four staged coverage failures; checkout restored.')
