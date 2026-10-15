"""Trace the next strict failure with the candidate Cargo dependency graph."""
from pathlib import Path
import json, hashlib, shutil, difflib
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
CORE = Path('/tmp/csgrs-strict-drilled-workflows/candidate-v1/release')
TARGET = ROOT / 'target/strict-drilled-workflows/diagnostic-v3'
SOURCE = Path('/tmp/csgrs-strict-drilled-workflows/diagnostic-v3-source/hypermesh/src')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert not TARGET.exists() and not SOURCE.exists()
shutil.copytree(ROOT / 'crates/hypermesh/src', SOURCE)
p = SOURCE / 'predicate.rs'
before = p.read_text()
after = Path('/tmp/csgrs-strict-drilled-workflows/diagnostic-v1/hypermesh/src/predicate.rs').read_text().replace('eprintln!("unresolved_scalar={}", value.to_json());', 'eprintln!("unresolved_scalar_debug={value:?}");')
p.write_text(after)
(E / 'diagnostic-v3.patch').write_text(''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile='crates/hypermesh/src/predicate.rs', tofile='diagnostic-v3-source/hypermesh/src/predicate.rs')))


def library(name):
    matches = list((CORE / 'deps').glob(f'lib{name}-*.rlib'))
    if name == 'hyperreal':
        matches = [p for p in matches if 'cached-f64-approx' in json.loads((CORE/'.fingerprint'/p.stem[3:]/'lib-hyperreal.json').read_text())['features']]
    assert len(matches) == 1, (name, matches)
    return matches[0]


def build(name, source, dependencies):
    cmd = ['rustc','--crate-name',name,'--crate-type','rlib','--edition=2024','-C','opt-level=3','-C','debuginfo=1','-C','force-frame-pointers=yes','-D','warnings',str(source),'-L',f'dependency={CORE}/deps','-L',f'dependency={TARGET}','-o',str(TARGET/f'lib{name}.rlib')]
    if name == 'csgrs': cmd += ['--cfg','feature="curve"']
    for dep in dependencies:
        path = TARGET/f'lib{dep}.rlib' if dep in ['hypermesh','hyperphysics'] else library(dep)
        cmd += ['--extern',f'{dep}={path}']
    return cmd


commands = [['prepare-target',['mkdir',str(TARGET)]],
 ['native-build',build('hypermesh',SOURCE/'lib.rs',['hyperreal','hyperlattice','hyperlimit','hypertri'])],
 ['physics-build',build('hyperphysics',ROOT/'crates/hyperphysics/src/lib.rs',['hyperreal','hyperlattice','hyperlimit','hypersolve','hypermesh'])],
 ['root-build',build('csgrs',ROOT/'src/lib.rs',['hyperreal','hyperlattice','hyperlimit','hypertri','hypercurve','hypermesh','hyperphysics','thiserror'])],
 ['probe-build',['rustc','--edition=2024','-C','opt-level=3','-C','debuginfo=1','-C','force-frame-pointers=yes','-D','warnings','--extern',f'csgrs={TARGET}/libcsgrs.rlib','-L',f'dependency={CORE}/deps','-L',f'dependency={TARGET}',str(E/'drill_probe_v1.rs'),'-o',str(TARGET/'probe')]],
 ['drill-16-strict',[str(TARGET/'probe'),'16','strict']],
 ['drill-32-strict',[str(TARGET/'probe'),'32','strict']]]
keep = json.loads((E/'baseline-closure-v1.json').read_text())['artifacts_sha256']
for p in [*CORE.rglob('*.rlib'), *CORE.rglob('*.so'), *SOURCE.rglob('*.rs'), E/'drill_probe_v1.rs', Path(__file__)]: keep[str(p)] = sha(p)
for suffix, value in [('.json',commands),('-target.json',str(TARGET.relative_to(ROOT))),('-seed.json',None),('-preserve.json',keep)]:
 p=E/('diagnostic-v3'+suffix);assert not p.exists();p.write_text(json.dumps(value,indent=2)+'\n')
