"""Instrument source copies and rebuild the complete selected local crate graph."""
from pathlib import Path
import difflib, hashlib, json, shutil, tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREVIOUS = ROOT/'docs/evidence/2026-10-15-shared-scalar-proofs'
CORE = Path('/run/user/1000/csgrs-shared-scalar-proofs/scalar-and-drill-v1/release')
TARGET = Path('/home/tim/Documents/GitHub/workspace/csgrs/target/properties/polynomial-quotient-reduction/diagnostic-v2')
SOURCE = Path('/home/tim/Documents/GitHub/workspace/csgrs/target/properties/polynomial-quotient-reduction/diagnostic-v2-source')
PROBE = E/'composition_probe_v1.rs'
EXTERNAL = Path('/run/user/1000/csgrs-quotient-factor-proofs/diagnostic-v3/external')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
assert not SOURCE.exists() and not TARGET.exists()
assert json.loads((PREVIOUS/'scalar-and-drill-v1/result.json').read_text())['passed']
for name in ['hyperreal', 'hypermesh']:
    shutil.copytree(ROOT/'crates'/name/'src', SOURCE/name/'src')
patches = []


def change(name, transform):
    source = SOURCE/name
    before = source.read_text()
    after = transform(before)
    assert before != after, name
    source.write_text(after)
    patches.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
        fromfile='crates/'+name, tofile='diagnostic-v2-source/'+name))


def ring(text):
    text = text.replace("    atoms: Vec<&'a Computable>,", "    atoms: Vec<&'a Computable>,\n    diagnose: bool,")
    text = text.replace('            atoms: Vec::new(),', '            atoms: Vec::new(),\n            diagnose: false,')
    text = text.replace('        self.operations_left = self.operations_left.checked_sub(1)?;', '''        if self.operations_left == 0 && self.diagnose { eprintln!("quotient_decline=work"); }
        self.operations_left = self.operations_left.checked_sub(1)?;''')
    text = text.replace('            return None;\n        }\n        self.operations_left', '            if self.diagnose { eprintln!("quotient_decline=cancelled"); }\n            return None;\n        }\n        self.operations_left')
    text = text.replace('if !Self::coefficient_fits(&coefficient) || monomial.len() > Self::MAX_DEGREE {', 'if !Self::coefficient_fits(&coefficient) || monomial.len() > Self::MAX_DEGREE {\n            if self.diagnose { eprintln!("quotient_decline=insert-coefficient-or-degree degree={}", monomial.len()); }')
    text = text.replace('if !Self::coefficient_fits(&sum) {', 'if !Self::coefficient_fits(&sum) {\n                    if self.diagnose { eprintln!("quotient_decline=coefficient-sum"); }')
    text = text.replace('if result.terms.len() == Self::MAX_TERMS {', 'if result.terms.len() == Self::MAX_TERMS {\n                    if self.diagnose { eprintln!("quotient_decline=terms"); }')
    text = text.replace('if a.len() + b.len() > Self::MAX_DEGREE {', 'if a.len() + b.len() > Self::MAX_DEGREE {\n                    if self.diagnose { eprintln!("quotient_decline=product-degree left={} right={}", a.len(), b.len()); }')
    text = text.replace('if self.atoms.len() == Self::MAX_ATOMS {', 'if self.atoms.len() == Self::MAX_ATOMS {\n            if self.diagnose { eprintln!("quotient_decline=atoms"); }')
    return limits(text)


def limits(text):
    text = text.replace('if depth == 0 {', 'if depth == 0 {\n            if self.diagnose { eprintln!("quotient_decline=depth"); }')
    text = text.replace('        self.nodes_left = self.nodes_left.checked_sub(1)?;', '''        if self.nodes_left == 0 && self.diagnose { eprintln!("quotient_decline=nodes"); }
        self.nodes_left = self.nodes_left.checked_sub(1)?;''')
    text = text.replace('if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {', 'if u64::from(shift.unsigned_abs()) >= Self::MAX_COEFFICIENT_BITS {\n                    if self.diagnose { eprintln!("quotient_decline=shift"); }')
    return text


change('hyperreal/src/computable/node/ring_polynomial.rs', ring)
change('hyperreal/src/computable/node/ring_quotient.rs', lambda text: limits(text) + '''
impl Computable {
    pub(crate) fn diagnostic_quotient_limits(&self) {
        let mut recognition = RingRecognition::new(self);
        recognition.diagnose = true;
        let expanded = recognition.expand_quotient(self, RingRecognition::MAX_DEPTH);
        let rational = expanded.as_ref().and_then(|q| recognition.project_quotient(q));
        eprintln!("quotient_diagnostic expanded={} rational={rational:?} nodes_left={} work_left={} atoms={} cached={}",
            expanded.is_some(), recognition.nodes_left, recognition.operations_left,
            recognition.atoms.len(), recognition.quotients.len());
    }
}
''')
change('hyperreal/src/real/arithmetic/facts.rs', lambda text: text + '''
impl Real {
    /// Diagnostic-only copied-source query; it does not publish a root proof.
    #[doc(hidden)]
    pub fn diagnostic_quotient_limits(&self) {
        self.computable_ref().diagnostic_quotient_limits();
    }
}
''')


def native(text):
    needle = '            classify_real_sign(value, decisions.policy()),'
    assert text.count(needle) == 1
    return text.replace(needle, '''            {
                let outcome = classify_real_sign(value, decisions.policy());
                if matches!(outcome, hyperlimit::PredicateOutcome::Unknown { .. }) {
                    eprintln!("unresolved_composition_scalar={value:?}");
                    value.diagnostic_quotient_limits();
                    eprintln!("{}", std::backtrace::Backtrace::force_capture());
                }
                outcome
            },''')


change('hypermesh/src/predicate.rs', native)
(E/'diagnostic-v2.patch').write_text(''.join(patches))
root_files = list(CORE.glob('.fingerprint/csgrs-*/lib-csgrs.json'))
assert len(root_files) == 1
root_meta = json.loads(root_files[0].read_text())
assert json.loads(root_meta['features']) == ['curve']
mapping = {}
workspace = {}
order = []


def library(name, expected):
    key = f'{name}:{expected}'
    if key in mapping: return mapping[key]
    candidates = [p for p in (CORE/'.fingerprint').glob('*/lib-'+name)
                  if int.from_bytes(bytes.fromhex(p.read_text().strip()), 'little') == expected]
    assert len(candidates) == 1, (name, expected, candidates)
    fingerprint = candidates[0]
    suffix = fingerprint.parent.name.rsplit('-', 1)[1]
    paths = [CORE/'deps'/f'lib{name}-{suffix}{ext}' for ext in ['.rlib', '.so']]
    paths = [p for p in paths if p.exists()]
    assert len(paths) == 1, (name, paths)
    record = {'path': str(paths[0]), 'sha256': sha(paths[0]), 'cargo_fingerprint': expected,
              'fingerprint_json': str(fingerprint.with_suffix('.json')),
              'metadata': json.loads(fingerprint.with_suffix('.json').read_text())}
    mapping[key] = record
    return record


def visit(name, metadata):
    if name in workspace:
        assert workspace[name] == metadata
        return
    workspace[name] = metadata
    for _, dependency, _, expected in metadata['deps']:
        record = library(dependency, expected)
        if (ROOT/'crates'/dependency/'Cargo.toml').exists():
            visit(dependency, record['metadata'])
    order.append(name)


visit('csgrs', root_meta)
assert set(order) == {'hyperreal', 'hyperlimit', 'hyperlattice', 'hypersolve', 'hypertri', 'hypermesh', 'hyperphysics', 'hypercurve', 'csgrs'}, order
commands = [['prepare-target', ['mkdir', str(TARGET)]]]
for name in order:
    crate = ROOT if name == 'csgrs' else ROOT/'crates'/name
    manifest = tomllib.loads((crate/'Cargo.toml').read_text())['package']
    source = SOURCE/name/'src/lib.rs' if name in ['hyperreal', 'hypermesh'] else crate/'src/lib.rs'
    command = ['env', 'TMPDIR='+str(TARGET), 'CARGO_PKG_NAME='+name, 'CARGO_PKG_VERSION='+manifest['version'],
               'CARGO_MANIFEST_DIR='+str(crate), 'rustc', '--crate-name', name, '--crate-type', 'rlib',
               '--edition='+manifest['edition'], '-C', 'opt-level=3', '-C', 'debuginfo=1',
               '-C', 'force-frame-pointers=yes', '-D', 'warnings', str(source),
               '-L', f'dependency={EXTERNAL}', '-L', f'dependency={TARGET}', '-o', str(TARGET/f'lib{name}.rlib')]
    for feature in json.loads(workspace[name]['features']):
        command += ['--cfg', 'feature='+json.dumps(feature)]
    if name == 'hyperreal':
        command += ['--check-cfg', 'cfg(test)', '--check-cfg', 'cfg(verus_keep_ghost)',
                    '--check-cfg', 'cfg(feature, values('+','.join(json.dumps(f) for f in json.loads(workspace[name]['declared_features']))+'))']
    for _, dependency, _, expected in workspace[name]['deps']:
        path = str(TARGET/f'lib{dependency}.rlib') if dependency in workspace else library(dependency, expected)['path']
        command += ['--extern', dependency+'='+path]
    commands.append([name+'-build', command])
commands += [['probe-build', ['rustc', '--edition=2024', '-C', 'opt-level=3', '-C', 'debuginfo=1',
                             '-D', 'warnings', '--extern', f'csgrs={TARGET}/libcsgrs.rlib',
                             '--extern', f'hypermesh={TARGET}/libhypermesh.rlib',
                             '-L', f'dependency={EXTERNAL}', '-L', f'dependency={TARGET}',
                             str(PROBE), '-o', str(TARGET/'probe')]]]
commands += [[f'compose-{n}-{mode}', ['env', 'TMPDIR='+str(TARGET), 'timeout', '120', str(TARGET/'probe'), str(n), mode]]
             for mode, counts in [('chain', [8, 16, 24, 32]), ('direct', [16, 32])] for n in counts]
keep = {}  # Historical external losses are recorded separately, never declared preserved.
keep[str(E/'external-artifact-loss-v1.json')] = sha(E/'external-artifact-loss-v1.json')
for p in [*EXTERNAL.iterdir(), *CORE.rglob('*.rlib'), *CORE.rglob('*.so'), *SOURCE.rglob('*.rs'),
          PROBE, Path(__file__), *CORE.glob('.fingerprint/*/lib-*.json')]:
    keep[str(p)] = sha(p)
for suffix, value in [('.json', commands), ('-seed.json', None), ('-target.json', str(TARGET)), ('-preserve.json', keep)]:
    p = E/('diagnostic-v2'+suffix); assert not p.exists()
    p.write_text(json.dumps(value, indent=2)+'\n')
(E/'diagnostic-dependencies-v1.json').write_text(json.dumps(mapping, indent=2)+'\n')
print(json.dumps({'build_order': order, 'libraries_pinned': len(mapping), 'commands': len(commands)}))
