from pathlib import Path
import concurrent.futures
import hashlib
import json
import os
import shutil
import subprocess
import time

a = Path(__file__).resolve().parent
w = a.parent / 'hypercurve'
r = Path('/tmp/hypercurve-region-admission-qualification/hypercurve')
prefix = 'normalized-corner-publication-diagnostic1'
files = ['src/bezier_region.rs', 'src/curve_region_boolean.rs', 'tests/hypercurve_curve_region_promotion.rs']
working = {name: hashlib.sha256((w/name).read_bytes()).hexdigest() for name in [*files, 'src/bezier_offset.rs']}
for name in files:
    (r/name).write_bytes((w/name).read_bytes())

def substitute(text, before, after, count=1):
    assert text.count(before) == count, (before[:100], text.count(before), count)
    return text.replace(before, after)

file = r/'src/curve_region_boolean.rs'
s = file.read_text()
s = substitute(s, '/// Region operand that owns one retained Boolean carrier.', '''macro_rules! corner_trace {
    ($($args:tt)*) => {
        if std::env::var_os("HYPERCURVE_CORNER_DIAGNOSTIC").is_some() {
            eprintln!($($args)*);
        }
    };
}

/// Region operand that owns one retained Boolean carrier.''')
s = substitute(s, '        self.resolve_regularization(policy, || {', '''        corner_trace!("normalize enter loops={} fragments={:?}", self.len(), self.boundary_loops().iter().map(|b|b.len()).collect::<Vec<_>>());
        self.resolve_regularization(policy, || {''')
s = substitute(s, '        let pairs = build_unary_carrier_pairs(&carriers, policy)?;', '''        corner_trace!("schedule enter carriers={}", carriers.len());
        let pairs = build_unary_carrier_pairs(&carriers, policy)?;
        corner_trace!("schedule done pairs={}", pairs.len());''')
s = substitute(s, '            let result = self.pair_result(pair)?;', '''            corner_trace!("pair enter {} {} kind={:?}", pair.first_carrier_index, pair.second_carrier_index, std::mem::discriminant(&pair.context));
            let result = self.pair_result(pair)?;
            corner_trace!("pair done {} {} contacts={} overlaps={} blockers={}", pair.first_carrier_index, pair.second_carrier_index, result.contacts.len(), result.overlaps.len(), result.blockers.len());''', s.count('            let result = self.pair_result(pair)?;'))
s = substitute(s, '            .map(|(carrier_index, carrier)| {\n                split_carrier(', '''            .map(|(carrier_index, carrier)| {
                corner_trace!("split enter carrier={} events={}", carrier_index, events[carrier_index].len());
                split_carrier(''')
s = substitute(s, '        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);', '''        corner_trace!("split topology done");
        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);
        corner_trace!("fragment selection enter simple_side={:?}", simple_loop_filled_side);''')
s = substitute(s, '        let mut arrangement_fragments = Vec::new();', '''        corner_trace!("fragment selection done");
        let mut arrangement_fragments = Vec::new();''', s.count('        let mut arrangement_fragments = Vec::new();'))
s = substitute(s, '        // The pair kernel has already certified the only nonendpoint event', '''        corner_trace!("cusp strict-interior ordering shortcut carrier={}/{} reversed={} start={} cut={} end={}", carrier.loop_index, carrier.fragment_index, carrier.reversed, start, interior, end);
        // The pair kernel has already certified the only nonendpoint event''')
s = substitute(s, '''                return Err(CurveError::Topology(
                    "algebraic cusp split boundaries are not increasing".into(),''', '''                corner_trace!("cusp inconsistent order carrier={}/{} reversed={}", carrier.loop_index, carrier.fragment_index, carrier.reversed);
                for (label, parameter) in [("carrier-start", &carrier.start), ("carrier-end", &carrier.end), ("cut-before", &pair[0].parameter), ("cut-after", &pair[1].parameter)] {
                    if let Some(parameter) = parameter.as_algebraic_cusp() {
                        let kind = match parameter {
                            BezierAlgebraicCuspSemicircleParameter2::Exact(value) => format!("Exact({value:?})"),
                            BezierAlgebraicCuspSemicircleParameter2::Mapped(data) => format!("Mapped({:?})", std::mem::discriminant(data.as_ref())),
                        };
                        corner_trace!("cusp {label}: kind={kind} bracket={:?}", parameter.parameter_bracket(4, policy));
                    }
                }
                return Err(CurveError::Topology(
                    "algebraic cusp split boundaries are not increasing".into(),''')
file.write_text(s)
file = r/'src/bezier_region.rs'
s = file.read_text()
s = substitute(s, '        let edited = Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)', '''        if std::env::var_os("HYPERCURVE_CORNER_DIAGNOSTIC").is_some() {
            eprintln!("corner candidate enter operation={operation:?} loop={loop_index} fragments={}", loops[loop_index].len());
        }
        let edited = Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)''')
file.write_text(s)

manifest = []
for repo in sorted(r.parent.iterdir()):
    if repo.is_dir():
        for path in sorted(repo.rglob('*')):
            if path.is_file() and 'target' not in path.parts:
                manifest.append(dict(file=str(path.relative_to(r.parent)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(a/(prefix+'-sources.json')).write_text(json.dumps(dict(working=working, isolated=manifest), indent=2)+'\n')
for name in files:
    (a/(prefix+'-'+Path(name).name)).write_bytes((r/name).read_bytes())

def verify():
    for name, digest in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest() == digest, name
    for row in manifest: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']

env = dict(os.environ, **json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cmd = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'test', '--release', '--all-features', '--test', 'hypercurve_curve_region_promotion', '--no-run', '--message-format=json', '--locked', '--offline']
start = time.monotonic()
with (a/(prefix+'-build.jsonl')).open('w') as out, (a/(prefix+'-build.log')).open('w') as err:
    result = subprocess.run(cmd, cwd=r, env=env, stdout=out, stderr=err, timeout=900)
print('build', result.returncode, round(time.monotonic()-start, 2), flush=True)
verify()
if result.returncode:
    print((a/(prefix+'-build.log')).read_text()[-5000:], flush=True)
    raise SystemExit(1)
for line in (a/(prefix+'-build.jsonl')).read_text().splitlines():
    item = json.loads(line)
    if item.get('reason') == 'compiler-artifact' and item.get('executable') and item['target']['name'] == 'hypercurve_curve_region_promotion':
        assert item['fresh'] is False
        binary = a/(prefix+'-test')
        shutil.copy2(item['executable'], binary)
        break
else: raise AssertionError('missing binary')

names = ['unified_region_chamfer_and_fillet_edit_higher_order_loops', 'retained_circular_regions_chamfer_over_the_full_support', 'unified_region_reuses_design_parameter_corner_solvers', 'line_parabola_fillet_extends_the_regular_incident_cell_exactly', 'arc_parabola_fillet_recovers_exact_complement_contacts', 'non_ph_bezier_pair_projective_fillet_retains_algebraic_extensions']
def run(name):
    log = a/(prefix+'-'+name+'.log')
    command = [str(binary), '--exact', name, '--test-threads=1', '--nocapture', '--color', 'never']
    start = time.monotonic()
    with log.open('w') as out:
        try: code = subprocess.run(command, cwd=r, env=dict(env, HYPERCURVE_CORNER_DIAGNOSTIC='1'), stdout=out, stderr=subprocess.STDOUT, timeout=120).returncode
        except subprocess.TimeoutExpired: code = 'timeout'
    row = dict(name=name, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    print(json.dumps(row), flush=True)
    print(log.read_text()[-4500:], flush=True)
    return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    results = list(pool.map(run, names))
verify()
(a/(prefix+'-runs.json')).write_text(json.dumps(results, indent=2)+'\n')
