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
r = Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
prefix = 'corner-normalization-20260923-diagnostic1'
files = ['src/bezier_region.rs', 'src/bezier_offset.rs', 'src/curve_region_boolean.rs', 'src/error.rs']
working = {name: hashlib.sha256((w/name).read_bytes()).hexdigest() for name in files}
for name in files:
    content = ((a/'single-loop-corner-candidate.rs').read_bytes() if name == 'src/bezier_region.rs'
               else subprocess.check_output(['git', 'show', 'c980fed:'+name], cwd=w))
    (r/name).write_bytes(content)

def replace(s, before, after, count=1):
    assert s.count(before) == count, (before[:140], s.count(before), count)
    return s.replace(before, after)

file = r/'src/error.rs'
s = file.read_text()
s = replace(s, '    pub(crate) const fn blocked(\n', '    #[track_caller]\n    pub(crate) fn blocked(\n')
s = replace(s, '        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))', '''        eprintln!("BLOCK {operation:?} {family:?} {reason:?} {}", std::panic::Location::caller());
        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))''')
file.write_text(s)

file = r/'src/curve_region_boolean.rs'
s = file.read_text()
s = replace(s, '        self.resolve_regularization(policy, || {', '''        eprintln!("normalize enter loops={} fragments={:?}", self.len(), self.boundary_loops().iter().map(|b|b.len()).collect::<Vec<_>>());
        self.resolve_regularization(policy, || {''')
s = replace(s, '        let pairs = build_unary_carrier_pairs(&carriers, policy)?;', '''        eprintln!("schedule enter carriers={}", carriers.len());
        let pairs = build_unary_carrier_pairs(&carriers, policy)?;
        eprintln!("schedule done pairs={}", pairs.len());''')
before = '            let result = self.pair_result(pair)?;'
s = replace(s, before, '''            eprintln!("pair enter {} {} kind={:?}", pair.first_carrier_index, pair.second_carrier_index, std::mem::discriminant(&pair.context));
            let result = self.pair_result(pair)?;
            eprintln!("pair done {} {} contacts={} overlaps={} blockers={}", pair.first_carrier_index, pair.second_carrier_index, result.contacts.len(), result.overlaps.len(), result.blockers.len());''', s.count(before))
s = replace(s, '            .map(|(carrier_index, carrier)| {\n                split_carrier(', '''            .map(|(carrier_index, carrier)| {
                eprintln!("split enter carrier={} events={}", carrier_index, events[carrier_index].len());
                split_carrier(''')
s = replace(s, '        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);', '''        eprintln!("split topology done");
        let simple_loop_filled_side = self.certified_simple_single_loop_filled_side(&topology);
        eprintln!("fragment selection enter simple_side={:?}", simple_loop_filled_side);''')
for label in ['first', 'second']:
    before = f'                let {label}_location = parameter_location_in_carrier('
    s = replace(s, before, f'                eprintln!("contact location {label} enter");\n'+before, s.count(before))
before = '                if first_location == CarrierParameterLocation::Outside'
s = replace(s, before, '                eprintln!("contact locations done {:?} {:?}", first_location, second_location);\n'+before, s.count(before))
s = replace(s, '                let first_existing = existing_contact_event_vertex_if_decided(', '                eprintln!("existing contact lookup enter");\n                let first_existing = existing_contact_event_vertex_if_decided(')
s = replace(s, '                let mut topology_vertex = first_existing.or(second_existing);', '                eprintln!("existing contact lookup done {:?} {:?}", first_existing, second_existing);\n                let mut topology_vertex = first_existing.or(second_existing);')
s = replace(s, '                for (existing_index, existing) in contact_points.iter().enumerate() {', '                eprintln!("contact identity enter count={}", contact_points.len());\n                for (existing_index, existing) in contact_points.iter().enumerate() {')
s = replace(s, '                let topology_vertex = topology_vertex.unwrap_or_else(|| {', '                eprintln!("contact identity done");\n                let topology_vertex = topology_vertex.unwrap_or_else(|| {')
start = s.index('fn parameter_location_in_carrier(\n')
end = s.index('\nfn ranges_intersect(', start)
block = s[start:end]
block = replace(block, '    if parameter == &carrier.start {', '    eprintln!("location storage start enter");\n    if parameter == &carrier.start {')
block = replace(block, '    if parameter == &carrier.end {', '    eprintln!("location storage end enter");\n    if parameter == &carrier.end {')
before = '    if let (Some(parameter), CurveSupport2::Circle(fragment)) ='
block = replace(block, before, '    eprintln!("location circle stage enter");\n'+before, block.count(before))
block = replace(block, '        let point = parameter\n', '        eprintln!("circle location point construction enter");\n        let point = parameter\n')
block = replace(block, '        if let Classification::Decided(Some(point)) = point {', '        eprintln!("circle location point construction done");\n        if let Classification::Decided(Some(point)) = point {')
block = replace(block, '            let location = fragment\n', '            eprintln!("circle location physical enter");\n            let location = fragment\n')
block = replace(block, '            match location {', '            eprintln!("circle location physical done {location:?}");\n            match location {')
block = replace(block, '        return match fragment\n', '        eprintln!("circle location final parameter order enter");\n        return match fragment\n')
s = s[:start]+block+s[end:]
file.write_text(s)

file = r/'src/bezier_region.rs'
s = file.read_text()
s = replace(s, '        let edited = Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)', '''        eprintln!("corner candidate enter operation={operation:?} loop={loop_index} fragments={}", loops[loop_index].len());
        let edited = Self::try_new_with_loop_topology(loops, roles, fill_rules, interior_sides)''')
file.write_text(s)

file = r/'src/bezier_offset.rs'
s = file.read_text()
start = s.index('    pub(crate) fn certified_incident_point_evidence_location(\n')
end = s.index('    fn incident_location_from_orders(', start)
block = s[start:end]
block = replace(block, '        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Start};', '''        eprintln!("circle endpoint identities enter");
        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Start};''')
block = replace(block, '            let endpoint = match self.endpoint_point_evidence(at_start, policy)? {', '            eprintln!("circle endpoint {at_start} construction enter");\n            let endpoint = match self.endpoint_point_evidence(at_start, policy)? {')
block = replace(block, '        let endpoint_side =\n', '        eprintln!("circle bounded chord side enter");\n        let endpoint_side =\n')
block = replace(block, '        if let Classification::Decided(side) = endpoint_side', '        eprintln!("circle bounded chord side done {endpoint_side:?}");\n        if let Classification::Decided(side) = endpoint_side')
block = replace(block, '            let endpoint_parameter = if source_start {', '            eprintln!("circle order {source_start} bounded scalar enter");\n            let endpoint_parameter = if source_start {')
block = replace(block, '            if matches!(scalar_order, Classification::Decided(_)) {', '            eprintln!("circle bounded scalar order done {scalar_order:?}");\n            if matches!(scalar_order, Classification::Decided(_)) {')
block = replace(block, '            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {', '            eprintln!("circle shared parametric point enter");\n            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {')
block = replace(block, '            let projective = recursive_projective_incident_point_order(', '            eprintln!("circle recursive point order enter");\n            let projective = recursive_projective_incident_point_order(')
block = replace(block, '            if let Some(Classification::Decided(order)) = projective {', '            eprintln!("circle recursive point order done {projective:?}");\n            if let Some(Classification::Decided(order)) = projective {')
s = s[:start]+block+s[end:]
file.write_text(s)

manifest = [dict(file=str(p.relative_to(r.parent)), sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for repo in sorted(r.parent.iterdir()) if repo.is_dir()
            for p in sorted(repo.rglob('*')) if p.is_file() and 'target' not in p.parts]
(a/(prefix+'-sources.json')).write_text(json.dumps(dict(working=working, isolated=manifest), indent=2)+'\n')
for name in files:
    (a/(prefix+'-'+Path(name).name)).write_bytes((r/name).read_bytes())
def verify():
    for name,digest in working.items(): assert hashlib.sha256((w/name).read_bytes()).hexdigest()==digest,name
    for row in manifest: assert hashlib.sha256((r.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']

env = dict(os.environ, **json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cmd = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','test','--release','--all-features','--lib','--no-run','--message-format=json','--locked','--offline']
start=time.monotonic()
with (a/(prefix+'-build.jsonl')).open('w') as out, (a/(prefix+'-build.log')).open('w') as err:
    code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=err,timeout=900).returncode
verify()
print('build',code,round(time.monotonic()-start,2),flush=True)
if code:
    print((a/(prefix+'-build.log')).read_text()[-6000:],flush=True)
    raise SystemExit(code)
for line in (a/(prefix+'-build.jsonl')).read_text().splitlines():
    item=json.loads(line)
    if item.get('reason')=='compiler-artifact' and item.get('executable') and item['target']['name']=='hypercurve':
        assert item['fresh'] is False
        binary=a/(prefix+'-libtest');shutil.copy2(item['executable'],binary)
        break
else: raise AssertionError('missing library')
names=[
 'general_nonrepresented_chord_and_retained_rational_arc_complete_the_fillet_kernel',
 'independent_oblique_chord_pair_fillet_crosses_a_rational_line_exactly',
 'independent_oblique_chord_pair_fillet_crosses_algebraic_chords_exactly',
]
listing=subprocess.check_output([str(binary),'--list'],cwd=r,text=True).splitlines()
def run(suffix):
    matches=[line.removesuffix(': test') for line in listing if line.endswith(suffix+': test')]
    assert len(matches)==1
    cmd=[str(binary),'--exact',matches[0],'--test-threads=1','--nocapture','--color','never']
    log=a/(prefix+'-'+suffix+'.log')
    start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run(cmd,cwd=r,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
        except subprocess.TimeoutExpired: code='timeout'
    row=dict(name=matches[0],command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    print(json.dumps(row),flush=True);print(log.read_text()[-4500:],flush=True)
    return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    rows=list(pool.map(run,names))
verify()
(a/(prefix+'-runs.json')).write_text(json.dumps(rows,indent=2)+'\n')
