from pathlib import Path
import json, re, subprocess

audit = Path(__file__).resolve().parent
for prefix in ('circle-map-direction-20260923-final1', 'corner-publication-decision-sites-20260923-trace5'):
    assert json.loads((audit/f'{prefix}-terminal.json').read_text())['all_processes_reaped']
root = Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
path = root/'hypercurve/src/bezier_offset.rs'
source = path.read_text()
sections = [
    ('    pub(crate) fn parameter_at_certified_point(', '    /// Clips a support-certified point'),
    ('    pub(crate) fn source_related_intersections(', '    pub(crate) fn algebraic_endpoint_parameter('),
    ('    fn exact_line_retained_circle_intersections(', '    /// Reuses the selected circle/chord'),
    ('    fn recursive_projective_rational_intersections(', '    /// Discovers exact incidence evidence covering'),
    ('    pub(crate) fn rational_intersections(\n        &self,\n        source:', '    /// Reuses finite chord replay'),
]
branches = {}
edits = []
for start_marker, end_marker in sections:
    start = source.index(start_marker)
    end = source.index(end_marker, start+len(start_marker))
    part = source[start:end]
    def annotate(match):
        line = source[:start+match.start()].count('\n')+1
        label = f'CHORD_RATIONAL:{line}'
        branches[label] = {'file': 'src/bezier_offset.rs', 'line': line, 'context': part[max(0,match.start()-180):match.end()+90]}
        return '{ eprintln!("DECISION_SITE '+label+' reason={:?}", '+match.group('reason')+'); '+match.group()+' }'
    part = re.sub(r'return Ok\((?:Some\()?Classification::Uncertain\((?P<reason>reason|UncertaintyReason::\w+)\)\)?\);', annotate, part)
    part = part.replace('return Ok(Classification::Uncertain($reason));', 'eprintln!("CHORD_RATIONAL_RECURSIVE stage={} reason={:?}", $stage, $reason); return Ok(Classification::Uncertain($reason));')
    if 'parameter_at_certified_point' in start_marker:
        part = part.replace('let (lower, upper) = match orders {', 'if matches!(orders, (Classification::Uncertain(_), _) | (_, Classification::Uncertain(_))) { eprintln!("CHORD_FINITE_ORDERS orders={orders:?} point={:?} start={:?} end={:?}", std::mem::discriminant(&point.0), std::mem::discriminant(&self.start().0), std::mem::discriminant(&self.end().0)); } let (lower, upper) = match orders {')
    part = part.replace('Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),', 'Classification::Uncertain(reason) => { eprintln!("CHORD_RATIONAL_TAIL reason={reason:?}"); Ok(Classification::Uncertain(reason)) },')
    part = part.replace('Classification::Uncertain(reason) => Classification::Uncertain(reason),', 'Classification::Uncertain(reason) => { eprintln!("CHORD_RATIONAL_PROJECTIVE reason={reason:?}"); Classification::Uncertain(reason) },')
    part = part.replace('intersections => return Ok(intersections),', 'intersections => { if let Classification::Uncertain(reason) = &intersections { eprintln!("CHORD_RATIONAL_SOURCE_RELATED reason={reason:?}"); } return Ok(intersections); },')
    edits.append((start,end,part))
for start,end,part in sorted(edits,reverse=True):
    source = source[:start]+part+source[end:]
path.write_text(source)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt', '--edition', '2024', '--config', 'skip_children=true', str(path)],check=True)
(audit/'corner-publication-decision-sites6-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner = (audit/'run-corner-publication-decision-sites5-20260923.py').read_text().replace('corner-publication-decision-sites-20260923-trace5', 'corner-publication-decision-sites-20260923-trace6')
(audit/'run-corner-publication-decision-sites6-20260923.py').write_text(runner)
print('Prepared chord/rational replay diagnostics:',len(branches),'sites. Runner not launched.')
