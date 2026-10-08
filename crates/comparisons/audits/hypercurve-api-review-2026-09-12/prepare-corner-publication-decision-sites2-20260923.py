from pathlib import Path
import json, re, subprocess

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-corner-publication-decision-sites-2026-09-23')
assert json.loads((audit/'corner-publication-decision-sites-20260923-trace1-terminal.json').read_text())['all_processes_reaped']
branches = {}
p = root/'hypercurve/src/rational_bezier_general.rs'
s = p.read_text()
for start, end, tag in [
    ('fn shared_conic_endpoint_parameters(', '\nfn quadratic_homogeneous_blossom(', 'CONIC_ENDPOINT'),
    ('    fn endpoint_parameter_relation(', '\n    fn certified_polynomial_graph_component(', 'CONIC_OVERLAP'),
]:
    a=s.index(start); b=s.index(end,a); body=s[a:b]; base=s[:a].count('\n')+1
    def replacement(match):
        line=base+body[:match.start()].count('\n')
        label=f'{tag}:{line}'
        branches[label]=dict(file='src/rational_bezier_general.rs',line=line,expression=match.group())
        return '{ eprintln!("DECISION_SITE '+label+' reason={:?}", '+match.group(1)+'); '+match.group()+' }'
    body=re.sub(r'return Classification::Uncertain\((reason|UncertaintyReason::\w+)\)',replacement,body)
    s=s[:a]+body+s[b:]
p.write_text(s)

p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text()
a=s.index('    pub(crate) fn cmp_by_refinement(',s.index('impl BezierAlgebraicCuspSemicircleParameter2'))
b=s.index('    pub(crate) fn strict_scalar_between(',a)
body=s[a:b]
needle='        Ok(Classification::Uncertain(UncertaintyReason::Ordering))\n'
assert body.count(needle)==1
diagnostic='''        fn parameter_shape(parameter: &BezierAlgebraicCuspSemicircleParameter2, depth: usize) -> String {
            use BezierAlgebraicCuspSemicircleMappedParameterData2 as M;
            match parameter {
                BezierAlgebraicCuspSemicircleParameter2::Exact(_) => "Exact".into(),
                BezierAlgebraicCuspSemicircleParameter2::Mapped(data) => {
                    let tag = format!("{:?}", std::mem::discriminant(data.as_ref()));
                    if depth == 0 { return tag; }
                    match data.as_ref() {
                        M::PairOverlapMap { source, source_first, .. } => format!("PairOverlapMap({source_first},{})", parameter_shape(source, depth-1)),
                        M::Chamfer { source, .. } => format!("Chamfer({})", parameter_shape(source, depth-1)),
                        M::SimilarityTransport { source, .. } => format!("Similarity({})", parameter_shape(source, depth-1)),
                        _ => tag,
                    }
                }
            }
        }
        eprintln!("CUSP_CMP_EXHAUSTED first={} second={}", parameter_shape(self, 5), parameter_shape(other, 5));
'''
body=body.replace(needle,diagnostic+needle)
s=s[:a]+body+s[b:];p.write_text(s)

p=root/'hypercurve/src/bezier_split.rs';s=p.read_text()
a=s.index('pub(crate) fn intersect_parameter_ranges(');b=s.index('\n/// A native Bezier subcurve',a)
body=s[a:b]
needle='eprintln!("DECISION_SITE PARAMETER_CLIP:1190 reason={reason:?}");'
assert body.count(needle)==1
body=body.replace(needle,needle+'''
            eprintln!("CLIP_BOUNDS kinds=({:?},{:?}) low_first={} low_second={} high_first={} high_second={} structurally_equal={}", std::mem::discriminant(&low.data), std::mem::discriminant(&high.data), std::ptr::eq(low, first_low), std::ptr::eq(low, second_low), std::ptr::eq(high, first_high), std::ptr::eq(high, second_high), low == high);
''')
s=s[:a]+body+s[b:];p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',*[str(root/'hypercurve'/name) for name in ('src/rational_bezier_general.rs','src/bezier_offset.rs','src/bezier_split.rs')]],check=True)
(audit/'corner-publication-decision-sites2-20260923-branches.json').write_text(json.dumps(branches,indent=2)+'\n')
runner=(audit/'run-corner-publication-decision-sites-20260923.py').read_text().replace('corner-publication-decision-sites-20260923-trace1','corner-publication-decision-sites-20260923-trace2')
(audit/'run-corner-publication-decision-sites2-20260923.py').write_text(runner)
print('Prepared narrow conic and selected-circle identity diagnostics:',len(branches),'branches')
