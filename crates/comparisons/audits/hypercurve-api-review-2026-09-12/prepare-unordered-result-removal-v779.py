from pathlib import Path
import hashlib,json,re
A=Path(__file__).resolve().parent;W=A.parent;C=A/'unordered-result-removal-candidate-v779'
assert (A/'native-boundary-admission-v780-committed.json').exists()
assert not C.exists();base={};results={}
def change(name,edit):
 p=W/name;s=p.read_text();t=edit(s);assert s!=t,name
 base[name]=hashlib.sha256(p.read_bytes()).hexdigest();results[name]=t
def region(s):
 s=s.replace('    RetainedTopologyStatus, Segment2, SegmentKindCounts, UncertaintyReason,','    Segment2, UncertaintyReason,')
 a=s.index('/// Furthest exact stage reached by unified unordered-boundary arrangement.');b=s.index('/// Certified source-segmentation evidence',a);s=s[:a]+s[b:]
 a=s.index('impl CurveRegionArrangement2 {');b=s.index('impl CurveRegionSegmentationLoopEvidence2 {',a);s=s[:a]+s[b:]
 a=s.index('fn arrange_unordered_native_segments_raw(');b=s.index('fn curve_region_edit_error(',a)
 old=s[a:b];paths=old[old.index('    let paths = rings'):old.index('    let mut raw = match')]
 new='''fn arrange_unordered_native_segments_raw(
    source_segments: &[Segment2],
    fill_rule: FillRule,
    policy: &CurveContext,
) -> ExactCurveResult<CurveRegion2> {
    let rings = assemble_unordered_segment_rings(source_segments, policy).map_err(|reason| {
        let family = if source_segments.iter().any(|segment| matches!(segment, Segment2::Arc(_))) {
            CurveFamily2::CircularArc
        } else {
            CurveFamily2::Line
        };
        ExactCurveError::blocked(CurveOperation2::Construction, family, reason)
    })?;
'''+paths+'''    let mut raw = CurveRegion2::try_from_boundary_paths_raw(&paths, policy)?;
    if !raw.is_empty() {
        raw.data_mut_for_construction().certified_loop_fill_rules =
            Some(Arc::from(vec![fill_rule; paths.len()]));
    }
    raw.finish_construction(policy)
}

'''
 s=s[:a]+new+s[b:]
 s=s.replace('    ) -> ExactCurveResult<CurveOutcome<CurveRegionArrangement2>> {','    ) -> ExactCurveResult<CurveOutcome<Self>> {')
 old='''    /// operations. An empty collection produces the canonical empty region.'''
 assert s.count(old)==1
 s=s.replace(old,'''    /// operations. The rule fills each assembled walk; nesting and composition
    /// across walks use parity. An empty collection produces the canonical
    /// empty region. Unresolved assembly or arrangement returns an exact blocker.
    /// Output counts and native views are available on the returned region.''')
 assert 'CurveRegionArrangement'not in s and 'region_segment_kind_counts'not in s
 return s
change('hypercurve/src/bezier_region.rs',region)
change('hypercurve/src/lib.rs',lambda s:s.replace('CurveRegionArrangement2, CurveRegionArrangementStage2,',''))
def replace_fn(s,name,body):
 start=s.index('fn '+name+'(');a=s.rfind('#[test]',0,start);depth=0;opened=False
 for i in range(s.index('{',start),len(s)):
  if s[i]=='{':depth+=1;opened=True
  elif s[i]=='}':
   depth-=1
   if opened and depth==0:return s[:a]+body+s[i+1:]
 raise AssertionError(name)
def native(s):
 s=s.replace('    CurveRegionArrangement2, CurveRegionArrangementStage2, CurveString2, FillRule,','    CurveString2, ExactCurveError, ExactCurveResult, FillRule,')
 a=s.index('fn arrange_lines(');b=s.index('#[test]',a)
 s=s[:a]+'''fn arrange_lines(segments: Vec<crate::LineSeg2>, fill_rule: FillRule) -> ExactCurveResult<CurveRegion2> {
    arrange_segments(segments.into_iter().map(Segment2::Line).collect(), fill_rule)
}

fn arrange_segments(segments: Vec<Segment2>, fill_rule: FillRule) -> ExactCurveResult<CurveRegion2> {
    CurveRegion2::arrange_unordered_segments(&segments, fill_rule, &policy()).map(|outcome| outcome.into_value())
}

fn assert_boundary_blocked(result: ExactCurveResult<CurveRegion2>) {
    let Err(ExactCurveError::Blocked(blocker)) = result else {
        panic!("an open endpoint graph must retain its boundary blocker");
    };
    assert_eq!(blocker.operation(), crate::CurveOperation2::Construction);
    assert_eq!(blocker.reason(), UncertaintyReason::Boundary);
}

'''+s[b:]
 # Successful fixture helpers now return the common exact result. Failure
 # fixtures are replaced below without formatting a possible geometry payload.
 s=re.sub(r'(let (?:built|coincident) = arrange_(?:lines|segments)\([\s\S]*?)(\);)',r'\1).unwrap();',s)
 s=replace_fn(s,'unordered_open_lines_retain_a_boundary_blocker','''#[test]
fn unordered_open_lines_retain_a_boundary_blocker() {
    assert_boundary_blocked(arrange_lines(vec![line(0,0,1,0),line(3,0,4,0)], FillRule::NonZero));
}''')
 s=replace_fn(s,'unordered_crossing_and_overlapping_lines_remain_explicit_blockers','''#[test]
fn unordered_crossing_and_overlapping_lines_remain_explicit_blockers() {
    for lines in [vec![line(0,0,4,4),line(0,4,4,0)],vec![line(0,0,4,0),line(2,0,6,0)]] {
        assert_boundary_blocked(arrange_lines(lines, FillRule::NonZero));
    }
}''')
 a=s.index('fn native_overlap_regularizes_empty_and_open_crossings_remain_blocked()');b=s.index('#[test]',a)
 block=s[a:b]
 old='''        let built = arrange_segments(segments, FillRule::NonZero).unwrap();
        assert!(built.region().is_none());
        assert_eq!(built.blocker(), Some(UncertaintyReason::Boundary));'''
 assert old in block
 block=block.replace(old,'''        assert_boundary_blocked(arrange_segments(segments, FillRule::NonZero));''');s=s[:a]+block+s[b:]
 # Derived geometry facts stay on their authoritative region.
 s=re.sub(r'    (?:assert|prop_assert)!\(built.status\(\).is_native_exact\(\)\);\n','',s)
 s=s.replace('    assert!(coincident.status().is_native_exact());\n','')
 s=re.sub(r'    (?:assert_eq|prop_assert_eq)!\(built.source_segment_count\(\), \d+\);\n','',s)
 s=re.sub(r'    assert_eq!\(\s*built.stage\(\),\s*CurveRegionArrangementStage2::CurveArrangement\s*\);\n','',s)
 s=re.sub(r'((?:assert_eq|prop_assert_eq)!\()built.output_ring_count\(\), Some\((\d+)\)',r'\1built.len(), \2',s)
 s=s.replace('assert_eq!(coincident.output_ring_count(), Some(0));','assert!(coincident.is_empty());')
 s=s.replace('built.output_boundary_segment_count(), Some(4)','built.boundary_loops().iter().map(|loop_| loop_.len()).sum::<usize>(), 4')
 s=re.sub(r'let region = built\s*\.region\(\)\s*\.expect\("[^"\n]+"\);','let region = &built;',s)
 s=s.replace('built.region().unwrap()','&built').replace('built.region().expect("semicircle should materialize")','&built')
 s=s.replace('''coincident
            .region()
            .expect("oppositely traversed coincident arcs have decided topology")
            .is_empty()''','coincident.is_empty()')
 a=s.index('fn unordered_line_arc_segments_recover_the_exact_native_view()');b=s.index('#[test]',a)
 block=s[a:b];x=block.index('    assert_eq!(\n        built.output_boundary_segment_kind_counts()');y=block.index('    let region =',x);block=block[:x]+block[y:];s=s[:a]+block+s[b:]
 s=s.replace('built.output_boundary_segment_kind_counts(),\n            Some(SegmentKindCounts { lines: 1, arcs: 2 })','built.structural_facts(&policy()).unwrap().into_value().map(|facts| facts.segment_kinds),\n            Classification::Decided(SegmentKindCounts { lines: 1, arcs: 2 })')
 a=s.index('    let strict = CurveRegion2::arrange_unordered_segments(',s.index('fn unordered_native_arrangement_obeys_the_approximate_512_terminal()'));b=s.index('    let approximate =',a)
 s=s[:a]+'''    let strict = CurveRegion2::arrange_unordered_segments(&segments, FillRule::NonZero, &CurveContext::STRICT);
    assert!(matches!(strict, Err(ExactCurveError::Blocked(_))));

'''+s[b:]
 s=s.replace('    assert!(approximate.value.region().is_some());\n    assert!(approximate.value.status().is_native_exact());','    assert!(!approximate.value.is_empty());')
 a=s.index('            let arranged = outcome.into_value();',s.index('fn empty_unordered_arrangement_reenters_exact_set_operations()'));b=s.index('            assert!(empty.is_empty());',a)
 s=s[:a]+'            let empty = outcome.into_value();\n'+s[b:]
 s += """
#[test]
fn unordered_native_regions_reenter_operations_without_summary_queries() {
    use crate::{BooleanOp, OffsetCornerStyle2};
    let full_circle = CircularArc2::try_from_center(p(2,0),p(2,0),p(0,0),false).unwrap();
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for fill in [FillRule::NonZero, FillRule::EvenOdd] {
            for (segments, boundary, outside) in [
                (rectangle(0,0,4,4).segments().to_vec(), p(-1,2), p(-2,2)),
                (vec![Segment2::Arc(full_circle.clone())], p(3,0), p(4,0)),
            ] {
                let outcome = CurveRegion2::arrange_unordered_segments(&segments, fill, &policy).unwrap();
                assert_eq!(outcome.certainty, CurveCertainty::Certified);
                let region = outcome.into_value();
                // The next operation is deliberately the first consumer.
                let grown = region.offset(Real::one(), &OffsetCornerStyle2::Round, &policy).unwrap().into_value();
                assert_eq!(grown.classify_point(&boundary.into(), &policy).unwrap().into_value(), Classification::Decided(RegionPointLocation::Boundary));
                assert_eq!(grown.classify_point(&outside.into(), &policy).unwrap().into_value(), Classification::Decided(RegionPointLocation::Outside));
                let original = grown.boolean_region(&region, BooleanOp::Intersection, &policy).unwrap().into_value();
                assert!(original.boolean_region(&region, BooleanOp::Xor, &policy).unwrap().into_value().is_empty());
            }
        }
    }
}
"""
 for x in ['CurveRegionArrangement','built.region()','built.status()','output_boundary_segment','arranged.','strict.value.']:assert x not in s,x
 return s
change('hypercurve/src/native_region_tests.rs',native)
def example(s):
 s=s.replace('let result = CurveRegion2::arrange_unordered_segments(','let region = CurveRegion2::arrange_unordered_segments(')
 a=s.index('    let region = match result.region_classification()');b=s.index('    assert!(matches!(',a)
 return s[:a]+s[b:]
change('hypercurve/examples/arrangement.rs',example)
def bench(s):
 s=s.replace('CurveRegionArrangementStage2,','')
 for name,label,segments in [
  ('bench_unordered_line_segment_region_build','unordered_line_segment_region_build','''        Segment2::Line(line(0, 0, 10, 0)),
        Segment2::Line(line(0, 10, 10, 10)),
        Segment2::Line(line(0, 0, 0, 10)),
        Segment2::Line(line(10, 0, 10, 10)),'''),
  ('bench_unordered_native_segment_region_build','unordered_native_segment_region_build','''        Segment2::Line(line(4, 0, 0, 0)),
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(4, 0), s(1))?),''')]:
  a=s.index('fn '+name+'(');start=s.index('{',a);depth=0
  for i in range(start,len(s)):
   if s[i]=='{':depth+=1
   elif s[i]=='}':
    depth-=1
    if depth==0:b=i+1;break
  s=s[:a]+f'''fn {name}(iterations: u32) -> CurveResult<()> {{
    let segments = vec![
{segments}
    ];
    let policy = CurveContext::STRICT;
    let started = Instant::now();
    let mut total_loops = 0_usize;
    let mut total_spans = 0_usize;
    for _ in 0..iterations {{
        let region = CurveRegion2::arrange_unordered_segments(&segments, FillRule::NonZero, &policy)
            .expect("native arrangement must produce an exact region")
            .into_value();
        total_loops += black_box(region.len());
        total_spans += black_box(region.boundary_loops().iter().map(|loop_| loop_.len()).sum::<usize>());
    }}
    let elapsed = started.elapsed();
    println!("{label}: {{iterations}} iterations in {{elapsed:?}} ({{:?}}/iter), loops={{total_loops}}, spans={{total_spans}}", elapsed / iterations);
    Ok(())
}}'''+s[b:]
 return s
change('hypercurve/benches/editing.rs',bench)
def integration(s):
 return replace_fn(s,'unified_native_arrangement_exposes_immediate_evidence','''#[test]
fn unified_native_arrangement_returns_a_certified_region() {
    let source = square(0, 0, 4, 4);
    let result = CurveRegion2::arrange_unordered_segments(source.segments(), FillRule::NonZero, &CurveContext::STRICT).unwrap();
    assert_eq!(result.certainty, CurveCertainty::Certified);
    let region = result.into_value();
    assert_eq!(region.len(), 1);
    assert_eq!(decided(region.filled_area(&CurveContext::STRICT).unwrap()), Some(Real::from(16)));
    for (point, expected) in [(p(2,2),RegionPointLocation::Inside),(p(0,2),RegionPointLocation::Boundary),(p(5,2),RegionPointLocation::Outside)] {
        assert_eq!(region.classify_point(&point.into(), &CurveContext::STRICT).unwrap().into_value(), Classification::Decided(expected));
    }
}''')
change('hypercurve/tests/hypercurve_curve_region_promotion.rs',integration)
def docs(s):
 key='''  fill semantics.''';assert s.count(key)==1
 return s.replace(key,key+'''
  Unordered segment admission returns the region through the same exact-result
  contract. Unresolved endpoint assembly or topology remains an exact blocker;
  native views and structural counts are computed when requested.''')
change('hypercurve/README.md',docs)
def perf(s):
 s=s.replace('''### Immediate arrangement reports

The unordered''','''### Immediate arrangement reports

These measurements describe historical reporting interfaces. Unordered
arrangement now returns `CurveRegion2` through the common exact-result type;
its former report, stage, and eager summary computation have been removed.
Counts and native views are requested on the region. The historical timings
below do not measure that later removal.

The unordered''')
 s=s.replace('`CurveRegionArrangement2` follows the same `report()` vocabulary.','The curved arrangement result then followed the same `report()` vocabulary.')
 s=s.replace('''operation without entering a second report lifecycle. `CurveRegionArrangement2`
uses the same ownership model after immediately promoting any native output and
exposes its summary, status, blocker, and source count directly.''','''operation without entering a second report lifecycle. At this checkpoint,
the curved arrangement result used the same ownership model after immediately
promoting native output and exposed its summary, status, blocker, and source count.''')
 return s
change('hypercurve/PERFORMANCE.md',perf)
for name,source in results.items():
 q=C/name;q.parent.mkdir(parents=True,exist_ok=True);q.write_text(source)
(A/'unordered-result-removal-base-v779.json').write_text(json.dumps(base,indent=2)+'\n')
print('Prepared',len(base),'candidate files')
