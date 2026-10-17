"""Freeze a narrow typed source-failure candidate after current reproduction."""
import gzip, hashlib, json, shutil, subprocess
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];S=W/'candidate-source-v1';O=W/'candidate-preparation-v1'
BASE='ac14e99174318291a548749395552ab982a7d097'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
reproduction=read(W/'baseline-probe-v1/result.json')
assert reproduction['complete'] and reproduction['invalid_source_observations']==96
assert reproduction['invalid_source_categories']=={'missing_feature':81,'successful_geometry':15,'retained_failure':0}
assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==BASE
with gzip.open(W/'baseline-preparation-v1/source-hashes.json.gz','rt') as f:inputs=json.load(f)
assert all(sha(ROOT/p)==h for p,h in inputs.items())
S.mkdir(exist_ok=False);O.mkdir(exist_ok=False)
for p,h in inputs.items():
    dest=S/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,dest);assert sha(dest)==h
for fixture in ['CPArti FPGA dev board.zip','HVP109A.zip']:
    source=W/'baseline-source-v1/crates/hyperdrc/docs'/fixture
    dest=S/'crates/hyperdrc/docs'/fixture
    shutil.copy2(source,dest);assert sha(dest)==sha(source)
shutil.copy2(W/'draft-v1/tracks.rs',S/'crates/hyperdrc/src/kicad/tracks.rs')
p=S/'crates/hyperdrc/src/kicad.rs';s=p.read_text()
s=s.replace('mod text;','mod text;\nmod tracks;\n\npub use tracks::{TrackLoadError, TrackLoadFailure};')
s=s.replace('SourceUnit, line_polygon, polygon_from_points,','SourceUnit, polygon_from_points,')
s=s.replace('/// Run or compute `load_kicad_pcb`.','''/// Load a KiCad board, preserving exact source locations and native PCB regions.
///
/// # Errors
///
/// Straight tracks require one `start` and `end` pair and one nonnegative
/// `width`. Malformed fields, unresolved signs, exhausted shared work budgets,
/// and failed positive-width construction return a [`TrackLoadError`] through
/// the error chain before a board is published. Callers can downcast through
/// the path context. Validated zero-width tracks denote the empty set.
///
/// Straight-track bodies still use the legacy finite rectangular adapter:
/// success does not certify round caps or fidelity to the exact source stroke.
/// Other KiCad object families retain their existing validation contracts.
/// No total source-size, parsing-work, memory, or cancellation bound is added.''')
old='''        &mut drills,
    );
    parse_zones'''
new='''        &mut drills,
    )
    .with_context(|| format!("failed to load tracks from {}", path.display()))?;
    parse_zones'''
assert s.count(old)==1;s=s.replace(old,new)
begin=s.index('fn parse_tracks_and_vias(');start=s.index(') {\n    for segment',begin);end=s.index('    for via',start)
s=s[:start]+''') -> Result<(), TrackLoadError> {
    tracks::parse_segments(root, nets, copper)?;

'''+s[end:]
end=s.index('\nfn drill_diameter_scalar',begin)
assert s[end-3:end]=='\n}\n';s=s[:end-3]+'\n    Ok(())\n}\n'+s[end:]
p.write_text(s)
p=S/'crates/hyperdrc/src/readiness.rs';s=p.read_text();assert s.count('CHECK_IMPLEMENTATION_VERSION: u32 = 20;')==1
p.write_text(s.replace('CHECK_IMPLEMENTATION_VERSION: u32 = 20;','CHECK_IMPLEMENTATION_VERSION: u32 = 21;'))
p=S/'crates/hyperdrc/README.md';s=p.read_text();anchor='## Useful API\n';assert s.count(anchor)==1
s=s.replace(anchor,'''### Straight-track load failures

The KiCad loader now rejects malformed straight-track `start`, `end`, and
`width` fields instead of dropping, defaulting, or truncating them. Each field
must occur once with two endpoint coordinates or one width. Negative widths
fail using the exact source value, including values below binary64 range.
Validated zero width retains the existing empty-set behavior.

`load_kicad_pcb` keeps its `anyhow::Result<BoardModel>` signature. A caller can
downcast the error to `kicad::TrackLoadError` and inspect its zero-based segment
index and `TrackLoadFailure`: invalid fields, an undecided width sign, shared
budget exhaustion, unsupported finite geometry, or native geometry failure.
No partial board is returned. The CLI reports the source path and diagnosis
with exit status 1, including with `--allow-findings`.

This is a failure-contract migration; readiness implementation version 21
invalidates version-20 check evidence. Ordinary admitted tracks retain their
existing geometry and metadata. Their body still comes from the legacy finite
rectangular adapter: exact round caps, exact coordinate fidelity, and extreme
source ranges remain open work. Rejecting a positive-width track that collapses
in this adapter makes failure visible; it does not complete that geometry.
Other KiCad object families retain their existing contracts. Shared refinement
budgets do not bound source parsing, total work, heap bytes, or cancellation.

'''+anchor);p.write_text(s)

# Preserve all individually reproduced source cases, adding typed expectations.
s=(W/'prototype-v1/public_tests.rs').read_text()
s=s.replace('BoardModel, load_kicad_pcb','BoardModel, TrackLoadError, TrackLoadFailure, load_kicad_pcb')
s=s.replace('fn invalid_source(fields: &str) {','fn invalid_source(fields: &str, expected: TrackLoadFailure) {')
s=s.replace('''        assert!(result.is_err(), "invalid source published a board: {fields}");''','''        let error = result.expect_err("invalid source must not publish a board");
        let track = error.downcast_ref::<TrackLoadError>().expect("typed track source error");
        assert_eq!(track.segment_index, 0);
        assert_eq!(track.failure, expected, "{fields}");
        assert!(format!("{error:#}").contains("failed to load tracks from"));''')
begin=s.index('macro_rules! malformed');end=s.index('\n#[test]\nfn valid_source',begin)
replacement='''macro_rules! malformed {
    ($name:ident, $fields:literal, $failure:expr) => {
        #[test]
        fn $name() { invalid_source($fields, $failure); }
    };
}
'''
for case in read(W/'prototype-v1/cases.json'):
    if case['expected']!='invalid_input':continue
    name=case['name'];field=name.split('_')[0]
    if name=='zero_width_missing_end':field='end';suffix='missing'
    elif name=='zero_width_invalid_start':field='start';suffix='bad'
    else:suffix=name[len(field)+1:]
    if suffix=='missing':kind=f'MissingField {{ field: "{field}" }}'
    elif suffix=='duplicate':kind=f'DuplicateField {{ field: "{field}" }}'
    elif suffix in ['empty','short','extra']:
        n=1 if field=='width' else 2;actual={'empty':0,'short':1,'extra':n+1}[suffix]
        kind=f'InvalidArity {{ field: "{field}", expected: {n}, actual: {actual} }}'
    elif suffix.startswith('negative'):kind='NegativeWidth'
    else:kind=f'InvalidNumber {{ field: "{field}", value_index: {1 if suffix=="bad_y" else 0} }}'
    replacement+=f'malformed!({name}, {json.dumps(case["fields"])}, TrackLoadFailure::{kind});\n'
s=s[:begin]+replacement+s[end:]
s=s.replace('''    assert!(escaped.unwrap().is_err());''','''    let error = escaped.unwrap().expect_err("exhausted load cannot escape as a board");
    let track = error.downcast_ref::<TrackLoadError>().expect("typed budget failure survives scope exit");
    let TrackLoadFailure::BudgetExceeded(budget) = track.failure else { panic!("{track:?}") };
    assert_eq!(budget.limit, 0);
    assert_eq!(budget.used, 0);
    assert_eq!(budget.requested, 1);
    assert!(std::error::Error::source(track).is_some());''')
s+='''
#[test]
fn positive_source_that_the_finite_adapter_cannot_construct_is_an_error() {
    for fields in [
        "(start 1 2) (end 1 2) (width 2)",
        "(start 0 0) (end 2 0) (width 1e-400)",
        "(start 0 0) (end 1e-200 0) (width 2)",
        "(start 1e400 0) (end 1e400 2) (width 2)",
        "(start -1e308 0) (end 1e308 0) (width 2)",
        "(start 0 0) (end 2 0) (width 1e400)",
        "(start 100000000000000000000 0) (end 100000000000000000001 0) (width 2)",
    ] {
        for policy in [hyperlimit::PredicatePolicy::STRICT, hyperlimit::PredicatePolicy::APPROXIMATE_512, hyperlimit::PredicatePolicy::STRICT] {
            let result = hypercurve::evaluate_under(policy, || board(fields)).into_unverified();
            let error = result.expect_err("positive source must not disappear as empty geometry");
            let track = error.downcast_ref::<TrackLoadError>().expect("typed adapter failure");
            assert!(matches!(track.failure, TrackLoadFailure::UnsupportedGeometry { .. } | TrackLoadFailure::Geometry(_)), "{track:?}");
        }
    }
}

#[test]
fn second_segment_failure_does_not_publish_the_valid_prefix() {
    let error = board("(start 10 20) (end 12 20) (width 2)) (segment (start 0 0) (end 2 0)")
        .expect_err("valid prefix must not escape as a complete board");
    let track = error.downcast_ref::<TrackLoadError>().unwrap();
    assert_eq!(track.segment_index, 1);
    assert_eq!(track.failure, TrackLoadFailure::MissingField { field: "width" });
}

#[test]
fn cli_preserves_source_diagnostic_and_cannot_allow_a_failed_load() {
    let path = std::env::temp_dir().join(format!("hyperdrc-track-cli-{}.kicad_pcb", std::process::id()));
    std::fs::write(&path, "(kicad_pcb (segment (start 10 20) (end 12 20) (width 2)) (segment (start 0 0) (end 2 0)))").unwrap();
    let result = std::process::Command::new(env!("CARGO_BIN_EXE_hyperdrc"))
        .args(["--kicad-pcb"]).arg(&path)
        .args(["--check", "copper-net-intent", "--format", "json", "--allow-findings"])
        .output().unwrap();
    std::fs::remove_file(&path).unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(result.stdout.is_empty(), "no success report for a partial board");
    let diagnostic = String::from_utf8(result.stderr).unwrap();
    assert!(diagnostic.contains(&path.display().to_string()));
    assert!(diagnostic.contains("KiCad segment 2: missing width"), "{diagnostic}");
}
'''
(S/'crates/hyperdrc/tests/kicad_track_failures.rs').write_text(s)
changes=['crates/hyperdrc/src/kicad.rs','crates/hyperdrc/src/kicad/tracks.rs','crates/hyperdrc/src/readiness.rs','crates/hyperdrc/README.md','crates/hyperdrc/tests/kicad_track_failures.rs']
subprocess.run(['cargo','fmt','--all'],cwd=S,check=True)
assert {p for p,h in inputs.items() if sha(S/p)!=h}==set(changes)&set(inputs)
for p in changes:inputs[p]=sha(S/p)
with gzip.open(O/'source-hashes.json.gz','wt') as f:json.dump(inputs,f,sort_keys=True)
dump(O/'result.json',{'complete':True,'base_commit':BASE,'source_inputs':len(inputs),'source_hashes_sha256':sha(O/'source-hashes.json.gz'),
    'changes':{p:inputs[p] for p in changes},'runner_sha256':sha(__file__),'draft_sha256':sha(W/'draft-v1/tracks.rs'),
    'contracts_sha256':{p.name:sha(p) for p in W.glob('contract-v*.md')},'reproduction_sha256':sha(W/'baseline-probe-v1/result.json'),'main_sources_modified':False})
print(json.dumps({'complete':True,'source_inputs':len(inputs),'changes':changes}),flush=True)
