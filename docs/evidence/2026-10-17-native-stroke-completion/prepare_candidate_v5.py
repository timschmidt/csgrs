"""Freeze the unchanged stroke fix and the reproduced typed text consumer repair."""
import gzip, hashlib, json, shutil, subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(W / 'circuit-baseline-v1/result.json')['baseline_compile_failure_reproduced']
assert read(W / 'candidate-validation-v4/result.json')['unexpected_exits'] == ['curve-all', 'circuit-all', 'clippy']
with gzip.open(W / 'candidate-preparation-v4/source-hashes.json.gz', 'rt') as f:
    prior = json.load(f)
with gzip.open(W / 'candidate-preparation-v1/baseline-source-hashes.json.gz', 'rt') as f:
    baseline = json.load(f)
assert all(sha(W / 'candidate-source-v4' / p) == h for p, h in prior.items())
assert all(sha(ROOT / p) == h for p, h in baseline.items())
source = W / 'candidate-source-v5'
out = W / 'candidate-preparation-v5'
source.mkdir(exist_ok=False)
out.mkdir(exist_ok=False)
for p, h in prior.items():
    dest = source / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(W / 'candidate-source-v4' / p, dest)
    assert sha(dest) == h

name = 'crates/hypercircuit/src/materialize.rs'
text = (source / name).read_text()
old = '''            let profile = decisions.rotated(
                &curve::truetype_text(text, &policy.font_data, point_height),
                rotation_degrees.clone(),
            )?;
'''
new = '''            let profile = decisions
                .consume_geometry(curve::truetype_text_with_context(
                    text,
                    &policy.font_data,
                    point_height,
                    &decisions.geometry_context(),
                ))
                .map_err(GeometryMaterializationError::Construction)?;
            let profile = decisions.rotated(&profile, rotation_degrees.clone())?;
'''
assert text.count(old) == 1
text = text.replace(old, new)
old = '/// Reproducible caller-selected font policy for production artwork.\n'
new = old + '''///
/// Construction uses the selected materialization context and retains its
/// certainty receipt. Invalid fonts, missing glyphs and failed outlines return
/// [`GeometryMaterializationError::Construction`] with the typed source error;
/// they no longer produce a successful report with omitted artwork.
'''
assert text.count(old) == 1
text = text.replace(old, new)
old = '    /// The selected production font could not produce geometry for non-whitespace text.\n'
new = '    /// A valid selected font produced an empty outline for non-whitespace text.\n'
assert text.count(old) == 1
(source / name).write_text(text.replace(old, new))

name = 'crates/hypercircuit/tests/fabrication.rs'
text = (source / name).read_text()
old = '''    let rejected = layout
        .materialize(
            &circuit,
            &hypercircuit::MaterializationContext::STRICT,
            MaterializationOptions {
                production_text: Some(ProductionTextPolicy {
                    font_name: "invalid-font".into(),
                    font_data: b"not a font".to_vec(),
                }),
                ..MaterializationOptions::default()
            },
        )
        .unwrap();
    assert!(rejected.process_omissions.iter().any(|omission| matches!(
        omission,
        ProcessMaterializationOmission::TextFontRejected { source }
            if source.ends_with(":reference-text")
    )));
'''
new = '''    // A selected but invalid font is a typed failure, so there is no partial
    // successful materialization available for the fabrication exporter.
    use csgrs::errors::ValidationError;
    use std::error::Error;
    for context in [
        hypercircuit::MaterializationContext::STRICT,
        hypercircuit::MaterializationContext::APPROXIMATE_512,
        hypercircuit::MaterializationContext::STRICT,
    ] {
        let error = layout
            .materialize(
                &circuit,
                &context,
                MaterializationOptions {
                    production_text: Some(ProductionTextPolicy {
                        font_name: "invalid-font".into(),
                        font_data: b"not a font".to_vec(),
                    }),
                    ..MaterializationOptions::default()
                },
            )
            .unwrap_err();
        assert!(matches!(
            &error,
            hypercircuit::GeometryMaterializationError::Construction(ValidationError::FontFace(_))
        ));
        assert!(matches!(
            error.source().unwrap().downcast_ref::<ValidationError>(),
            Some(ValidationError::FontFace(_))
        ));

        let mut missing_glyph = layout.clone();
        let mut changed_text = 0;
        for graphic in &mut missing_glyph.land_patterns[0].graphics {
            if let LandPatternGraphicPrimitive::Text { text, .. } = &mut graphic.primitive {
                // The valid A prefix must not escape as complete AB artwork.
                *text = "AB".to_owned();
                changed_text += 1;
            }
        }
        assert_eq!(changed_text, 1);
        let error = missing_glyph
            .materialize(
                &circuit,
                &context,
                MaterializationOptions {
                    production_text: Some(ProductionTextPolicy {
                        font_name: "ttf-parser-demo".into(),
                        font_data: demo_font(),
                    }),
                    ..MaterializationOptions::default()
                },
            )
            .unwrap_err();
        assert!(matches!(
            &error,
            hypercircuit::GeometryMaterializationError::Construction(
                ValidationError::MissingFontGlyph { character: 'B' }
            )
        ));
        assert!(matches!(
            error.source().unwrap().downcast_ref::<ValidationError>(),
            Some(ValidationError::MissingFontGlyph { character: 'B' })
        ));

        let valid = layout
            .materialize(
                &circuit,
                &context,
                MaterializationOptions {
                    production_text: Some(ProductionTextPolicy {
                        font_name: "ttf-parser-demo".into(),
                        font_data: demo_font(),
                    }),
                    ..MaterializationOptions::default()
                },
            )
            .unwrap();
        assert_eq!(valid.predicate_policy, context.predicate_policy());
        assert_eq!(valid.predicate_certainty, csgrs::GeometryCertainty::Certified);
        assert_eq!(valid.process_features.len(), materialized.process_features.len());
        assert_eq!(valid.production_text, materialized.production_text);
        assert!(!valid.process_omissions.iter().any(|omission| matches!(
            omission,
            ProcessMaterializationOmission::TextArtwork { .. }
                | ProcessMaterializationOmission::TextFontRejected { .. }
        )));
    }
'''
assert text.count(old) == 1
(source / name).write_text(text.replace(old, new))
fmt = subprocess.run(['cargo', 'fmt', '--all'], cwd=source, capture_output=True)
(out / 'format.stdout').write_bytes(fmt.stdout)
(out / 'format.stderr').write_bytes(fmt.stderr)
assert fmt.returncode == 0
final = {p: sha(source / p) for p in prior}
delta = {p: h for p, h in final.items() if prior[p] != h}
assert set(delta) == {'crates/hypercircuit/src/materialize.rs', 'crates/hypercircuit/tests/fabrication.rs'}
changes = {p: h for p, h in final.items() if baseline.get(p) != h}
assert len(final) == 2835 and len(changes) == 7
assert all(final[p] == h for p, h in read(W / 'candidate-preparation-v4/result.json')['changes'].items())
with gzip.open(out / 'source-hashes.json.gz', 'wt') as f:
    json.dump(final, f, sort_keys=True)
dump(out / 'result.json', {'complete': True, 'base_commit': 'b137381247a03342ce5b6184710475fcc22c9d63',
    'source_inputs': len(final), 'source_hashes_sha256': sha(out / 'source-hashes.json.gz'),
    'changes': changes, 'delta_from_v4': delta, 'runner_sha256': sha(__file__),
    'stroke_sources_unchanged_from_v4': True, 'baseline_compile_failure_reproduced': True,
    'main_sources_modified': False})
common = (W / 'validation_common_v4.py').read_text()
common = common.replace('candidate-source-v4', 'candidate-source-v5').replace('candidate-preparation-v4', 'candidate-preparation-v5')
for kind in ['validation', 'target', 'scratch', 'executables']:
    old = "WORK/(kind+'-" + kind + "-v4')"
    new = "WORK/'consumer-" + kind + "-v1'"
    assert old in common
    common = common.replace(old, new)
(W / 'consumer_common_v1.py').write_text(common)
print(json.dumps({'source_inputs': len(final), 'changed_paths': list(changes), 'delta_from_v4': list(delta)}), flush=True)
