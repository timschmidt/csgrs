#![allow(dead_code)]
use std::{collections::{BTreeMap, BTreeSet}, fs, path::Path};
use hypercircuit::{DrcReadinessPolicy, HyperDrcHandoff, MaterializationOptions, SemanticDocument};
use hyperdrc::{PcbRegionExt, Scalar, Violation, Severity, EvidenceContext};
pub use hyperdrc::geometry;
mod scalar {
    use super::Scalar;
    pub fn compare(left: &Scalar, right: &Scalar) -> Option<std::cmp::Ordering> { hyperlimit::compare_reals(left, right, super::PREDICATE_POLICY).value() }
    pub fn lt(a: &Scalar, b: &Scalar) -> bool { compare(a,b).is_some_and(|o| o.is_lt()) }
    pub fn le(a: &Scalar, b: &Scalar) -> bool { compare(a,b).is_some_and(|o| o.is_le()) }
    pub fn eq(a: &Scalar, b: &Scalar) -> bool { compare(a,b).is_some_and(|o| o.is_eq()) }
    pub fn ge(a: &Scalar, b: &Scalar) -> bool { compare(a,b).is_some_and(|o| o.is_ge()) }
}
const PREDICATE_POLICY: hyperlimit::PredicatePolicy = hyperlimit::PredicatePolicy::STRICT;
mod legacy_distance;

fn location(feature: &hyperdrc::kicad::CopperFeature) -> [f64; 2] {
    feature.location.each_ref().map(|x| x.to_f64_lossy().unwrap())
}
fn native_distance(left: &hyperdrc::PcbRegion, right: &hyperdrc::PcbRegion) -> Scalar {
    hypercurve::evaluate_under(PREDICATE_POLICY, || {
        let left = left.native_contours_fast_path().unwrap().unwrap();
        let right = right.native_contours_fast_path().unwrap().unwrap();
        let mut minimum = None;
        // Intentionally unpruned: audit the production candidate schedule
        // against every pair of authoritative native line/arc segments.
        for a in left.material_contours().iter().chain(left.hole_contours()).flat_map(|c| c.segments()) {
            for b in right.material_contours().iter().chain(right.hole_contours()).flat_map(|c| c.segments()) {
                let squared = a.distance_squared(b).unwrap();
                if minimum.as_ref().is_none_or(|old| scalar::compare(&squared, old).unwrap().is_lt()) { minimum = Some(squared); }
            }
        }
        minimum.unwrap().sqrt().unwrap()
    }).certified().unwrap()
}
fn main() {
    let out = std::env::args().nth(1).unwrap();
    fs::create_dir_all(&out).unwrap();
    for slug in ["nano", "uno", "esp32", "esp32s3", "rp2040", "stm32f103"] {
        let base = Path::new("crates/hypercircuit/tests/fixtures/easyduino");
        let document = SemanticDocument::from_json(&fs::read_to_string(base.join("native").join(format!("{slug}.hypercircuit.json"))).unwrap()).unwrap();
        let mut layout = document.pcb.as_ref().unwrap().clone();
        layout.zones.clear();
        let materialized = layout.materialize(&document.circuit, &hypercircuit::MaterializationContext::STRICT, MaterializationOptions { circular_segments: 8, aggregate_layer_images: false, ..Default::default() }).unwrap();
        let handoff = HyperDrcHandoff::from_materialization(&layout, &materialized);
        let report = handoff.run_readiness(&DrcReadinessPolicy::default());
        fs::write(Path::new(&out).join(format!("{slug}-violations.json")), serde_json::to_vec_pretty(&report.violations).unwrap()).unwrap();
        let baseline = std::env::args().nth(2).map(std::path::PathBuf::from).unwrap_or_else(|| base.join("fast-drc"));
        let old: serde_json::Value = serde_json::from_slice(&fs::read(baseline.join(format!("{slug}.json"))).unwrap()).unwrap();
        let before = old["violations"].as_array().unwrap().iter().map(|v| v["evidence_id"].as_str().unwrap().to_owned()).collect::<BTreeSet<_>>();
        let after = report.violations.iter().map(|v| v.evidence_id.clone()).collect::<BTreeSet<_>>();
        let requested = before.symmetric_difference(&after).cloned().collect::<BTreeSet<_>>();
        let mut unmatched = requested.clone();
        let context = EvidenceContext { release_digest: None, policy_digest: Some(report.capability_profile_digest.clone()) };
        let features = &handoff.board.copper;
        let mut views = BTreeMap::new();
        let mut rows = Vec::new();
        let thresholds = handoff.net_classes.iter().filter_map(|c| c.min_clearance.as_ref().map(|value| (c.name.as_str(), value))).collect::<Vec<_>>();
        let max_clearance = thresholds.iter().filter_map(|(_, v)| v.to_f64_lossy()).fold(0.0, f64::max);
        let bounds = features.iter().map(|f| f.region.bounds().unwrap().unwrap()).collect::<Vec<_>>();
        for (i, left) in features.iter().enumerate() {
            for (j, right) in features.iter().enumerate().skip(i + 1) {
                if left.layer != right.layer || left.net == right.net { continue; }
                let a = &bounds[i]; let b = &bounds[j];
                // Only discover IDs here; every requested ID must be found.
                // A generous finite scheduling margin is never a metric oracle.
                let margin = max_clearance + 0.1;
                if a.min_x().to_f64_lossy().unwrap() > b.max_x().to_f64_lossy().unwrap() + margin
                    || b.min_x().to_f64_lossy().unwrap() > a.max_x().to_f64_lossy().unwrap() + margin
                    || a.min_y().to_f64_lossy().unwrap() > b.max_y().to_f64_lossy().unwrap() + margin
                    || b.min_y().to_f64_lossy().unwrap() > a.max_y().to_f64_lossy().unwrap() + margin { continue; }
                let mut identity = Violation::new("net-constraint-readiness", Severity::Error, vec![left.layer.clone()], None, vec![], vec![location(left), location(right)], None);
                identity.bind_evidence_context(&context);
                if !requested.contains(&identity.evidence_id) { continue; }
                unmatched.remove(&identity.evidence_id);
                let native = native_distance(&left.region, &right.region);
                for (index, feature) in [(i, left), (j, right)] { views.entry(index).or_insert_with(|| feature.region.try_to_multipolygon().unwrap()); }
                let chords = legacy_distance::polygon_boundary_distance_scalar(&views[&i], &views[&j]).unwrap();
                let comparisons = thresholds.iter().map(|(name, threshold)| serde_json::json!({"class":name,"threshold":threshold.to_string(),"native_below":scalar::compare(&native, threshold).unwrap().is_lt(),"chords_below":scalar::compare(&chords, threshold).unwrap().is_lt()})).collect::<Vec<_>>();
                rows.push(serde_json::json!({"evidence_id":identity.evidence_id,"change":if after.contains(&identity.evidence_id){"added"}else{"removed"},"indices":[i,j],"layer":left.layer,"nets":[left.net,right.net],"locations":[location(left),location(right)],"native_distance":native.to_string(),"chord_distance":chords.to_string(),"native_display":native.to_f64_lossy(),"chord_display":chords.to_f64_lossy(),"thresholds":comparisons}));
            }
        }
        let result = serde_json::json!({"board":slug,"rows":rows,"unmatched":unmatched});
        fs::write(Path::new(&out).join(format!("{slug}-audit.json")), serde_json::to_vec_pretty(&result).unwrap()).unwrap();
        println!("{slug}: changed={} unmatched={}",result["rows"].as_array().unwrap().len(),unmatched.len());
        assert!(unmatched.is_empty(), "all changed finding IDs must be attributed");
    }
}
