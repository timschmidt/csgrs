#[path = "common/pathological.rs"]
mod pathological_fixture;

use std::env;
use std::hint::black_box;
use std::time::Instant;

use hypercurve::{BooleanOp, Classification, CurvePolicy, FillRule};

use pathological_fixture::{MemoryTier, NativeDataset, rotated_region, selected_tiers};

#[derive(Clone, Copy)]
enum BenchmarkMode {
    Build,
    Transform,
    Boolean,
    All,
}

impl BenchmarkMode {
    fn from_environment() -> Self {
        match env::var("HYPERCURVE_PATHOLOGICAL_MODE")
            .unwrap_or_else(|_| "all".to_owned())
            .to_ascii_lowercase()
            .as_str()
        {
            "build" => Self::Build,
            "transform" | "rotate" => Self::Transform,
            "boolean" | "booleans" => Self::Boolean,
            "all" => Self::All,
            value => panic!(
                "invalid HYPERCURVE_PATHOLOGICAL_MODE={value:?}; expected build, transform, boolean, or all"
            ),
        }
    }

    const fn includes_transform(self) -> bool {
        matches!(self, Self::Transform | Self::All)
    }

    const fn includes_boolean(self) -> bool {
        matches!(self, Self::Boolean | Self::All)
    }
}

fn main() {
    let mode = BenchmarkMode::from_environment();
    let tiers = selected_tiers("HYPERCURVE_PATHOLOGICAL_TIERS", &[MemoryTier::Mib100]);
    println!(
        "pathological CurveRegion2 benchmark: tiers={:?}, cell_limit={:?}",
        tiers.iter().map(|tier| tier.name()).collect::<Vec<_>>(),
        env::var("HYPERCURVE_PATHOLOGICAL_CELL_LIMIT").ok(),
    );
    println!(
        "each cell contains every curve family, every Real representation sample, and an exact 3-4-5 rotated/translated mate"
    );

    for tier in tiers {
        run_tier(tier, mode);
    }
}

fn run_tier(tier: MemoryTier, mode: BenchmarkMode) {
    let resident_before = resident_bytes();
    let started = Instant::now();
    let dataset = NativeDataset::build(tier);
    let build_elapsed = started.elapsed();
    let resident_delta = resident_before
        .zip(resident_bytes())
        .map(|(before, after)| after.saturating_sub(before));
    println!(
        "pathological/{}/build: cells={} estimated_input={} observed_rss_delta={} elapsed={build_elapsed:?}",
        tier.name(),
        dataset.cells.len(),
        format_bytes(dataset.estimated_resident_bytes),
        resident_delta.map_or_else(|| "unavailable".to_owned(), format_bytes),
    );
    assert_eq!(dataset.tier, tier);

    let family_checksum = dataset
        .cells
        .iter()
        .flat_map(|cell| cell.source_path.curves())
        .fold(0_usize, |checksum, curve| {
            checksum ^ curve.family() as usize
        });
    let representation_checksum = dataset
        .cells
        .iter()
        .flat_map(|cell| &cell.representations)
        .fold(0_usize, |checksum, sample| {
            checksum
                ^ sample.name.len()
                ^ usize::from(sample.value.structural_facts().exact_rational)
        });
    black_box((family_checksum, representation_checksum));

    if mode.includes_transform() {
        let started = Instant::now();
        let transformed = dataset
            .cells
            .iter()
            .enumerate()
            .map(|(index, cell)| rotated_region(&cell.source_path, index))
            .collect::<Vec<_>>();
        let boundary_checksum = transformed.iter().map(|region| region.len()).sum::<usize>();
        println!(
            "pathological/{}/deep_copy_rotate: cells={} boundaries={} elapsed={:?}",
            tier.name(),
            transformed.len(),
            black_box(boundary_checksum),
            started.elapsed(),
        );
        black_box(transformed);
    }

    if mode.includes_boolean() {
        benchmark_booleans(&dataset);
    }
}

fn benchmark_booleans(dataset: &NativeDataset) {
    let policy = CurvePolicy::certified();
    let started = Instant::now();
    let mut prepared_count = 0_usize;
    let mut candidate_pair_count = 0_usize;
    let mut decided_count = 0_usize;
    let mut blocked_count = 0_usize;
    let mut boundary_checksum = 0_usize;
    let mut first_blocker = None;

    for cell in &dataset.cells {
        match cell.source.try_prepare_boolean(&cell.rotated, &policy) {
            Ok(prepared) => {
                prepared_count += 1;
                candidate_pair_count += prepared.carrier_pair_count();
                for operation in [
                    BooleanOp::Union,
                    BooleanOp::Intersection,
                    BooleanOp::Difference,
                    BooleanOp::Xor,
                ] {
                    match prepared.boolean_region_view(operation) {
                        Ok(region) => {
                            decided_count += 1;
                            boundary_checksum ^= region.len();
                        }
                        Err(error) => {
                            blocked_count += 1;
                            first_blocker.get_or_insert_with(|| error.to_string());
                        }
                    }
                }
            }
            Err(error) => {
                blocked_count += 4;
                first_blocker.get_or_insert_with(|| error.to_string());
            }
        }
    }

    println!(
        "pathological/{}/boolean_all_ops: cells={} prepared={} candidate_pairs={} decided={} blocked={} first_blocker={first_blocker:?} checksum={} elapsed={:?}",
        dataset.tier.name(),
        dataset.cells.len(),
        prepared_count,
        candidate_pair_count,
        decided_count,
        blocked_count,
        black_box(boundary_checksum),
        started.elapsed(),
    );

    let started = Instant::now();
    let mut projection_decided = 0_usize;
    let mut projection_uncertain = 0_usize;
    let mut projection_errors = 0_usize;
    let mut projection_checksum = 0_usize;
    for cell in &dataset.cells {
        for operation in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::Xor,
        ] {
            match cell.source_projection.boolean_region(
                &cell.rotated_projection,
                operation,
                FillRule::EvenOdd,
                &policy,
            ) {
                Ok(Classification::Decided(region)) => {
                    projection_decided += 1;
                    projection_checksum ^=
                        region.material_contours().len() + region.hole_contours().len();
                }
                Ok(Classification::Uncertain(_)) => projection_uncertain += 1,
                Err(_) => projection_errors += 1,
            }
        }
    }
    println!(
        "pathological/{}/flattened_exact_boolean_all_ops: cells={} decided={} uncertain={} errors={} checksum={} elapsed={:?}",
        dataset.tier.name(),
        dataset.cells.len(),
        projection_decided,
        projection_uncertain,
        projection_errors,
        black_box(projection_checksum),
        started.elapsed(),
    );
}

fn resident_bytes() -> Option<usize> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmRSS:"))?;
    let kibibytes = line.split_whitespace().nth(1)?.parse::<usize>().ok()?;
    kibibytes.checked_mul(1024)
}

fn format_bytes(bytes: usize) -> String {
    format!("{:.1} MiB", bytes as f64 / (1024.0 * 1024.0))
}
