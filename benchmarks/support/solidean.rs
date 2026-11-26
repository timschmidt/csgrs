//! Opt-in acquisition and execution of Solidean's published Boolean requests.
//!
//! This benchmark adapter parses OBJ decimals through binary64 before exact scalar
//! ingress. It implements ordered single-consumption requests, not arbitrary reusable
//! expression graphs. Panics report malformed input, acquisition and operation failure;
//! no typed partial-result or cancellation contract is provided.
//!
//! Archive bytes and selected control files are hashed, but reuse checks do not hash
//! every extracted OBJ. Derived dome caches check three filenames only. PID staging
//! is renamed into place without locking or guaranteed cleanup after failure.

use std::{
    env, fs,
    path::{Component, Path, PathBuf},
    process::Command,
    time::Instant,
};

use csgrs::{Real, TriangleMesh, solid::SolidExt};
use hyperlattice::Point3;
use hypermesh::Triangle;

/// Explicit opt-in required before acquiring the repository fixture cache.
const ENABLE_ENV: &str = "SOLIDEAN_BENCH";
/// Authored upstream revision used in URLs and cache names.
const COMMIT: &str = "80900cf867fb690e6c7571880857e82f6f45ae34";
/// HTTPS source of the pinned repository tarball; fetched only on a cache miss/mismatch.
const ARCHIVE_URL: &str = "https://github.com/solidean/bench-blog-data/archive/80900cf867fb690e6c7571880857e82f6f45ae34.tar.gz";
/// Required SHA-256 of the compressed repository archive.
const ARCHIVE_SHA256: &str =
    "4387976b719a4e7f242d8f73130dfa20b0a57cca28ea5ac2b689b6aa7a09851a";
/// Required SHA-256 of the nested dome 7z archive within the repository cache.
const DOME_ARCHIVE_SHA256: &str =
    "1547e32e3810fd23b2a8adf5a183368f2590c5ff1959072e71e0a6c47e3abedd";

/// Relative control file for ten primitive loads and nine differences.
const FIRST_REQUEST: &str =
    "blog/2026/first-benchmark-results-iterated-csg/primitives.request.json";
/// Relative control file for the 225-load, 224-Boolean terrain workload.
const TERRAIN_REQUEST: &str = "blog/2026/terrain-carve-benchmark/0.2_-y.request.json";
/// Relative control file for the 1000-load, 1999-Boolean checker-grid workload.
const CUBE_GRID_REQUEST: &str =
    "blog/2026/iterated-cube-grid-benchmark/checker_iter_even_fill_remove.request.json";
/// Relative nested archive containing the five admitted dome levels.
const DOME_ARCHIVE: &str = "blog/2026/iterated-dome-carve-benchmark/dome10-through-5000.7z";

/// Parsed OBJ coordinates and fan-triangulated faces before exact native ingress.
/// No independent topology or source-fidelity certificate is attached.
#[derive(Clone, Debug)]
pub struct RawMesh {
    /// Decimal tokens parsed as binary64; finiteness is checked only at native ingress.
    positions: Vec<[f64; 3]>,
    /// Zero-based indices checked against the final position count by the parser.
    triangles: Vec<[usize; 3]>,
}

/// Supported request instructions; Boolean operands index earlier instruction results.
#[derive(Clone, Copy, Debug)]
enum Operation {
    /// Converts one parsed mesh into a fresh native carrier.
    Load(
        /// Index in the request's parsed mesh list.
        usize,
    ),
    /// Consumes two distinct earlier outputs and computes their regularized union.
    Union(
        /// Earlier left output index.
        usize,
        /// Earlier right output index.
        usize,
    ),
    /// Consumes two distinct earlier outputs and computes their intersection.
    Intersection(
        /// Earlier left output index.
        usize,
        /// Earlier right output index.
        usize,
    ),
    /// Consumes two distinct earlier outputs and subtracts right from left.
    Difference(
        /// Earlier minuend output index.
        usize,
        /// Earlier subtrahend output index.
        usize,
    ),
}

/// One parsed version-one benchmark run with fixed load/Boolean count assertions.
/// Parsing checks backward references; aliasing and repeated consumption are rejected
/// only during execution. The final instruction is the returned root.
#[derive(Debug)]
pub struct Request {
    /// Document-level identifier retained for diagnostics, not a verified content hash.
    pub id: String,
    /// Single run's case label retained for diagnostics and errors.
    pub case_id: String,
    /// Nonempty ordered instructions; only loads, union, intersection and difference.
    operations: Vec<Operation>,
    /// One raw mesh per load instruction, including repeated paths if present.
    meshes: Vec<RawMesh>,
    /// Parsed non-load instruction count, checked against the workload expectation.
    boolean_count: usize,
    /// Sum of fan-triangulated input face counts over every load occurrence.
    input_triangles: usize,
}

impl Request {
    /// Returns the number of Boolean instructions, excluding native load conversions.
    pub const fn boolean_count(&self) -> usize {
        self.boolean_count
    }

    /// Returns summed loaded face counts, including duplicate mesh loads.
    pub const fn input_triangles(&self) -> usize {
        self.input_triangles
    }

    /// Replays the request with fresh native loads and immediate strict mesh Booleans.
    /// Moves operands out of prior slots and drops them after use; unconsumed outputs
    /// are dropped on return. No local certainty receipts are exposed. Exactness starts
    /// at binary64 ingress, not the original OBJ decimal spellings.
    ///
    /// `SOLIDEAN_BENCH_DIAGNOSTICS=1` prints per-instruction timings and, when enabled,
    /// cumulative dispatch snapshots; callers timing this method include that overhead.
    /// Invalid/nonfinite input, aliased/reused operands or failed operations panic.
    pub fn execute(&self) -> TriangleMesh {
        let diagnostics =
            env::var("SOLIDEAN_BENCH_DIAGNOSTICS").is_ok_and(|value| value == "1");
        let mut values = std::iter::repeat_with(|| None)
            .take(self.operations.len())
            .collect::<Vec<Option<TriangleMesh>>>();

        for (operation_index, operation) in self.operations.iter().copied().enumerate() {
            let started = diagnostics.then(Instant::now);
            let output = match operation {
                Operation::Load(mesh_index) => to_triangle_mesh(&self.meshes[mesh_index]),
                Operation::Union(left, right) => {
                    let (left, right) =
                        take_operands(&mut values, left, right, operation_index);
                    left.try_union(&right).unwrap_or_else(|error| {
                        panic!(
                            "{} operation {operation_index} union failed: {error}",
                            self.case_id
                        )
                    })
                },
                Operation::Intersection(left, right) => {
                    let (left, right) =
                        take_operands(&mut values, left, right, operation_index);
                    left.try_intersection(&right).unwrap_or_else(|error| {
                        panic!(
                            "{} operation {operation_index} intersection failed: {error}",
                            self.case_id
                        )
                    })
                },
                Operation::Difference(left, right) => {
                    let (left, right) =
                        take_operands(&mut values, left, right, operation_index);
                    left.try_difference(&right).unwrap_or_else(|error| {
                        panic!(
                            "{} operation {operation_index} difference failed: {error}",
                            self.case_id
                        )
                    })
                },
            };
            if let Some(started) = started {
                eprintln!(
                    "Solidean {} operation {operation_index}/{} {}: {:?}, {} vertices, {} triangles",
                    self.case_id,
                    self.operations.len() - 1,
                    operation.name(),
                    started.elapsed(),
                    output.positions.len(),
                    output.triangles.len()
                );
                #[cfg(feature = "dispatch-trace")]
                {
                    let snapshot = hyperreal::dispatch_trace::snapshot_trace();
                    eprintln!(
                        "Solidean trace through operation {operation_index}: {:?}; rational={:?}",
                        snapshot.correlation_summary(),
                        snapshot.rational
                    );
                }
            }
            values[operation_index] = Some(output);
        }

        values
            .last_mut()
            .and_then(Option::take)
            .expect("Solidean request has a final output")
    }
}

impl Operation {
    /// Returns the request-format operation name for per-instruction diagnostics.
    const fn name(self) -> &'static str {
        match self {
            Self::Load(_) => "load-mesh",
            Self::Union(_, _) => "boolean-union",
            Self::Intersection(_, _) => "boolean-intersection",
            Self::Difference(_, _) => "boolean-difference",
        }
    }
}

/// Recognizes case-insensitive `1`, `true`, `yes` or `on` without whitespace trimming.
/// Missing/non-Unicode values and other strings leave fixture acquisition disabled.
pub fn enabled() -> bool {
    env::var(ENABLE_ENV).ok().is_some_and(|value| {
        matches!(
            value.to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

/// Acquires/parses the pinned primitive request, asserting ten loads and nine Booleans.
pub fn first_request() -> Request {
    let root = repository_root();
    load_request(&root.join(FIRST_REQUEST), 10, 9)
}

/// Acquires/parses the pinned terrain request, asserting 225 loads and 224 Booleans.
pub fn terrain_request() -> Request {
    let root = repository_root();
    load_request(&root.join(TERRAIN_REQUEST), 225, 224)
}

/// Acquires/parses the checker-grid request, asserting 1000 loads and 1999 Booleans.
pub fn cube_grid_request() -> Request {
    let root = repository_root();
    load_request(&root.join(CUBE_GRID_REQUEST), 1_000, 1_999)
}

/// Acquires a dome request at level 10, 100, 250, 1000 or 5000; other levels panic.
/// Requires `level + 1` loads and `level` Boolean instructions after extraction.
pub fn dome_request(level: usize) -> Request {
    assert!(
        matches!(level, 10 | 100 | 250 | 1_000 | 5_000),
        "published Solidean dome level must be 10, 100, 250, 1000, or 5000"
    );
    let directory = dome_directory(level);
    load_request(&directory.join("request.json"), level + 1, level)
}

/// Sums projected triangle areas and the absolute signed tetrahedral volume sum.
/// Coordinates are converted lossily to binary64; no error bound or closedness proof
/// accompanies the result. Absolute value removes orientation of the aggregate,
/// not of each tetrahedron. Empty faces give `(0, 0)`; invalid indices or conversion
/// failure panic, while floating arithmetic can still produce nonfinite results.
pub fn area_and_volume(mesh: &TriangleMesh) -> (f64, f64) {
    let mut area = 0.0;
    let mut signed_volume = 0.0;
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices().map(|index| {
            let point = &mesh.positions[index];
            [
                point
                    .x
                    .to_f64_lossy()
                    .expect("Solidean x coordinate is finite"),
                point
                    .y
                    .to_f64_lossy()
                    .expect("Solidean y coordinate is finite"),
                point
                    .z
                    .to_f64_lossy()
                    .expect("Solidean z coordinate is finite"),
            ]
        });
        let ab = subtract(b, a);
        let ac = subtract(c, a);
        let normal = cross(ab, ac);
        area += dot(normal, normal).sqrt() * 0.5;
        signed_volume += dot(a, cross(b, c)) / 6.0;
    }
    (area, signed_volume.abs())
}

/// Fingerprints only position/face counts; equal values do not imply equal geometry.
pub fn output_checksum(mesh: &TriangleMesh) -> u64 {
    (mesh.positions.len() as u64).rotate_left(32) ^ mesh.triangles.len() as u64
}

/// Parses one version-one Boolean request, loads its OBJs and verifies expected counts.
/// Requires exactly one nonempty run and supported instruction names. Backward
/// references are checked here; single consumption is checked by `execute`.
/// Unknown extra JSON fields are ignored. I/O, schema, path and count errors panic.
fn load_request(path: &Path, expected_loads: usize, expected_booleans: usize) -> Request {
    let source = fs::read_to_string(path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    let document: serde_json::Value = serde_json::from_str(&source)
        .unwrap_or_else(|error| panic!("failed to parse {}: {error}", path.display()));
    assert_eq!(
        document.get("kind").and_then(serde_json::Value::as_str),
        Some("boolean-benchmark"),
        "{} is not a Boolean benchmark request",
        path.display()
    );
    assert_eq!(
        document.get("version").and_then(serde_json::Value::as_u64),
        Some(1),
        "{} has an unsupported request version",
        path.display()
    );
    let id = required_string(&document, "id", path).to_owned();
    let runs = document
        .get("runs")
        .and_then(serde_json::Value::as_array)
        .unwrap_or_else(|| panic!("{} has no runs array", path.display()));
    assert_eq!(
        runs.len(),
        1,
        "{} must contain exactly one run",
        path.display()
    );
    let run = &runs[0];
    let case_id = required_string(run, "case_id", path).to_owned();
    let encoded_operations = run
        .get("operations")
        .and_then(serde_json::Value::as_array)
        .unwrap_or_else(|| panic!("{} has no operations array", path.display()));
    assert!(
        !encoded_operations.is_empty(),
        "{} has no operations",
        path.display()
    );

    let base = path
        .parent()
        .expect("Solidean request path has a parent directory");
    let mut meshes = Vec::with_capacity(expected_loads);
    let mut operations = Vec::with_capacity(encoded_operations.len());
    let mut boolean_count = 0;
    let mut input_triangles = 0;
    for (operation_index, encoded) in encoded_operations.iter().enumerate() {
        let name = required_string(encoded, "op", path);
        let operation = match name {
            "load-mesh" => {
                let relative = required_string(encoded, "path", path);
                let mesh_path = safe_join(base, relative);
                let mesh = parse_triangle_obj(&mesh_path);
                input_triangles += mesh.triangles.len();
                let mesh_index = meshes.len();
                meshes.push(mesh);
                Operation::Load(mesh_index)
            },
            "boolean-union" => {
                boolean_count += 1;
                let [left, right] = operation_args(encoded, operation_index, path);
                Operation::Union(left, right)
            },
            "boolean-intersection" => {
                boolean_count += 1;
                let [left, right] = operation_args(encoded, operation_index, path);
                Operation::Intersection(left, right)
            },
            "boolean-difference" => {
                boolean_count += 1;
                let [left, right] = operation_args(encoded, operation_index, path);
                Operation::Difference(left, right)
            },
            other => panic!(
                "{} operation {operation_index} uses unsupported operation {other:?}",
                path.display()
            ),
        };
        operations.push(operation);
    }

    assert_eq!(
        meshes.len(),
        expected_loads,
        "{} load count changed",
        path.display()
    );
    assert_eq!(
        boolean_count,
        expected_booleans,
        "{} Boolean count changed",
        path.display()
    );

    Request {
        id,
        case_id,
        operations,
        meshes,
        boolean_count,
        input_triangles,
    }
}

/// Requires exactly two unsigned, usize-representable indices strictly below `index`.
/// Does not reject equal or previously consumed indices; execution checks those.
fn operation_args(encoded: &serde_json::Value, index: usize, path: &Path) -> [usize; 2] {
    let args = encoded
        .get("args")
        .and_then(serde_json::Value::as_array)
        .unwrap_or_else(|| panic!("{} operation {index} has no args", path.display()));
    assert_eq!(
        args.len(),
        2,
        "{} operation {index} must have two args",
        path.display()
    );
    let args = args
        .iter()
        .map(|value| {
            value
                .as_u64()
                .and_then(|value| usize::try_from(value).ok())
                .unwrap_or_else(|| {
                    panic!(
                        "{} operation {index} has a non-index argument",
                        path.display()
                    )
                })
        })
        .collect::<Vec<_>>();
    assert!(
        args.iter().all(|&argument| argument < index),
        "{} operation {index} refers forward",
        path.display()
    );
    [args[0], args[1]]
}

/// Borrows a required JSON string; absence/type mismatch panics with the source path.
fn required_string<'a>(value: &'a serde_json::Value, key: &str, path: &Path) -> &'a str {
    value
        .get(key)
        .and_then(serde_json::Value::as_str)
        .unwrap_or_else(|| panic!("{} has no string field {key:?}", path.display()))
}

/// Parses OBJ `v`/`f` records with binary64 coordinates and fan-triangulated polygons.
/// Ignores other records, extra vertex fields and slash-separated texture/normal
/// indices. Negative indices resolve against positions seen so far; positive indices
/// may reference later vertices. Zero is treated as the current position count and
/// can pass the final bounds check if later vertices exist; it is not explicitly
/// rejected as invalid OBJ syntax. Polygon convexity, planarity and topology are
/// unchecked. Malformed records, short faces and final out-of-range indices panic.
fn parse_triangle_obj(path: &Path) -> RawMesh {
    let source = fs::read_to_string(path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()));
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (line_index, line) in source.lines().enumerate() {
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("v") => {
                let mut coordinate = || {
                    fields
                        .next()
                        .unwrap_or_else(|| {
                            panic!(
                                "{} vertex on line {} is incomplete",
                                path.display(),
                                line_index + 1
                            )
                        })
                        .parse::<f64>()
                        .unwrap_or_else(|error| {
                            panic!(
                                "{} has an invalid coordinate on line {}: {error}",
                                path.display(),
                                line_index + 1
                            )
                        })
                };
                positions.push([coordinate(), coordinate(), coordinate()]);
            },
            Some("f") => {
                let face = fields
                    .map(|field| {
                        let raw = field
                            .split('/')
                            .next()
                            .expect("split always yields the OBJ position index");
                        let index = raw.parse::<isize>().unwrap_or_else(|error| {
                            panic!(
                                "{} has an invalid face index on line {}: {error}",
                                path.display(),
                                line_index + 1
                            )
                        });
                        if index > 0 {
                            usize::try_from(index - 1).expect("positive OBJ index fits usize")
                        } else {
                            positions
                                .len()
                                .checked_sub(index.unsigned_abs())
                                .unwrap_or_else(|| {
                                    panic!(
                                        "{} face index is out of range on line {}",
                                        path.display(),
                                        line_index + 1
                                    )
                                })
                        }
                    })
                    .collect::<Vec<_>>();
                assert!(
                    face.len() >= 3,
                    "{} face on line {} is incomplete",
                    path.display(),
                    line_index + 1
                );
                for index in 1..face.len() - 1 {
                    triangles.push([face[0], face[index], face[index + 1]]);
                }
            },
            _ => {},
        }
    }
    assert!(
        triangles
            .iter()
            .flatten()
            .all(|&index| index < positions.len()),
        "{} has an out-of-range face index",
        path.display()
    );
    RawMesh {
        positions,
        triangles,
    }
}

/// Converts every stored binary64 coordinate exactly and copies triangle indices.
/// Does not independently validate the mesh or recover original decimal semantics.
fn to_triangle_mesh(mesh: &RawMesh) -> TriangleMesh {
    TriangleMesh::new(
        mesh.positions
            .iter()
            .map(|point| Point3::new(real(point[0]), real(point[1]), real(point[2])))
            .collect(),
        mesh.triangles
            .iter()
            .map(|triangle| Triangle::new(triangle[0], triangle[1], triangle[2]))
            .collect(),
    )
}

/// Converts a finite binary64 value to its exact scalar value; NaN/infinity panic.
fn real(value: f64) -> Real {
    Real::try_from(value).expect("Solidean OBJ coordinates are finite")
}

/// Moves distinct still-present operands out of their slots, left before right.
/// Aliasing, invalid indices and repeated consumption panic. If right extraction
/// fails, the left slot has already been emptied; there is no rollback contract.
fn take_operands(
    values: &mut [Option<TriangleMesh>],
    left: usize,
    right: usize,
    operation_index: usize,
) -> (TriangleMesh, TriangleMesh) {
    assert_ne!(
        left, right,
        "Solidean operation {operation_index} aliases its operands"
    );
    let left = values[left].take().unwrap_or_else(|| {
        panic!("Solidean operation {operation_index} reuses operand {left}")
    });
    let right = values[right].take().unwrap_or_else(|| {
        panic!("Solidean operation {operation_index} reuses operand {right}")
    });
    (left, right)
}

/// Requires opt-in, verifies the archive and reuses or extracts its repository cache.
/// Reuse validates three request hashes and the nested 7z hash, not every extracted
/// mesh. Invalid directories are removed before PID-staged `tar` extraction/rename.
/// External commands/I/O/check failures panic and can leave staging behind; there
/// is no concurrent-install lock or durability guarantee.
fn repository_root() -> PathBuf {
    assert!(
        enabled(),
        "the Solidean benchmarks are opt-in; set {ENABLE_ENV}=1 to download and run them"
    );
    let cache = fixture_directory();
    fs::create_dir_all(&cache).unwrap_or_else(|error| {
        panic!(
            "failed to create Solidean benchmark cache {}: {error}",
            cache.display()
        )
    });
    let archive = cache.join(format!("bench-blog-data-{COMMIT}.tar.gz"));
    ensure_archive(&archive);

    let repository = cache.join(format!("bench-blog-data-{COMMIT}"));
    if repository_is_valid(&repository) {
        return repository;
    }
    if repository.exists() {
        fs::remove_dir_all(&repository).unwrap_or_else(|error| {
            panic!(
                "failed to replace invalid Solidean fixture {}: {error}",
                repository.display()
            )
        });
    }

    let staging = cache.join(format!("extract-{COMMIT}.part-{}", std::process::id()));
    if staging.exists() {
        fs::remove_dir_all(&staging).unwrap_or_else(|error| {
            panic!(
                "failed to clear Solidean extraction staging {}: {error}",
                staging.display()
            )
        });
    }
    fs::create_dir_all(&staging).unwrap_or_else(|error| {
        panic!(
            "failed to create Solidean extraction staging {}: {error}",
            staging.display()
        )
    });
    let status = Command::new("tar")
        .args(["-xzf"])
        .arg(&archive)
        .arg("-C")
        .arg(&staging)
        .status()
        .unwrap_or_else(|error| panic!("failed to execute tar for Solidean: {error}"));
    assert!(status.success(), "tar failed while extracting Solidean");
    let extracted = staging.join(format!("bench-blog-data-{COMMIT}"));
    fs::rename(&extracted, &repository).unwrap_or_else(|error| {
        panic!(
            "failed to install Solidean fixture {}: {error}",
            repository.display()
        )
    });
    fs::remove_dir(&staging).unwrap_or_else(|error| {
        panic!(
            "failed to remove Solidean extraction staging {}: {error}",
            staging.display()
        )
    });
    assert!(
        repository_is_valid(&repository),
        "extracted Solidean fixture failed validation"
    );
    repository
}

/// Reuses matching tarball bytes or replaces them through `curl` and a SHA-256 check.
/// Removes a mismatching destination before fetching with three retries. Only a
/// verified temporary file is renamed into place; failure panics without cleanup.
fn ensure_archive(archive: &Path) {
    if archive.exists() && file_hash_matches(archive, ARCHIVE_SHA256) {
        return;
    }
    if archive.exists() {
        fs::remove_file(archive).unwrap_or_else(|error| {
            panic!(
                "failed to replace invalid Solidean archive {}: {error}",
                archive.display()
            )
        });
    }
    let temporary = archive.with_extension(format!("tar.gz.part-{}", std::process::id()));
    let status = Command::new("curl")
        .args(["-fL", "--retry", "3", "--output"])
        .arg(&temporary)
        .arg(ARCHIVE_URL)
        .status()
        .unwrap_or_else(|error| panic!("failed to execute curl for Solidean: {error}"));
    assert!(
        status.success(),
        "failed to download optional Solidean benchmark fixtures from {ARCHIVE_URL}"
    );
    assert!(
        file_hash_matches(&temporary, ARCHIVE_SHA256),
        "downloaded Solidean archive failed its SHA-256 check"
    );
    fs::rename(&temporary, archive).unwrap_or_else(|error| {
        panic!(
            "failed to install Solidean archive {}: {error}",
            archive.display()
        )
    });
}

/// Checks three pinned request files and the dome archive, stopping at first mismatch.
/// This does not verify the complete extraction tree, OBJ payloads or symlink layout.
fn repository_is_valid(repository: &Path) -> bool {
    [
        (
            FIRST_REQUEST,
            "ee43fef18a155550e70bd6ff3e07085143625412fa6b9ef6e0fd666b7db37299",
        ),
        (
            TERRAIN_REQUEST,
            "0094153063aa68ffde86ef902749d82b6f5b09b281f7a9bb586fb169c571bb92",
        ),
        (
            CUBE_GRID_REQUEST,
            "23dc58f46d6e4a7cefaeb2acffe8f9f73c58a72a6b13df18d7985238e971e6d1",
        ),
        (DOME_ARCHIVE, DOME_ARCHIVE_SHA256),
    ]
    .iter()
    .all(|(relative, hash)| file_hash_matches(&repository.join(relative), hash))
}

/// Reuses a dome directory passing presence checks or extracts its members with `7z`.
/// Obtains the verified repository first, removes invalid cache/staging directories,
/// and renames the extracted directory before checking it. Assumes an admitted level;
/// failures panic and may leave temporary state. No per-member hashes are checked.
fn dome_directory(level: usize) -> PathBuf {
    let repository = repository_root();
    let cache = fixture_directory();
    let directory = cache.join(format!("dome-{level}-{COMMIT}"));
    if dome_directory_is_valid(&directory, level) {
        return directory;
    }
    if directory.exists() {
        fs::remove_dir_all(&directory).unwrap_or_else(|error| {
            panic!(
                "failed to replace invalid Solidean dome fixture {}: {error}",
                directory.display()
            )
        });
    }

    let staging = cache.join(format!("dome-{level}.part-{}", std::process::id()));
    if staging.exists() {
        fs::remove_dir_all(&staging).unwrap_or_else(|error| {
            panic!(
                "failed to clear Solidean dome staging {}: {error}",
                staging.display()
            )
        });
    }
    fs::create_dir_all(&staging).unwrap_or_else(|error| {
        panic!(
            "failed to create Solidean dome staging {}: {error}",
            staging.display()
        )
    });
    let output_argument = format!("-o{}", staging.display());
    let member_pattern = format!("dome-{level}/*");
    let status = Command::new("7z")
        .arg("x")
        .arg("-y")
        .arg(output_argument)
        .arg(repository.join(DOME_ARCHIVE))
        .arg(member_pattern)
        .status()
        .unwrap_or_else(|error| panic!("failed to execute 7z for Solidean: {error}"));
    assert!(
        status.success(),
        "7z failed while extracting Solidean dome {level}"
    );
    let extracted = staging.join(format!("dome-{level}"));
    fs::rename(&extracted, &directory).unwrap_or_else(|error| {
        panic!(
            "failed to install Solidean dome fixture {}: {error}",
            directory.display()
        )
    });
    fs::remove_dir(&staging).unwrap_or_else(|error| {
        panic!(
            "failed to remove Solidean dome staging {}: {error}",
            staging.display()
        )
    });
    assert!(
        dome_directory_is_valid(&directory, level),
        "extracted Solidean dome {level} fixture is incomplete"
    );
    directory
}

/// Tests only that request, block and the final numbered tool are files.
/// Intermediate tools, contents, counts and source correspondence are not checked.
fn dome_directory_is_valid(directory: &Path, level: usize) -> bool {
    directory.join("request.json").is_file()
        && directory.join("block.obj").is_file()
        && directory.join(format!("tool_{level:04}.obj")).is_file()
}

/// Appends a path only when every component yielded by Rust is `Normal`.
/// Rejects roots, prefixes and yielded dot/parent components; an empty component
/// sequence passes. This lexical check neither canonicalizes nor confines symlinks.
fn safe_join(base: &Path, relative: &str) -> PathBuf {
    let relative = Path::new(relative);
    assert!(
        relative
            .components()
            .all(|component| matches!(component, Component::Normal(_))),
        "Solidean request contains unsafe mesh path {relative:?}"
    );
    base.join(relative)
}

/// Names the retained `benchmark-fixtures/solidean` cache under the chosen target path.
fn fixture_directory() -> PathBuf {
    target_directory().join("benchmark-fixtures/solidean")
}

/// Resolves `CARGO_TARGET_DIR`, relative to the process directory when necessary.
/// Without it, uses the compile-time manifest's `target`; no directory is created here.
fn target_directory() -> PathBuf {
    match env::var_os("CARGO_TARGET_DIR") {
        Some(directory) => {
            let directory = PathBuf::from(directory);
            if directory.is_absolute() {
                directory
            } else {
                env::current_dir()
                    .expect("benchmark working directory is available")
                    .join(directory)
            }
        },
        None => Path::new(env!("CARGO_MANIFEST_DIR")).join("target"),
    }
}

/// Compares the first UTF-8 stdout token of successful `sha256sum` with `expected`.
/// Spawn/status/encoding failures return false; no typed I/O diagnostic is retained.
fn file_hash_matches(path: &Path, expected: &str) -> bool {
    let Ok(output) = Command::new("sha256sum").arg(path).output() else {
        return false;
    };
    output.status.success()
        && std::str::from_utf8(&output.stdout)
            .ok()
            .and_then(|line| line.split_whitespace().next())
            == Some(expected)
}

/// Componentwise binary64 difference used for projected triangle edge vectors.
fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

/// Rounded 3D cross product for projected area and signed-volume sums.
fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

/// Rounded 3D dot product with ordinary arithmetic and no certified error bound.
fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}
