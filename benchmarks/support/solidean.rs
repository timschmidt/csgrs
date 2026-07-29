//! Opt-in acquisition and execution of Solidean's published Boolean requests.

use std::{
    env, fs,
    path::{Component, Path, PathBuf},
    process::Command,
    time::Instant,
};

use csgrs::{Real, TriangleMesh, solid::SolidExt};
use hyperlattice::Point3;
use hypermesh::Triangle;

const ENABLE_ENV: &str = "SOLIDEAN_BENCH";
const COMMIT: &str = "80900cf867fb690e6c7571880857e82f6f45ae34";
const ARCHIVE_URL: &str = "https://github.com/solidean/bench-blog-data/archive/80900cf867fb690e6c7571880857e82f6f45ae34.tar.gz";
const ARCHIVE_SHA256: &str =
    "4387976b719a4e7f242d8f73130dfa20b0a57cca28ea5ac2b689b6aa7a09851a";
const DOME_ARCHIVE_SHA256: &str =
    "1547e32e3810fd23b2a8adf5a183368f2590c5ff1959072e71e0a6c47e3abedd";

const FIRST_REQUEST: &str =
    "blog/2026/first-benchmark-results-iterated-csg/primitives.request.json";
const TERRAIN_REQUEST: &str = "blog/2026/terrain-carve-benchmark/0.2_-y.request.json";
const CUBE_GRID_REQUEST: &str =
    "blog/2026/iterated-cube-grid-benchmark/checker_iter_even_fill_remove.request.json";
const DOME_ARCHIVE: &str = "blog/2026/iterated-dome-carve-benchmark/dome10-through-5000.7z";

#[derive(Clone, Debug)]
pub struct RawMesh {
    positions: Vec<[f64; 3]>,
    triangles: Vec<[usize; 3]>,
}

#[derive(Clone, Copy, Debug)]
enum Operation {
    Load(usize),
    Union(usize, usize),
    Intersection(usize, usize),
    Difference(usize, usize),
}

#[derive(Debug)]
pub struct Request {
    pub id: String,
    pub case_id: String,
    operations: Vec<Operation>,
    meshes: Vec<RawMesh>,
    boolean_count: usize,
    input_triangles: usize,
}

impl Request {
    pub const fn boolean_count(&self) -> usize {
        self.boolean_count
    }

    pub const fn input_triangles(&self) -> usize {
        self.input_triangles
    }

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
    const fn name(self) -> &'static str {
        match self {
            Self::Load(_) => "load-mesh",
            Self::Union(_, _) => "boolean-union",
            Self::Intersection(_, _) => "boolean-intersection",
            Self::Difference(_, _) => "boolean-difference",
        }
    }
}

pub fn enabled() -> bool {
    env::var(ENABLE_ENV).ok().is_some_and(|value| {
        matches!(
            value.to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

pub fn first_request() -> Request {
    let root = repository_root();
    load_request(&root.join(FIRST_REQUEST), 10, 9)
}

pub fn terrain_request() -> Request {
    let root = repository_root();
    load_request(&root.join(TERRAIN_REQUEST), 225, 224)
}

pub fn cube_grid_request() -> Request {
    let root = repository_root();
    load_request(&root.join(CUBE_GRID_REQUEST), 1_000, 1_999)
}

pub fn dome_request(level: usize) -> Request {
    assert!(
        matches!(level, 10 | 100 | 250 | 1_000 | 5_000),
        "published Solidean dome level must be 10, 100, 250, 1000, or 5000"
    );
    let directory = dome_directory(level);
    load_request(&directory.join("request.json"), level + 1, level)
}

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

pub fn output_checksum(mesh: &TriangleMesh) -> u64 {
    (mesh.positions.len() as u64).rotate_left(32) ^ mesh.triangles.len() as u64
}

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

fn required_string<'a>(value: &'a serde_json::Value, key: &str, path: &Path) -> &'a str {
    value
        .get(key)
        .and_then(serde_json::Value::as_str)
        .unwrap_or_else(|| panic!("{} has no string field {key:?}", path.display()))
}

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

fn real(value: f64) -> Real {
    Real::try_from(value).expect("Solidean OBJ coordinates are finite")
}

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

fn dome_directory_is_valid(directory: &Path, level: usize) -> bool {
    directory.join("request.json").is_file()
        && directory.join("block.obj").is_file()
        && directory.join(format!("tool_{level:04}.obj")).is_file()
}

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

fn fixture_directory() -> PathBuf {
    target_directory().join("benchmark-fixtures/solidean")
}

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

fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}
