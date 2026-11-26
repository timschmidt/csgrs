//! Opt-in acquisition and retained cache for the external YeahRight benchmark corpus.
//!
//! Runtime requests may invoke curl, unzip and sha256sum. Archive/control bytes
//! are pinned, while a previously derived hull is reused by existence alone.
//! Staging uses process-ID filenames without explicit locks, durability sync or
//! cancellation cleanup; a failed attempt can leave temporary files behind.

use std::{
    env, fs,
    fs::File,
    io::BufReader,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

/// Runtime flag permitting acquisition when its value is an accepted true spelling.
const ENABLE_ENV: &str = "YEAHRIGHT_BENCH";
/// Configured download location for the pinned archive; availability is not checked at compile time.
const ARCHIVE_URL: &str =
    "https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/yeahright.zip";
/// Required SHA-256 digest of the downloaded or cached archive bytes.
const ARCHIVE_SHA256: &str =
    "b3a4f314bfb8c67e36eab5faa96a146ffac90524e120769b5adb8a71be8ba3dc";
/// Required byte count of the extracted controlmesh.obj member.
const CONTROL_MESH_BYTES: u64 = 615_279;
/// Required SHA-256 digest of the extracted control OBJ bytes.
const CONTROL_MESH_SHA256: &str =
    "bf8768bf019ac505c39c92ae5d63808ee1773047aefd5d5d6700b88c1b5f1c3e";

/// Accepts case-insensitive `1`, `true`, `yes` or `on` in YEAHRIGHT_BENCH.
/// Missing/non-Unicode values and all other spellings are false; whitespace is
/// not trimmed. Reading this flag performs no acquisition.
pub fn enabled() -> bool {
    env::var(ENABLE_ENV).ok().is_some_and(|value| {
        matches!(
            value.to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

/// Requires opt-in, validates/acquires the archive and returns the verified control OBJ.
///
/// Creates the cache directory and checks the archive even when a control file
/// already exists. Reuses control bytes only after size and SHA-256 checks;
/// otherwise extracts exactly controlmesh.obj via unzip into a process-specific
/// staging file, validates it and renames it into place. Failures panic and may
/// retain staged files. Stored assets remain cached after successful use.
pub fn control_mesh_path() -> PathBuf {
    assert!(
        enabled(),
        "the YeahRight benchmark is opt-in; set {ENABLE_ENV}=1 to download and run it"
    );

    let directory = fixture_directory();
    fs::create_dir_all(&directory).unwrap_or_else(|error| {
        panic!(
            "failed to create YeahRight benchmark cache {}: {error}",
            directory.display()
        )
    });
    let archive = directory.join("yeahright.zip");
    ensure_archive(&archive);

    let control_mesh = directory.join("controlmesh.obj");
    if control_mesh
        .metadata()
        .is_ok_and(|metadata| metadata.len() == CONTROL_MESH_BYTES)
        && file_hash_matches(&control_mesh, CONTROL_MESH_SHA256)
    {
        return control_mesh;
    }

    let temporary = directory.join(format!("controlmesh.obj.part-{}", std::process::id()));
    let output = File::create(&temporary).unwrap_or_else(|error| {
        panic!(
            "failed to create temporary YeahRight fixture {}: {error}",
            temporary.display()
        )
    });
    let status = Command::new("unzip")
        .arg("-p")
        .arg(&archive)
        .arg("controlmesh.obj")
        .stdout(Stdio::from(output))
        .status()
        .unwrap_or_else(|error| panic!("failed to execute unzip for YeahRight: {error}"));
    assert!(status.success(), "unzip failed while extracting YeahRight");
    let extracted_bytes = temporary
        .metadata()
        .unwrap_or_else(|error| {
            panic!(
                "failed to inspect extracted YeahRight fixture {}: {error}",
                temporary.display()
            )
        })
        .len();
    assert_eq!(
        extracted_bytes, CONTROL_MESH_BYTES,
        "downloaded YeahRight control mesh has an unexpected size"
    );
    assert!(
        file_hash_matches(&temporary, CONTROL_MESH_SHA256),
        "extracted YeahRight control mesh failed its SHA-256 check"
    );
    fs::rename(&temporary, &control_mesh).unwrap_or_else(|error| {
        panic!(
            "failed to install YeahRight fixture {}: {error}",
            control_mesh.display()
        )
    });
    control_mesh
}

/// Reads the verified opt-in control OBJ as UTF-8 text; acquisition/read failures panic.
pub fn control_mesh_source() -> String {
    let path = control_mesh_path();
    fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()))
}

/// Return a serialized convex hull shared by the Rust and native Boolean cases.
///
/// A new hull is derived from the verified full-resolution control mesh and
/// exported through the OBJ projection boundary. Opt-in control/archive checks
/// run first, but an existing hull path is then reused by existence alone: its
/// bytes, geometry and dependency/version freshness are not verified. New writes
/// use staging plus rename; failures panic and may retain the temporary file.
pub fn boolean_hull_path() -> PathBuf {
    let control_mesh = control_mesh_path();
    let hull_path = fixture_directory().join("yeahright_boolean_hull.obj");
    if hull_path.exists() {
        return hull_path;
    }

    let file = File::open(&control_mesh).unwrap_or_else(|error| {
        panic!(
            "failed to open YeahRight control mesh {}: {error}",
            control_mesh.display()
        )
    });
    let source = csgrs::io::obj::from_obj(BufReader::new(file)).unwrap_or_else(|error| {
        panic!(
            "failed to import YeahRight control mesh {}: {error}",
            control_mesh.display()
        )
    });
    let hull = csgrs::solid::convex_hull(&source)
        .expect("YeahRight control points span a three-dimensional hull");
    let serialized = csgrs::io::obj::to_obj(&hull, "yeahright_boolean_hull")
        .expect("YeahRight convex hull is OBJ representable");
    let temporary = hull_path.with_extension(format!("obj.part-{}", std::process::id()));
    fs::write(&temporary, serialized).unwrap_or_else(|error| {
        panic!(
            "failed to serialize derived YeahRight hull {}: {error}",
            temporary.display()
        )
    });
    fs::rename(&temporary, &hull_path).unwrap_or_else(|error| {
        panic!(
            "failed to install derived YeahRight hull {}: {error}",
            hull_path.display()
        )
    });
    hull_path
}

/// Returns the effective target directory's benchmark-fixtures/yeahright subdirectory.
fn fixture_directory() -> PathBuf {
    target_directory().join("benchmark-fixtures/yeahright")
}

/// Resolves runtime CARGO_TARGET_DIR or the compiled manifest directory plus target.
/// Relative overrides use the current working directory and may panic if its
/// lookup fails. Does not canonicalize paths or create directories.
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

/// Reuses a matching archive hash or removes an invalid archive before replacement.
/// Downloads through curl with redirects and three retries, verifies SHA-256,
/// then renames its process-specific staging file. The caller owns opt-in and
/// directory creation; this helper does not recheck either. Failures panic and
/// may leave the old archive removed and a staging file retained.
fn ensure_archive(archive: &Path) {
    if archive.exists() && file_hash_matches(archive, ARCHIVE_SHA256) {
        return;
    }
    if archive.exists() {
        fs::remove_file(archive).unwrap_or_else(|error| {
            panic!(
                "failed to replace invalid YeahRight archive {}: {error}",
                archive.display()
            )
        });
    }

    let temporary = archive.with_extension(format!("zip.part-{}", std::process::id()));
    let status = Command::new("curl")
        .args(["-fL", "--retry", "3", "--output"])
        .arg(&temporary)
        .arg(ARCHIVE_URL)
        .status()
        .unwrap_or_else(|error| panic!("failed to execute curl for YeahRight: {error}"));
    assert!(
        status.success(),
        "failed to download optional YeahRight benchmark fixture from {ARCHIVE_URL}"
    );
    assert!(
        file_hash_matches(&temporary, ARCHIVE_SHA256),
        "downloaded YeahRight archive failed its SHA-256 check"
    );
    fs::rename(&temporary, archive).unwrap_or_else(|error| {
        panic!(
            "failed to install YeahRight archive {}: {error}",
            archive.display()
        )
    });
}

/// Runs sha256sum and compares the first stdout token with the expected digest.
/// A failed process status, non-UTF-8 stdout or mismatch returns false; failure
/// to launch the command panics. Does not validate file size or geometry.
fn file_hash_matches(path: &Path, expected: &str) -> bool {
    let output = Command::new("sha256sum")
        .arg(path)
        .output()
        .unwrap_or_else(|error| panic!("failed to execute sha256sum for YeahRight: {error}"));
    output.status.success()
        && std::str::from_utf8(&output.stdout)
            .ok()
            .and_then(|line| line.split_whitespace().next())
            == Some(expected)
}
