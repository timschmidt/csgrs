//! Opt-in acquisition for the external YeahRight benchmark corpus.

use std::{
    env, fs,
    fs::File,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

const ENABLE_ENV: &str = "YEAHRIGHT_BENCH";
const ARCHIVE_URL: &str = "https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/yeahright.zip";
const ARCHIVE_SHA256: &str = "b3a4f314bfb8c67e36eab5faa96a146ffac90524e120769b5adb8a71be8ba3dc";
const CONTROL_MESH_BYTES: u64 = 615_279;
const CONTROL_MESH_SHA256: &str =
    "bf8768bf019ac505c39c92ae5d63808ee1773047aefd5d5d6700b88c1b5f1c3e";

pub fn enabled() -> bool {
    env::var(ENABLE_ENV).ok().is_some_and(|value| {
        matches!(
            value.to_ascii_lowercase().as_str(),
            "1" | "true" | "yes" | "on"
        )
    })
}

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

pub fn control_mesh_source() -> String {
    let path = control_mesh_path();
    fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", path.display()))
}

fn fixture_directory() -> PathBuf {
    target_directory().join("benchmark-fixtures/yeahright")
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
        }
        None => Path::new(env!("CARGO_MANIFEST_DIR")).join("target"),
    }
}

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
