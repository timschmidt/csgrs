#![cfg(all(
    feature = "mesh",
    feature = "sketch",
    feature = "image-io",
    feature = "truetype-text",
    feature = "metaballs",
    feature = "sdf",
    feature = "offset"
))]

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

#[test]
fn readme_referenced_png_baselines_exist() {
    let repo = repo_root();
    let referenced = readme_png_references(&repo);

    assert!(
        !referenced.is_empty(),
        "README should reference checked-in docs/*.png images"
    );

    let missing = referenced
        .iter()
        .filter(|name| !repo.join("docs").join(name).exists())
        .cloned()
        .collect::<Vec<_>>();

    assert!(
        missing.is_empty(),
        "README references PNGs missing from docs/: {missing:#?}"
    );
}

#[test]
#[ignore = "expensive byte-for-byte README render regression; run with `cargo test --test readme_render_regression -- --ignored`"]
fn generated_readme_pngs_match_checked_in_baselines() {
    let repo = repo_root();
    let output_dir = fresh_output_dir(&repo);

    let status = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".to_owned()))
        .current_dir(&repo)
        .args(["run", "--quiet", "--example", "readme_renders"])
        .env("README_RENDER_OUTPUT_DIR", &output_dir)
        .status()
        .expect("run readme_renders example");

    assert!(
        status.success(),
        "readme_renders example failed with status {status}"
    );

    let generated = png_names_in(&output_dir);
    assert!(
        !generated.is_empty(),
        "readme_renders did not generate any PNGs in {}",
        output_dir.display()
    );

    let referenced = readme_png_references(&repo);
    let missing_generated = referenced.difference(&generated).cloned().collect::<Vec<_>>();
    assert!(
        missing_generated.is_empty(),
        "README references PNGs that readme_renders did not generate: {missing_generated:#?}"
    );

    let missing_baselines = generated
        .iter()
        .filter(|name| !repo.join("docs").join(name).exists())
        .cloned()
        .collect::<Vec<_>>();
    assert!(
        missing_baselines.is_empty(),
        "readme_renders generated PNGs missing checked-in docs baselines: {missing_baselines:#?}"
    );

    let changed = generated
        .iter()
        .filter_map(|name| {
            let baseline_path = repo.join("docs").join(name);
            let generated_path = output_dir.join(name);
            let baseline = fs::read(&baseline_path).expect("read baseline README PNG");
            let generated = fs::read(&generated_path).expect("read generated README PNG");
            (baseline != generated).then(|| {
                format!(
                    "{name}: baseline={} bytes, generated={} bytes",
                    baseline.len(),
                    generated.len()
                )
            })
        })
        .collect::<Vec<_>>();

    assert!(
        changed.is_empty(),
        "generated README PNGs differ from checked-in baselines:\n{}",
        changed.join("\n")
    );
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn fresh_output_dir(repo: &Path) -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system time before UNIX_EPOCH")
        .as_nanos();
    let path = repo
        .join("target")
        .join("readme-render-regression")
        .join(format!("{}-{nanos}", std::process::id()));
    fs::create_dir_all(&path).expect("create README render regression directory");
    path
}

fn png_names_in(dir: &Path) -> BTreeSet<String> {
    fs::read_dir(dir)
        .unwrap_or_else(|err| panic!("read PNG directory {}: {err}", dir.display()))
        .map(|entry| entry.expect("read PNG directory entry").path())
        .filter(|path| path.extension().is_some_and(|extension| extension == "png"))
        .map(|path| {
            path.file_name()
                .expect("PNG path has file name")
                .to_string_lossy()
                .into_owned()
        })
        .collect()
}

fn readme_png_references(repo: &Path) -> BTreeSet<String> {
    let readme = fs::read_to_string(repo.join("readme.md")).expect("read readme.md");
    let mut references = BTreeSet::new();

    for line in readme.lines() {
        let mut rest = line;
        while let Some(start) = rest.find("docs/") {
            let tail = &rest[start + "docs/".len()..];
            let end = tail
                .find(['"', '\'', ')', '>', '<', ' ', '\t'])
                .unwrap_or(tail.len());
            let name = &tail[..end];
            if name.ends_with(".png") {
                references.insert(name.to_owned());
            }
            rest = &tail[end..];
        }
    }

    references
}
