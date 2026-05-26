#[test]
fn readme_version_matches_crate_version() {
    let readme = include_str!("../README.md");
    let expected = format!("Version `{}`", env!("CARGO_PKG_VERSION"));
    assert!(
        readme.contains(&expected),
        "README.md should mention the current crate version as {expected}"
    );
}
