#[test]
fn readme_version_matches_crate_version() {
    let readme = include_str!("../README.md");
    let expected = format!("Version `{}`", env!("CARGO_PKG_VERSION"));
    assert!(
        readme.contains(&expected),
        "README.md should mention the current crate version as {expected}"
    );
}

#[test]
fn gmp_binding_remains_a_development_only_dependency() {
    let manifest = include_str!("../Cargo.toml");
    let (_, after_dependencies) = manifest
        .split_once("[dependencies]")
        .expect("manifest has a normal dependency table");
    let (release_dependencies, after_dev_dependencies) = after_dependencies
        .split_once("[dev-dependencies]")
        .expect("manifest has a development dependency table");
    let dev_dependencies = after_dev_dependencies
        .lines()
        .take_while(|line| !line.starts_with('['))
        .collect::<Vec<_>>()
        .join("\n");

    assert!(
        !release_dependencies
            .lines()
            .any(|line| line.trim_start().starts_with("rug ")),
        "the GMP binding must not enter Hyperreal's release dependency graph"
    );
    assert!(
        dev_dependencies
            .lines()
            .any(|line| line.trim_start().starts_with("rug ")),
        "competitive GMP benchmarks require rug as a development dependency"
    );
}
