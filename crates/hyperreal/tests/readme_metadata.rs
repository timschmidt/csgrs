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
    let manifest: toml::Table = include_str!("../Cargo.toml")
        .parse()
        .expect("Cargo.toml is valid TOML");
    let release_dependencies = manifest["dependencies"]
        .as_table()
        .expect("manifest has a normal dependency table");
    let dev_dependencies = manifest["dev-dependencies"]
        .as_table()
        .expect("manifest has a development dependency table");

    assert!(
        !release_dependencies.contains_key("rug"),
        "the GMP binding must not enter Hyperreal's release dependency graph"
    );
    assert!(
        dev_dependencies.contains_key("rug"),
        "competitive GMP benchmarks require rug as a development dependency"
    );
}
