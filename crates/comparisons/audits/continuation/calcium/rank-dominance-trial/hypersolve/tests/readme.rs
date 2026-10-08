const README: &str = include_str!("../README.md");
const QUICKSTART: &str = include_str!("../examples/basic.rs");

#[test]
fn readme_quickstart_matches_the_runnable_example() {
    let start = "<!-- quickstart:start -->\n```rust\n";
    let end = "\n```\n<!-- quickstart:end -->";
    let block = README
        .split_once(start)
        .expect("README must contain the quick-start start marker")
        .1
        .split_once(end)
        .expect("README must contain the quick-start end marker")
        .0;
    assert_eq!(block.trim(), QUICKSTART.trim());
}

#[test]
fn readme_release_metadata_matches_the_manifest() {
    assert!(README.contains(env!("CARGO_PKG_VERSION")));
    for heading in [
        "## Primary types",
        "## Quick start",
        "## API guide",
        "## Guarantees and boundaries",
        "## Feature flags",
        "## References",
        "## Acknowledgements",
        "## License and contributing",
    ] {
        assert!(README.contains(heading), "missing {heading}");
    }
}
