const README: &str = include_str!("../README.md");
const QUICKSTART: &str = include_str!("../examples/basic.rs");

#[test]
fn readme_quickstart_matches_the_runnable_example() {
    let start = "<!-- quickstart:start -->\n```rust\n";
    let end = "\n```\n<!-- quickstart:end -->";
    let body = README
        .split_once(start)
        .expect("README quick-start start marker")
        .1
        .split_once(end)
        .expect("README quick-start end marker")
        .0;
    assert_eq!(body.trim(), QUICKSTART.trim());
}

#[test]
fn readme_release_metadata_matches_the_manifest() {
    assert!(README.contains("hyperdrc = \"0.3.0\""));
    for heading in [
        "## What HyperDRC is for",
        "## Primary types",
        "## Quick start",
        "## Command-line use",
        "## Useful API",
        "## Guarantees and boundaries",
        "## References",
        "## Acknowledgements",
        "## License",
    ] {
        assert!(README.contains(heading), "missing {heading}");
    }
    assert!(!README.contains("\n## Current Status"));
    assert!(!README.contains("\n## Performance Model"));
}
