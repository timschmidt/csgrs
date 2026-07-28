const README: &str = include_str!("../readme.md");
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
    assert!(README.contains("csgrs = \"0.23.0\""));
    for heading in [
        "## What CSGRS owns",
        "## Primary types",
        "## Quick start",
        "## Useful API",
        "## Features",
        "## Guarantees and boundaries",
        "## Performance and hard tests",
        "## References",
        "## Acknowledgements",
        "## License",
    ] {
        assert!(README.contains(heading), "missing {heading}");
    }
    assert!(!README.contains("\n## Roadmap"));
    assert!(!README.contains("\n## Todo"));
    assert!(README.contains("csgrs::adapter"));
    assert!(!README.contains(concat!("csgrs", "-adapter")));
    assert!(!README.contains("[adapters]"));
}
