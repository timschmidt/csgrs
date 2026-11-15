#!/usr/bin/env python3
"""Compiled regressions for release documentation metadata and flag isolation."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


script = Path(__file__).with_name("release-docs.py")
toolchain = sys.argv[1] if len(sys.argv) > 1 else ""
cargo = ["cargo"] + ([toolchain] if toolchain else [])
with tempfile.TemporaryDirectory(prefix="csgrs-release-docs.") as temporary:
    root = Path(temporary)
    (root / "Cargo.toml").write_text(
        '[workspace]\nmembers = ["first", "second"]\nresolver = "3"\n'
    )
    header = root / "header with spaces.html"
    header.write_text('<meta name="release-docs-fixture" content="argument preserved">')
    for name in ("first", "second"):
        package = root / name
        (package / "src").mkdir(parents=True)
        (package / "Cargo.toml").write_text(
            f'[package]\nname = "docs-{name}-fixture"\nversion = "0.0.0"\n'
            'edition = "2024"\n[features]\ndefault = ["default-marker"]\n'
            'default-marker = []\nchosen = []\n[package.metadata.docs.rs]\n'
            + ('all-features = true\nrustdoc-args = ["--cfg", "docsrs", '
               '"--html-in-header", ' + json.dumps(str(header)) + ']\n'
               if name == "first" else
               'no-default-features = true\nfeatures = ["chosen"]\n')
        )
    first = root / "first/src/lib.rs"
    first.write_text('''#[cfg(all(doc, not(docsrs)))]
compile_error!("required documentation cfg was lost");
#[cfg(all(doc, not(all(feature = "chosen", feature = "default-marker"))))]
compile_error!("all-features was lost");
/// Fixture with metadata flags.
pub fn first() {}
''')
    (root / "second/src/lib.rs").write_text('''#[cfg(all(doc, docsrs))]
compile_error!("documentation cfg leaked from previous crate");
#[cfg(all(doc, any(feature = "default-marker", not(feature = "chosen"))))]
compile_error!("explicit feature selection was lost");
/// Fixture without metadata flags.
pub fn second() {}
''')
    env = dict(os.environ, CARGO_NET_OFFLINE="true", RUSTDOCFLAGS="-A warnings",
               CARGO_ENCODED_RUSTDOCFLAGS="-A\x1fwarnings")
    subprocess.run(cargo + ["generate-lockfile", "--offline"], cwd=root,
                   env=env, check=True)
    command = [sys.executable, str(script), f"--toolchain={toolchain}",
               "--target-dir", str(root / "target")]
    names = ["docs-first-fixture", "docs-second-fixture"]

    def run(packages, success=True, diagnostic=None):
        result = subprocess.run(command + packages, cwd=root, env=env,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if (result.returncode == 0) != success:
            raise AssertionError(result.stdout)
        if diagnostic is not None and diagnostic not in result.stdout:
            raise AssertionError(result.stdout)
        print(result.stdout, end="")
        return result.stdout

    run(names)
    first_html = (root / "target/doc/docs_first_fixture/index.html").read_text()
    second_html = (root / "target/doc/docs_second_fixture/index.html").read_text()
    assert 'content="argument preserved"' in first_html
    assert 'content="argument preserved"' not in second_html

    # A successful Cargo run alone is insufficient: inspect its rendered output.
    # The warning gate must survive even explicitly permissive ambient flags.
    original = first.read_text()
    first.write_text(original + '\n/// [missing_release_fixture_symbol]\npub fn broken_link() {}\n')
    run(names, success=False, diagnostic="missing_release_fixture_symbol")
    first.write_text(original)

    output = run(names + ["missing-release-fixture"], success=False,
                 diagnostic="documentation packages missing")
    assert '"command":' not in output, "validation must precede every build"
    manifest = root / "second/Cargo.toml"
    manifest.write_text(manifest.read_text() + 'rustc-args = ["--cfg", "unhandled"]\n')
    output = run(names, success=False, diagnostic="unsupported docs.rs keys")
    assert '"command":' not in output, "must not partly document an invalid selection"
    print("release documentation fixture passed: flags, HTML, isolation, features, failures")
