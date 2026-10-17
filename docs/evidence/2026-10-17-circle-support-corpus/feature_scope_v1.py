"""Clarify the reused library's exact Cargo feature selection."""
import hashlib
import json
import tomllib
from pathlib import Path

W = Path(__file__).resolve().parent
OLD = W.parent / "circle-support-index/candidate-validation-v1"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


row = next(r for r in read(OLD / "commands.json") if r["name"] == "build")
assert row["exit_code"] == 0 and "--no-default-features" in row["command"]
assert sha(OLD / "build.stdout") == row["stdout_sha256"]
artifacts = [
    json.loads(line) for line in (OLD / "build.stdout").read_text().splitlines()
    if line.startswith("{")
]
artifact = next(a for a in artifacts if a.get("reason") == "compiler-artifact"
                and a["target"]["name"] == "hypercurve" and a["profile"]["test"])
assert artifact["features"] == []
manifest = W / "candidate-source-v1/crates/hypercurve/Cargo.toml"
defaults = tomllib.loads(manifest.read_text())["features"]["default"]
assert defaults == []
output = W / "default-corpus-v1/tests.stdout"
ignored = [line for line in output.read_text().splitlines() if " ... ignored" in line]
assert len(ignored) == 2
result = {
    "scope_clarification": "The earlier default-feature shorthand denotes the empty optional-feature configuration. The retained binary was actually compiled with --no-default-features; no separate default-feature Cargo invocation is claimed.",
    "build_command": row["command"], "compiled_features": artifact["features"],
    "declared_default_features": defaults, "retained_binary": read(OLD / "binary.json"),
    "unchanged_ignored_tests": ignored,
    "inputs_sha256": {str(p): sha(p) for p in [
        Path(__file__), OLD / "commands.json", OLD / "build.stdout", OLD / "binary.json",
        manifest, output,
    ]},
    "existing_protocol_and_logs_modified": False,
    "default_integration_or_doc_pass_claimed": False,
}
(W / "feature-scope-v1.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({"compiled_features": [], "declared_default_features": defaults, "existing_ignored_tests": len(ignored)}))
