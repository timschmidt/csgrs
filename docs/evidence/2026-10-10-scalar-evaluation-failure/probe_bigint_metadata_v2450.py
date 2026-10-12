"""Prove actual BigInt sign/magnitude bodies against private representation views."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
OUT = E / "bigint-metadata-export-v2450"
WORK = ROOT / "target/bigint-metadata-export-v2450"
PREVIOUS = E / "biguint-crate-export-v2442"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

GHOST = '''#[cfg(verus_keep_ghost)]
use vstd::prelude::*;

#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
impl BigInt {
    pub closed spec fn metadata_sign(&self) -> Sign {
        self.sign
    }

    pub closed spec fn metadata_magnitude(&self) -> nat {
        self.data.metadata_value()
    }

    pub closed spec fn metadata_storage_bits(&self) -> nat {
        self.data.metadata_storage_bits()
    }

    pub closed spec fn metadata_normalized(&self) -> bool {
        self.data.metadata_normalized()
            && (self.sign == Sign::NoSign <==> self.data.metadata_value() == 0)
    }

    pub closed spec fn metadata_value(&self) -> int {
        match self.sign {
            Sign::Minus => -(self.data.metadata_value() as int),
            Sign::NoSign => 0,
            Sign::Plus => self.data.metadata_value() as int,
        }
    }
}
}

'''
SIGN = '''    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result == self.metadata_sign(),
            self.metadata_normalized() ==> match result {
                Sign::Minus => self.metadata_value() < 0,
                Sign::NoSign => self.metadata_value() == 0,
                Sign::Plus => self.metadata_value() > 0,
            },
    ))]
    #[inline]
    pub fn sign(&self) -> Sign {
        proof! {
            reveal(BigInt::metadata_sign);
            reveal(BigInt::metadata_normalized);
            reveal(BigInt::metadata_value);
        }
        self.sign
    }'''
MAGNITUDE = '''    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        ensures result.metadata_value() == self.metadata_magnitude(),
            result.metadata_storage_bits() == self.metadata_storage_bits(),
            self.metadata_normalized() ==> result.metadata_normalized(),
            self.metadata_normalized() ==> result.metadata_value() ==
                if self.metadata_value() < 0 { (-self.metadata_value()) as nat }
                else { self.metadata_value() as nat },
    ))]
    #[inline]
    pub fn magnitude(&self) -> &BigUint {
        proof! {
            reveal(BigInt::metadata_magnitude);
            reveal(BigInt::metadata_storage_bits);
            reveal(BigInt::metadata_normalized);
            reveal(BigInt::metadata_value);
        }
        &self.data
    }'''


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    previous = read(PREVIOUS / "protocol.json")
    assert read(PREVIOUS / "result.json")["exit_code"] == 0
    old_vendor = Path(previous["vendor"])
    assert files(old_vendor) == previous["vendor_files_sha256"]
    tool = Path(previous["command"][0]).parent
    assert all(sha(tool / n) == h for n, h in previous["tool_sha256"].items())
    dependencies = read((E / checkpoint["source_record"]).parent / "proof-reports/dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    OUT.mkdir()
    vendor = WORK / "vendor/num-bigint"
    shutil.copytree(old_vendor, vendor)
    bigint = vendor / "src/bigint.rs"
    old = bigint.read_text()
    new = old
    for before, after in [
        ("pub enum Sign {", "#[cfg_attr(verus_keep_ghost, verus_verify)]\npub enum Sign {"),
        ("pub struct BigInt {", "#[cfg_attr(verus_keep_ghost, verus_verify)]\npub struct BigInt {"),
        ("// Note: derived `Clone` doesn't specialize `clone_from`,", GHOST + "// Note: derived `Clone` doesn't specialize `clone_from`,"),
        ("    #[inline]\n    pub fn sign(&self) -> Sign {\n        self.sign\n    }", SIGN),
        ("    #[inline]\n    pub fn magnitude(&self) -> &BigUint {\n        &self.data\n    }", MAGNITUDE),
    ]:
        assert new.count(before) == 1, before
        new = new.replace(before, after)
    bigint.write_text(new)
    (OUT / "bigint.rs").write_text(new)
    (OUT / "metadata.patch").write_text("".join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
        fromfile="v2442/src/bigint.rs", tofile="v2450/src/bigint.rs", n=0)))
    inputs = files(vendor)
    assert [n for n, h in inputs.items() if previous["vendor_files_sha256"].get(n) != h] == ["src/bigint.rs"]
    artifacts = WORK / "artifacts"
    artifacts.mkdir()
    command = [str(tool / "verus"), "--edition=2021", "--crate-type=lib", "--crate-name", "num_bigint",
               "--cfg", 'feature="std"', "--cfg", 'feature="default"', "--no-cheating", "--output-json",
               "--export", str(artifacts / "num_bigint.vir"), "--compile", "--out-dir", str(artifacts)]
    for name in ["num_integer", "num_traits"]:
        libraries = [n for n in dependencies["artifacts"] if Path(n).name.startswith("lib" + name + "-") and n.endswith(".rlib")]
        assert len(libraries) == 1
        command += ["--extern", name + "=" + libraries[0]]
    command += dependencies["verus_arguments"][-2:] + [str(vendor / "src/lib.rs")]
    protocol = {"status": "Frozen before actual BigInt metadata proof and crate export",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "previous_export_protocol_sha256": sha(PREVIOUS / "protocol.json"),
        "previous_vendor_files_sha256": previous["vendor_files_sha256"], "vendor": str(vendor),
        "vendor_files_sha256": inputs, "changed_files": ["src/bigint.rs"],
        "archived_bigint_sha256": sha(OUT / "bigint.rs"), "patch_sha256": sha(OUT / "metadata.patch"),
        "command": command, "tool_sha256": previous["tool_sha256"], "dependency_artifacts_sha256": dependencies["artifacts"],
        "scope": "Mark the actual Sign enum and BigInt representation for verification, introduce closed views of private fields and annotate the existing sign/magnitude methods. Runtime expressions still return self.sign and &self.data. Numerical sign and absolute-value conclusions are conditional on an explicit normalized/sign-consistent representation predicate; storage count bounds are not assumed. All other upstream bodies remain outside verification. No SOURCE or main edit, trust escape hatch, runtime admission or retained proof-count increase follows."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    run = subprocess.run(command, cwd=ROOT, env=os.environ, capture_output=True, text=True)
    (OUT / "stdout.json").write_text(run.stdout)
    (OUT / "stderr.log").write_text(run.stderr)
    assert files(vendor) == inputs and files(old_vendor) == previous["vendor_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    assert all(sha(tool / n) == h for n, h in previous["tool_sha256"].items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    try:
        parsed = json.loads(run.stdout).get("verification-results", {})
    except json.JSONDecodeError:
        parsed = {}
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start, "verification_results": parsed,
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.json"),
        "stderr_sha256": sha(OUT / "stderr.log"), "emitted_artifacts_sha256": files(artifacts),
        "source_unchanged": True, "main_unchanged": True, "vendor_unchanged": True,
        "tooling_unchanged": True, "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
