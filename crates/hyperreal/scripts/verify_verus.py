#!/usr/bin/env python3
"""Install the pinned verifier or check Hyperreal's current production proofs."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import zipfile

from check_verification_acceptance import check_acceptance

ROOT = Path(__file__).resolve().parents[1]
PIN = json.loads((ROOT / "verification/toolchain.json").read_text())
SCOPE = json.loads((ROOT / "verification/scope.json").read_text())
INSTALL = ROOT / "target/verus" / PIN["version"]
DEFAULT_VERUS = INSTALL / "verus-x86-linux/verus"
REPORTS = ROOT / "target/verification"


def sha256(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def verifier():
    path = Path(os.environ.get("VERUS", DEFAULT_VERUS)).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError("Verus is missing; run python3 scripts/verify_verus.py install or set VERUS")
    result = subprocess.run(
        [str(path), "--version", "--output-json"], check=True, text=True, capture_output=True
    )
    version = json.loads(result.stdout)["verus"]
    if version["version"] != PIN["version"] or version["commit"] != PIN["commit"]:
        raise RuntimeError(f"Expected pinned Verus {PIN['version']}, got {version}")
    return path


def install(archive):
    if (platform.system(), platform.machine()) != ("Linux", "x86_64"):
        raise RuntimeError("The checked-in installer currently pins Linux x86_64 only")
    installed = subprocess.run(
        ["rustup", "toolchain", "list"], check=True, text=True, capture_output=True
    ).stdout
    if PIN["rust_toolchain"] not in installed.split():
        subprocess.run(
            ["rustup", "toolchain", "install", PIN["rust_toolchain"], "--profile", "minimal", "--no-self-update"],
            check=True,
        )
    if DEFAULT_VERUS.exists():
        version = subprocess.run(
            [str(DEFAULT_VERUS), "--version", "--output-json"], check=True, text=True, capture_output=True
        )
        if json.loads(version.stdout)["verus"]["commit"] != PIN["commit"]:
            raise RuntimeError(f"Unexpected existing installation at {INSTALL}")
        print(DEFAULT_VERUS)
        return
    INSTALL.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="install-", dir=INSTALL.parent) as temporary:
        temporary = Path(temporary)
        if archive is None:
            archive = temporary / PIN["archive"]
            subprocess.run(
                ["curl", "--fail", "--location", "--proto", "=https", "--tlsv1.2",
                 "--retry", "3", "--output", str(archive), PIN["url"]],
                check=True,
            )
        if sha256(archive) != PIN["sha256"]:
            raise RuntimeError("Verus archive SHA-256 does not match the pinned upstream asset")
        unpacked = temporary / "unpacked"
        unpacked.mkdir()
        with zipfile.ZipFile(archive) as bundle:
            for entry in bundle.infolist():
                destination = (unpacked / entry.filename).resolve()
                if not destination.is_relative_to(unpacked):
                    raise RuntimeError(f"Invalid archive path: {entry.filename}")
                bundle.extract(entry, unpacked)
                if not entry.is_dir():
                    # Keep only ordinary file permission bits from the archive.
                    destination.chmod((entry.external_attr >> 16) & 0o777 or 0o644)
        for name in ["verus", "rust_verify", "z3", "cargo-verus"]:
            (unpacked / "verus-x86-linux" / name).chmod(0o755)
        shutil.move(str(unpacked), INSTALL)
    print(DEFAULT_VERUS)


def verify(require_complete):
    path = verifier()
    def source_hashes():
        sources = sorted(ROOT.glob("src/**/*.rs")) + sorted(ROOT.glob("verification/*"))
        sources += [Path(__file__).resolve(), ROOT / "scripts/check_verification_acceptance.py",
                    ROOT / "Cargo.toml", ROOT / "Cargo.lock"]
        return {str(p.relative_to(ROOT)): sha256(p) for p in sources if p.is_file()}

    before = source_hashes()
    REPORTS.mkdir(parents=True, exist_ok=True)
    command = [
        str(path), "--edition=2024", "--crate-type=lib", "--crate-name=hyperreal_proofs",
        "--no-cheating", "--output-json", "verification/lib.rs",
    ]
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    (REPORTS / "verus.json").write_text(result.stdout)
    (REPORTS / "verus.stderr").write_text(result.stderr)
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    if result.returncode:
        raise RuntimeError(f"Verus failed (exit {result.returncode}); see {REPORTS}")
    report = json.loads(result.stdout)
    verified = report["verification-results"]
    if not (
        verified["success"] and verified["is-verifying-entire-crate"]
        and verified["verified"] > 0 and verified["errors"] == 0
        and not verified["encountered-error"] and not verified["encountered-vir-error"]
    ):
        raise RuntimeError(f"Incomplete or failed verification: {verified}")
    details = report["func-details"]
    for name in SCOPE["verified_executable_contracts"] + SCOPE["verified_model_theorems"]:
        if f"hyperreal_proofs::{name}" not in details:
            raise RuntimeError(f"Expected contract or theorem missing from Verus output: {name}")
    if before != source_hashes():
        raise RuntimeError("Sources changed while Verus was running; rerun verification")
    evidence = {
        "command": command,
        "verus": report["verus"],
        "verification_results": verified,
        "sources": before,
        "scope": SCOPE,
    }
    (REPORTS / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(f"Verus: {verified['verified']} verification units verified, 0 errors; "
          f"{len(SCOPE['verified_executable_contracts'])} production contracts; "
          f"{len(SCOPE['verified_model_theorems'])} arithmetic model theorems.")
    pending = [item for item in SCOPE["obligations"] if item["status"] != "complete"]
    if pending:
        print(f"Full-crate proof incomplete: {len(pending)} obligation groups remain open.")
    if require_complete:
        failures = ["The full-crate proof obligation audit is incomplete"] if pending else []
        try:
            check_acceptance(ROOT)
        except (RuntimeError, OSError, ValueError, TypeError) as error:
            failures.append(str(error))
        if failures:
            raise RuntimeError("The 100% completion gate is not satisfied: " + "; ".join(failures))
        if before != source_hashes():
            raise RuntimeError("Sources changed while checking baseline acceptance; rerun verification")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    installer = commands.add_parser("install", help="install the pinned official Verus release")
    installer.add_argument("--archive", type=Path, help="use a local archive, still checking its SHA-256")
    checker = commands.add_parser("verify", help="verify all currently connected proof modules")
    checker.add_argument("--require-complete", action="store_true", help="also require the full proof audit and source-bound baseline qualification")
    args = parser.parse_args()
    if args.command == "install":
        install(args.archive)
    else:
        verify(args.require_complete)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError, OSError, ValueError, KeyError) as error:
        sys.exit(f"error: {error}")
