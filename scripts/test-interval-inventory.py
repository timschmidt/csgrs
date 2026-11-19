#!/usr/bin/env python3
"""Regression checks for the interval inventory command."""

import csv
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import interval_inventory
from interval_inventory import FIELDS


SCRIPT = Path(__file__).with_name("interval_inventory.py")
CANDIDATE = "lexical candidate; not yet audited"


class InventoryRefresh(unittest.TestCase):
    def setUp(self):
        temp = TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.workspace = Path(temp.name)
        self.repo = self.workspace / "csgrs"
        self.repo.mkdir()
        self.inventory = self.repo / "interval-inventory.tsv"

    def source(self, relative, contents):
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)

    def refresh(self, destination, *, check=True):
        return subprocess.run(
            [sys.executable, SCRIPT, "--workspace", self.workspace, "--output", destination],
            check=check, capture_output=True, text=True,
        )

    def write_review(self, path, name, line):
        row = dict.fromkeys(FIELDS, "")
        row.update(repository="csgrs", path="crates/core/src/lib.rs", line=line,
                   symbol=name, kind="fn", review_status="body reviewed at fixture revision A",
                   semantics="old body and its exact caller", consumers="caller A\tcaller B")
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS, delimiter="\t", lineterminator="\n")
            writer.writeheader()
            writer.writerow(row)
        return row

    def rows(self, path):
        with path.open(newline="") as stream:
            return list(csv.DictReader(stream, delimiter="\t"))

    def test_scan_and_historical_reviews_survive_repeated_refresh(self):
        manifest = "crates/core/Cargo.toml"
        self.source(manifest, '[package]\nname = "hyperinterval"\n')
        self.source("crates/core/src/lib.rs",
                    "\n\nimpl A {\n    pub const fn sqrt() {}\n}\n\n"
                    "impl B {\n    pub fn sqrt() {}\n}\n\nfn sqrt() {}\n")
        self.source("crates/core/tool/Cargo.toml", '[package]\nname = "external-probe"\n')
        self.source("crates/core/tool/src/lib.rs",
                    "pub fn sqrt() {}\n\nfn interval_probe() {}\n")
        self.source("crates/solver/src/lib.rs", "pub fn certify_quadratic_krawczyk_box() {}\n")
        old = self.write_review(self.inventory, "sqrt", "1")
        output = self.workspace / "preview.tsv"
        extra = self.write_review(output, "removed", "2")
        self.refresh(output)
        first = output.read_bytes()
        rows = self.rows(output)
        self.assertEqual([r for r in rows if r["review_status"] == old["review_status"]],
                         [old, extra])
        fresh = [r for r in rows if r["review_status"] == CANDIDATE]
        self.assertEqual([(r["symbol"], r["line"], r["owner"]) for r in fresh],
                         [("sqrt", "4", "hyperinterval"), ("sqrt", "8", "hyperinterval"),
                          ("sqrt", "11", "hyperinterval"),
                          ("interval_probe", "3", "external-probe"),
                          ("certify_quadratic_krawczyk_box", "1", "csgrs")])
        self.assertEqual(fresh[2]["visibility"], "private")
        self.assertIn("crates/core/tool/Cargo.toml", fresh[3]["baseline_tests"])
        self.assertEqual([r["repository"] for r in rows if r["review_status"] == "repository unavailable"],
                         ["alumina-interface", "alumina-firmware"])
        self.refresh(output)
        self.assertEqual(output.read_bytes(), first)
        self.inventory.write_bytes(first)
        self.refresh(self.inventory)
        self.assertEqual(self.inventory.read_bytes(), first)
        output.write_bytes(b"")
        self.refresh(output)
        self.assertEqual(output.read_bytes(), first)

    def test_default_checkout_can_be_a_named_worktree(self):
        checkout = self.workspace / "feature-worktree"
        self.repo.rename(checkout)
        command = checkout / "scripts" / SCRIPT.name
        command.parent.mkdir()
        command.write_bytes(SCRIPT.read_bytes())
        (checkout / "lib.rs").write_text("pub struct PrivateIntervalFixture;\n")
        (self.workspace / "alumina-interface").mkdir()
        output = self.workspace / "preview.tsv"
        subprocess.run([sys.executable, command, "--output", output], check=True, capture_output=True)
        rows = self.rows(output)
        self.assertEqual([(r["repository"], r["symbol"]) for r in rows],
                         [("csgrs", "PrivateIntervalFixture"), ("alumina-firmware", "")])

    def test_extern_functions_keep_their_owner_visibility_and_declaration_line(self):
        self.source("Cargo.toml", '[package]\nname = "hyperlattice-interval"\n')
        self.source("src/lib.rs", 'pub extern "C" fn image(input: i32) -> i32 { input }\n'
                    'pub(crate) unsafe extern "C-unwind" fn interval_hook() {}\n'
                    'extern fn private_bounds() {}\n'
                    'pub unsafe extern\n    "C" fn split() {}\n'
                    'unsafe extern "C" {\n    fn foreign_bounds();\n}\n'
                    'extern crate core as interval_core;\n')
        self.refresh(self.inventory)
        rows = [r for r in self.rows(self.inventory) if r["review_status"] == CANDIDATE]
        self.assertEqual(
            [(r["symbol"], r["kind"], r["visibility"], r["line"], r["owner"]) for r in rows],
            [(name, "fn", visibility, line, "hyperlattice-interval") for name, visibility, line in
             [("image", "pub", "1"), ("interval_hook", "pub(crate)", "2"),
              ("private_bounds", "private", "3"), ("split", "pub", "5"),
              ("foreign_bounds", "private", "7")]],
        )

    def test_source_map_picks_private_algorithms_by_package_and_module(self):
        self.source("math/Cargo.toml", '[package]\nname = "hypersolve"\n')
        self.source("math/src/interval.rs", "fn gauss_jordan() {}\n")
        self.source("math/src/elsewhere.rs", "fn unrelated() {}\n")
        self.source("path/Cargo.toml", '[package]\nname = "hyperpath"\n')
        self.source("path/src/routing/jerk_schedule.rs", "fn propagate() {}\n")
        self.source("path/src/routing_extra/other.rs", "pub fn unrelated() {}\n")
        self.refresh(self.inventory)
        fresh = [r for r in self.rows(self.inventory) if r["review_status"] == CANDIDATE]
        self.assertEqual([(r["symbol"], r["owner"]) for r in fresh],
                         [("gauss_jordan", "hypersolve"), ("propagate", "hyperpath")])

    def test_machine_migration_modules_include_private_declarations(self):
        expected = set()
        for repository, package, files in [
            ("alumina-interface", "alumina-interface-core",
             ["compiler.rs", "direct_motion.rs", "servo_motion.rs", "shared_timing_evidence.rs",
              "graph.rs", "graph/storage.rs", "graph/nested/replay.rs"]),
            ("alumina-firmware", "alumina-foc", ["angle.rs", "pwm.rs"]),
            ("alumina-firmware", "alumina-sim", ["foc.rs"]),
            ("alumina-firmware", "unrelated", []),
        ]:
            # A checkout's directory names need not equal Cargo package names.
            root = self.workspace / repository
            crate = root / ("impl-" + package)
            crate.mkdir(parents=True)
            (crate / "Cargo.toml").write_text(f'[package]\nname = "{package}"\n')
            for filename in set(files) | {"graph_extra/storage.rs", "other/angle.rs", "pwm.rs"}:
                path = crate / "src" / filename
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("fn calibrate() {}\n")
                if filename in files:
                    expected.add((repository, package, path.relative_to(root).as_posix()))
        self.refresh(self.inventory)
        found = [(r["repository"], r["owner"], r["path"]) for r in self.rows(self.inventory)
                 if r["symbol"] == "calibrate"]
        self.assertCountEqual(found, expected)

    def test_failed_refresh_leaves_existing_evidence_intact(self):
        self.write_review(self.inventory, "retired_bounds", "7")
        before = self.inventory.read_bytes()
        self.source("src/lib.rs", "pub fn bounds() {}\n")
        self.source("Cargo.toml", "[package\n")
        self.assertNotEqual(self.refresh(self.inventory, check=False).returncode, 0)
        self.assertEqual(self.inventory.read_bytes(), before)
        self.source("Cargo.toml", '[package]\nname = "csgrs"\n')
        output = self.workspace / "preview.tsv"
        output.write_text("unrecognized\tcolumns\n")
        invalid = output.read_bytes()
        self.assertNotEqual(self.refresh(output, check=False).returncode, 0)
        self.assertEqual(output.read_bytes(), invalid)

    def test_review_edited_during_scan_remains_in_place(self):
        self.write_review(self.inventory, "old", "3")
        scan = interval_inventory.scan

        def update_then_scan(root, repository, reviewed_names):
            if repository == "csgrs":
                self.write_review(self.inventory, "new", "5")
            return scan(root, repository, reviewed_names)

        with patch.object(interval_inventory, "scan", update_then_scan):
            with self.assertRaisesRegex(RuntimeError, "review records changed"):
                interval_inventory.refresh(self.workspace, self.repo, self.inventory)
        self.assertEqual([r["symbol"] for r in self.rows(self.inventory)], ["new"])


if __name__ == "__main__":
    unittest.main()
