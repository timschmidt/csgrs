#!/usr/bin/env python3
"""Run Hypersolve libtests, fresh-process replays and Horner comparisons on WASI Preview 1."""

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

from wasi_libtest import RUNNER, TARGET, build_tests, compare_dispatch, completed, execute, run_replay_child, test_names

# These interval suites are required even if a Cargo manifest changes both inventories.
INTERVAL_SUITES = {
    "coefficient_bit_budget", "expression_interval", "interval_box_radius", "interval_quadratic_extrema", "jacobian_matrix",
    "krawczyk_affine_oracle", "krawczyk_affine_rank", "krawczyk_affine_residuals", "krawczyk_certificates",
    "krawczyk_multivariate_certainty", "krawczyk_univariate_algebraic", "program_jacobian_trace",
    "real_interval_enclosures", "real_interval_migration_contract", "trivariate_arithmetic",
}

# Native parents retain their input generation, invalid-input checks and output assertions.
# Each row specifies the exact guest child, the input it consumes and the expected call count.
# None selects the parent's own test name.
REPLAYS = {
    "bivariate_vertical_components": {
        "vertical_source_replays_and_restricts_in_a_fresh_process": (None, "CSGRS_R06_ARCHIVE_CHILD", 1)},
    "expression_program": {
        "fresh_process_replay_retains_symbolic_constants_and_revalidates_edges": (
            "replay_child", "HYPERSOLVE_PROGRAM_REPLAY", 2),
        "tree_import_preserves_warm_scalar_dispatch_on_the_polynomial_fixture": (
            None, "HYPERSOLVE_PROGRAM_TRACE_CHILD", 1)},
    "ordered_field_resultant": {
        "eliminated_coefficients_replay_cold_and_reselect_their_dependent_root": (
            None, "HYPERSOLVE_FIELD_RESULTANT_REPLAY", 1)},
    "program_composition": {
        "replay_revalidates_both_original_sources_and_the_binding_selection": (
            None, "HYPERSOLVE_COMPOSITION_REPLAY", 1)},
    "program_coordinates": {
        "fresh_process_replay_rebuilds_inverse_and_source_association": (None, "HYPERSOLVE_COORDINATE_REPLAY", 2)},
    "program_jacobian": {
        "fresh_process_replay_regenerates_partials_from_source_and_variables": (None, "HYPERSOLVE_JACOBIAN_REPLAY", 1)},
    "program_taylor_affine": {
        "fresh_process_replay_validates_source_and_composed_map": (None, "HYPERSOLVE_TAYLOR_AFFINE_REPLAY", 2)},
    "program_taylor_composition": {
        "fresh_process_replay_rebuilds_both_certificates_and_generated_chain": (
            None, "HYPERSOLVE_TAYLOR_COMPOSITION_REPLAY", 2)},
    "program_taylor_model": {
        "fresh_process_replay_rebuilds_source_coefficients_and_remainder": (None, "HYPERSOLVE_TAYLOR_REPLAY", 2)},
    "program_taylor_polynomial": {
        "fresh_process_replay_reconstructs_every_source_and_map": (None, "HYPERSOLVE_POLYNOMIAL_MODEL_REPLAY", 2)},
    "selected_chamfer_enclosure": {
        "second_chamfer_scalar_enclosure_keeps_its_selected_graph": (None, "HYPERSOLVE_CHAMFER_ENCLOSURE_CHILD", 1)},
    "selected_polynomial_fields": {
        "polynomial_inverse_over_selected_coefficients_keeps_replay": (
            "selected_polynomial_inverse_replay_child", "HYPERSOLVE_FIELD_INVERSE_REPLAY", 4)},
    "selected_polynomial_join_retention": {
        "join_correspondences_survive_fresh_process_replay_and_further_joins": (
            None, "HYPERSOLVE_JOIN_CORRESPONDENCE_INPUT", 1),
        "equal_generator_reduction_replays_fresh_with_original_selections_and_retries": (
            None, "HYPERSOLVE_EQUAL_GENERATOR_REPLAY_INPUT", 1)},
    "selected_polynomial_replay": {
        "graph_encoding_shares_repeated_outputs_and_replays_in_a_fresh_process": (
            "replay_in_fresh_process_child", "HYPERSOLVE_SELECTED_REPLAY_TEST_INPUT", 1)},
    "selected_polynomial_root_discovery": {
        "discovered_roots_replay_in_a_fresh_process_and_feed_further_discovery": (
            "discovered_root_replay_child", "HYPERSOLVE_ROOT_DISCOVERY_REPLAY", 1)},
}

# Retain complete model, scalar, diagnostic and trace rows, including work counters.
RECORDS = {
    "HYPERSOLVE_PROGRAM_REPLAY": ("REPLAY=", "INTERVAL_REPLAY=", "INVALID_REFERENCE"),
    "HYPERSOLVE_PROGRAM_TRACE_CHILD": ("polynomial trace: ",),
    "HYPERSOLVE_COORDINATE_REPLAY": ("CHART ",),
    "HYPERSOLVE_JACOBIAN_REPLAY": ("JACOBIAN_REPLAY=",),
    "HYPERSOLVE_TAYLOR_AFFINE_REPLAY": ("AFFINE ",),
    "HYPERSOLVE_TAYLOR_COMPOSITION_REPLAY": ("MODEL ",),
    "HYPERSOLVE_TAYLOR_REPLAY": ("MODEL ",),
    "HYPERSOLVE_POLYNOMIAL_MODEL_REPLAY": ("MODEL ",),
}

THREAD_FAILURE = "kind: Unsupported"
HOST_CASES = {
    "hypersolve": {
        "work_budget::tests::cancellation_from_another_thread_reaches_a_running_operation": THREAD_FAILURE,
        "work_budget::tests::threads_start_without_a_budget": THREAD_FAILURE,
        "work_budget::tests::scopes_restore_after_unwinding_without_refunding_admitted_work": "exercise budget restoration"},
    "expression_program": {"deep_source_compiles_evaluates_and_drops_on_a_small_stack": THREAD_FAILURE},
    "program_jacobian": {"shared_source_graphs_expand_linearly_and_deep_graphs_use_a_small_stack": THREAD_FAILURE},
}
HORNER = {
    "algebraic::interval_migration::rational_horner_dispatch_differential": 1404,
    "root_isolation::interval_migration::rational_horner_exclusion_dispatch_differential": 2808,
}


def replay_child(args):
    key = os.environ["SOLVER_WASI_INPUT"]
    value = os.environ[key]
    guest = os.environ["SOLVER_WASI_GUEST"]
    case = os.environ["SOLVER_WASI_CHILD"]
    if args != ["--exact", case, "--nocapture"]:
        raise SystemExit(f"Unexpected child arguments for {case}: {args}")
    options = ["--env", key, value]
    if value.startswith("/"):
        source = Path(value)
        if source.parent != Path(os.environ["TMPDIR"]) or not source.is_file():
            raise SystemExit(f"Missing replay fixture for {case}")
        options = ["--env", key, "/replay/" + source.name, "--replay-dir", str(source.parent)]
    receipt = Path(os.environ["SOLVER_WASI_RECEIPT"])
    lines = run_replay_child(guest, args, case, options, receipt)
    if key in RECORDS:
        # Use the original host path for file inputs, including the parent's
        # second (invalid) graph. Only the WASI child needs a preopened mount.
        with tempfile.TemporaryDirectory() as temp:
            native = execute([os.environ["SOLVER_NATIVE_TEST"], *args], Path(temp) / "native.log")
        if native.returncode or not completed(native.stdout, {case}):
            raise SystemExit(native.stdout)
        host_lines = (line.removeprefix(f"test {case} ... ") for line in native.stdout.splitlines())
        expected = [line for line in host_lines if line.startswith(RECORDS[key])]
        actual = [line for line in lines if line.startswith(RECORDS[key])]
        if not expected or actual != expected:
            raise SystemExit(f"Fresh {case} records differ. Native {expected!r}, WASI {actual!r}")
        with receipt.with_suffix(".records.jsonl").open("a") as evidence:
            print(json.dumps(actual), file=evidence)


def run_parent(output, suite, case, child, key, count, guest, host):
    stem = output / f"{suite}.{case}"
    receipt = Path(str(stem) + ".children")
    log = Path(str(stem) + ".log")
    with tempfile.TemporaryDirectory(dir=output) as temp:
        env = os.environ | {"TMPDIR": temp, "SOLVER_WASI_GUEST": guest, "SOLVER_WASI_INPUT": key,
                            "SOLVER_WASI_CHILD": child, "SOLVER_WASI_RECEIPT": str(receipt), "SOLVER_NATIVE_TEST": host}
        env.pop(key, None)
        run = execute([host, "--exact", case, "--nocapture"], log, env)
    if run.returncode or not completed(run.stdout, {case}):
        raise SystemExit(f"Failed native parent or guest assertion for {case}. See {log}")
    calls = receipt.read_text().splitlines() if receipt.is_file() else []
    if calls != [child] * count:
        raise SystemExit(f"Expected {count} completed WASI calls to {child}, got {calls}")
    if key in RECORDS:
        records = [json.loads(line) for line in receipt.with_suffix(".records.jsonl").read_text().splitlines()]
        if len(records) != count or not all(records):
            raise SystemExit(f"Missing native/WASI evidence records for {case}")
        return sum(map(len, records))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cargo = ["cargo", f"+{args.toolchain}", "test", "--locked", "--release", "-p", "hypersolve", "--all-features"]
    env = os.environ.copy()
    env.pop("HYPERSOLVE_REPLAY_DRIVER", None)
    selection = ["--lib", "--tests"]
    guests = build_tests([*cargo, "--target", TARGET], selection, env)
    hosts = build_tests(cargo, selection, env | {"HYPERSOLVE_REPLAY_DRIVER": str(Path(__file__).resolve())})
    required = INTERVAL_SUITES | REPLAYS.keys() | HOST_CASES.keys()
    if set(hosts) != set(guests) or not required <= guests.keys():
        raise SystemExit("Missing test binaries or incompatible native and guest inventories")
    report = {"guest": {}, "fresh_replays": {}, "host_only": {}, "dispatch_windows": {}, "replay_records": {}}
    for suite, guest in sorted(guests.items()):
        command = ["node", str(RUNNER), guest]
        names = test_names(command)
        host_names = test_names([hosts[suite]])
        if not names or names != host_names:
            raise SystemExit(f"Empty or unequal inventories for {suite}: {names ^ host_names}")
        replays = REPLAYS.get(suite, {})
        host_cases = HOST_CASES.get(suite, {})
        modes = HORNER if suite == "hypersolve" else {}
        special = replays.keys() | host_cases.keys() | modes.keys()
        if not special <= names or any(child and child not in names for child, _, _ in replays.values()):
            raise SystemExit(f"Missing expected replay or host case for {suite}")
        direct = names - special
        exact_horner = {name.replace("_dispatch_differential", "_exact_differential") for name in modes}
        if not exact_horner <= direct:
            raise SystemExit(f"Missing exact Horner differentials: {exact_horner - direct}")
        log = output / f"{suite}.log"
        skip = [arg for name in sorted(special) for arg in ("--skip", name)]
        run = execute([*command, "--exact", "--test-threads=1", *skip], log)
        if run.returncode or not completed(run.stdout, direct):
            raise SystemExit(f"Incomplete or failed WASI {suite} test suite. See {log}")
        report["guest"][suite] = sorted(direct)
        for case, (child, key, count) in replays.items():
            matched = run_parent(output, suite, case, child or case, key, count, guest, hosts[suite])
            if matched:
                report["replay_records"].setdefault(suite, {})[case] = matched
            report["fresh_replays"].setdefault(suite, {})[case] = count
        for case, diagnostic in host_cases.items():
            stem = f"{suite}.{case}"
            log = output / (stem + "-guest.log")
            run = execute([*command, "--exact", case, "--nocapture"], log)
            if not run.returncode or diagnostic not in run.stdout or "RuntimeError: unreachable" not in run.stdout:
                raise SystemExit(f"Unexpected WASI capability result for {case}. See {log}")
            log = output / (stem + "-native.log")
            run = execute([hosts[suite], "--exact", case], log)
            if run.returncode or not completed(run.stdout, {case}):
                raise SystemExit(f"Native platform test {case} failed. See {log}")
            report["host_only"].setdefault(suite, {})[case] = diagnostic
        for case, windows in modes.items():
            report["dispatch_windows"][case] = compare_dispatch(
                guest, hosts[suite], case, "HYPERSOLVE_HORNER_BACKEND", "HORNER", windows, output)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{sum(map(len, report['guest'].values()))} direct WASI tests, "
          f"{sum(map(len, report['fresh_replays'].values()))} native parents with fresh WASI children, "
          f"{sum(map(len, report['host_only'].values()))} native platform tests. "
          f"{sum(report['dispatch_windows'].values())} Horner dispatch windows matched native and WASI.")


if __name__ == "__main__":
    if "--exact" in sys.argv[1:]:
        replay_child(sys.argv[1:])
    else:
        main()
