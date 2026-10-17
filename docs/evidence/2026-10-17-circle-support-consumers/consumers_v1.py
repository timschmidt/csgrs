"""Qualify the unchanged root, circuit and native binding consumers."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
from runner_v1 import Runner, V, dump, sha

r = Runner("consumer-validation-v1", V / "native-target-v1", Path(__file__))


def cargo(action, package, *args):
    return ["cargo", action, "--offline", "--locked", "-p", package, "--release", *args]


r.run("format", ["cargo", "fmt", "--all", "--", "--check"])
for name, package, flags in [
    ("root-all", "csgrs", ["--all-features"]),
    ("root-minimal", "csgrs", ["--no-default-features"]),
    ("circuit-all", "hypercircuit", ["--all-features"]),
]:
    code = r.run(name + "-build", cargo("test", package, *flags, "--no-run", "--message-format=json"))
    if code == 0:
        r.run(name, cargo("test", package, *flags, "--no-fail-fast", "--", "--test-threads=1"))
    else:
        r.omit(name, name + "-build did not complete successfully")
r.run("clippy", [
    "cargo", "clippy", "--offline", "--locked", "-p", "hypercurve", "-p", "csgrs",
    "-p", "hyperdrc", "-p", "hypercircuit", "--release", "--all-features", "--all-targets", "--", "-D", "warnings",
])
r.run("docs", [
    "cargo", "doc", "--offline", "--locked", "-p", "hypercurve", "-p", "csgrs",
    "-p", "hyperdrc", "-p", "hypercircuit", "--release", "--all-features", "--no-deps",
], docs=True)
ffi = r.run("ffi-build", cargo("build", "csgrs-ffi", "--message-format=json"))
if ffi == 0:
    r.run("ffi-tests", cargo("test", "csgrs-ffi", "--no-fail-fast", "--", "--test-threads=1"))
    r.run("ffi-clippy", cargo("clippy", "csgrs-ffi", "--all-targets", "--", "-D", "warnings"))
    library = r.target / "release/libcsgrs_ffi.so"
    assert library.is_file()
    before = sha(library)
    shared = {"CSGRS_LIBRARY": str(library), "PYTHONDONTWRITEBYTECODE": "1"}
    for name in ["empty_composition", "curve_constructor_failures", "planar_operation_failures", "exact_linear_modeling"]:
        for language, compiler, standard in [("c", "cc", "-std=c11"), ("cpp", "c++", "-std=c++17")]:
            source = r.source / "tests/ffi" / (name + "." + language)
            if not source.exists():
                continue
            binary = r.output / (name + "-" + language)
            code = r.run(name + "-" + language + "-build", [
                compiler, standard, "-Wall", "-Wextra", "-Werror", "-I", str(r.source / "crates/csgrs-ffi/include"),
                "-I", str(r.source / "bindings/cpp"), str(source), "-L", str(r.target / "release"),
                "-lcsgrs_ffi", "-Wl,-rpath," + str(r.target / "release"), "-o", str(binary),
            ])
            if code == 0:
                r.run(name + "-" + language, [str(binary)])
            else:
                r.omit(name + "-" + language, "consumer compilation did not complete successfully")
        source = r.source / "tests/ffi" / (name + ".py")
        if source.exists():
            r.run(name + "-python", ["python3", str(source)], extra=shared)
    assert sha(library) == before
    dump(r.output / "ffi-library.json", {"path": str(library), "sha256": before, "bytes": library.stat().st_size})
else:
    r.omit("ffi-tests-and-real-consumers", "ffi-build did not complete successfully")
raise SystemExit(r.finish())
