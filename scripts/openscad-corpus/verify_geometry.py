"""Verify independent exact controls against a supplied native worker.

Reference fixtures are finite rational holed shapes from the pinned exporter.
Their vertices and winding define the expected sets. They do not claim a
general fidelity bound for OpenSCAD exports.
"""
import argparse
from fractions import Fraction
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("native_collector", HERE / "native.py")
NATIVE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NATIVE)
OUT = None
WORKER = None


def decimal_half(value):
    # Control vertices are integral; edge midpoints have denominator at most 2.
    if value.denominator == 1:
        return str(value.numerator)
    assert value.denominator == 2
    return ("-" if value < 0 else "") + str(abs(value.numerator) // 2) + ".5"


def subdivided_reference():
    vertices, faces = [], []
    for line in (HERE / "controls/cube.obj.fixture").read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append(tuple(Fraction(x) for x in fields[1:]))
        if fields and fields[0] == "f":
            assert len(fields) == 4
            faces.append(tuple(int(x) - 1 for x in fields[1:]))
    midpoints = {}

    def middle(a, b):
        key = tuple(sorted((a, b)))
        if key not in midpoints:
            midpoints[key] = len(vertices)
            vertices.append(tuple((x + y) / 2 for x, y in zip(vertices[a], vertices[b])))
        return midpoints[key]

    triangles = []
    for a, b, c in faces:
        ab, bc, ca = middle(a, b), middle(b, c), middle(c, a)
        triangles.extend(((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca)))
    path = OUT / "subdivided.obj"
    path.write_text("\n".join("v " + " ".join(decimal_half(x) for x in v) for v in vertices)
                    + "\n" + "\n".join("f " + " ".join(str(x + 1) for x in f) for f in triangles) + "\n")
    return path


def main():
    global OUT, WORKER
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    WORKER = args.worker.resolve()
    OUT = args.output.resolve()
    OUT.mkdir(parents=True, exist_ok=False)
    cube = HERE / "controls/cube.scad"
    square = HERE / "controls/square.scad"
    for name, suffix in [("cube", "obj"), ("square", "svg")]:
        (OUT / (name + "." + suffix)).write_bytes((HERE / "controls" / (name + "." + suffix + ".fixture")).read_bytes())
    shifted = OUT / "same-volume-and-bounds-different-set.scad"
    shifted.write_text("difference(){cube([3,3,3]);translate([0.5,1,-1])cube([1,1,5]);}\n")
    empty = OUT / "empty.scad"
    empty.write_text("difference(){cube(1);cube(2);}\n")
    invalid = OUT / "invalid.scad"
    invalid.write_text("cube(;\n")
    empty_reference = OUT / "explicit-reference.empty"
    empty_reference.write_text("D03_EXPLICITLY_ADOPTED_EMPTY_REFERENCE_MODEL_V1\n")
    junk = OUT / "junk.obj"
    junk.write_text("This is not an OBJ model.\n")
    open_obj = OUT / "open.obj"
    open_obj.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    tests = [
        ("holed_cube", cube, OUT / "cube.obj", "represented_sets_equal"),
        ("holed_square_y_reflection", square, OUT / "square.svg", "represented_sets_equal"),
        ("different_tessellation", cube, subdivided_reference(), "represented_sets_equal"),
        ("same_bounds_volume_distinct_set", shifted, OUT / "cube.obj", "represented_sets_different"),
        ("canonical_empty", empty, empty_reference, "represented_sets_equal"),
        ("invalid_is_not_empty", invalid, empty_reference, "native_or_import_failed"),
        ("junk_is_not_empty", empty, junk, "native_or_import_failed"),
        ("open_topology_is_not_empty", empty, open_obj, "comparison_failed"),
    ]
    rows = []
    for name, source, reference, expected in tests:
        folder = OUT / name
        folder.mkdir()
        row = NATIVE.invoke(source, reference, folder, worker=WORKER)
        row.update(name=name, expected=expected)
        NATIVE.save(folder / "result.json", row)
        rows.append(row)
        print(name, row["status"], "expected", expected, flush=True)
    NATIVE.save(OUT / "results.json", rows)
    failures = [x["name"] for x in rows if x["status"] != x["expected"]]
    if failures:
        raise AssertionError(failures)


if __name__ == "__main__":
    main()
