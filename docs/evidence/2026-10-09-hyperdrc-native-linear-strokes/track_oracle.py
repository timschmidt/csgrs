"""Independent rational formulas for complete-board linear track capsules.

The oracle derives area, envelope and midpoint from authored endpoints/width.
It does not call the native stroke kernel or reproduce KiCad integer rounding.
Footprint artwork and curved tracks remain outside this linear-track oracle.
"""
from fractions import Fraction as F
from pathlib import Path
import hashlib
import importlib.util
import json
import re
import sys

sys.dont_write_bytecode = True
record = Path(__file__).resolve().parent
dependency = record.parent / "2026-10-09-hyperdrc-kicad-pad-sources/full_board_oracle.py"
spec = importlib.util.spec_from_file_location("pad_source_oracle", dependency)
parser = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parser)


def rational(value):
    return [str(value.numerator), str(value.denominator)]


def expected(path):
    raw = path.read_bytes()
    root = parser.parse(raw.decode())
    nets = {node[1]: node[2] for node in parser.children(root, "net")}
    rows = []
    for index, node in enumerate(parser.children(root, "segment")):
        start = list(map(F, parser.child(node, "start", [])))
        end = list(map(F, parser.child(node, "end", [])))
        assert len(start) == len(end) == 2
        width, = map(F, parser.child(node, "width", []))
        assert width > 0
        radius = width / 2
        squared_length = sum((b-a)**2 for a, b in zip(start, end))
        net = parser.child(node, "net", [])
        net = (nets.get(net[0], net[1] if len(net)>1 else None)
               if re.fullmatch(r"[+-]?\d+", net[0]) else net[0]) if net else None
        rows.append(dict(index=index, layer=parser.child(node, "layer", ["unknown"])[0], net=net,
                         start=[rational(v) for v in start], end=[rational(v) for v in end],
                         width=rational(width), length_squared=rational(squared_length),
                         area_pi=rational(radius*radius),
                         center=[rational((a+b)/2) for a,b in zip(start,end)],
                         bounds=[rational(min(start[0],end[0])-radius),
                                 rational(min(start[1],end[1])-radius),
                                 rational(max(start[0],end[0])+radius),
                                 rational(max(start[1],end[1])+radius)]))
    return dict(source=str(path), sha256=hashlib.sha256(raw).hexdigest(), rows=rows,
                excluded_root_arc_tracks=len(parser.children(root, "arc")),
                scope="All root linear segments; exact capsule area, envelope and authored midpoint. Footprint artwork and curved tracks excluded.")


if __name__ == "__main__":
    inputs, out = map(Path, sys.argv[1:3])
    out.mkdir(parents=True, exist_ok=False)
    for label, name in [("cparti-fpga", "CPArti FPGA dev board.kicad_pcb"), ("hvp109a", "HVP109A.kicad_pcb")]:
        result = expected(inputs / label / name)
        (out / (label + ".json")).write_text(json.dumps(result, separators=(",", ":")) + "\n")
        print(label, len(result["rows"]), "linear tracks; excluded arcs", result["excluded_root_arc_tracks"])
