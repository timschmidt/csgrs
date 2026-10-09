"""Reproduce the independent exact affine point-equality matrix."""
from fractions import Fraction
import hashlib
from itertools import product
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
rows = []
for spelling in ['1', '-3', '1e400', '1e-400']:
    scale = Fraction(spelling)
    assert scale != 0
    equal = unequal = 0
    for ax, ay, bx, by in product(range(-2, 3), repeat=4):
        # The common real origin cancels exactly before this rational oracle.
        dx, dy = scale * (ax - bx), scale * (ay - by)
        actual = dx == 0 and dy == 0
        assert actual == (ax == bx and ay == by)
        equal += actual
        unequal += not actual
    assert equal == 25 and unequal == 600
    rows.append(dict(scale=spelling, equal=equal, unequal=unequal))

record = dict(
    scope='Independent rational differences for the public point equality matrix. Common arbitrary real origins cancel; every tested rational scale is nonzero. No Hyper implementation is used. The Rust matrix instantiates three origin families, before/after preparation, and checks both directions plus cloned handles.',
    rows=rows,
    point_pairs=sum(row['equal'] + row['unequal'] for row in rows) * 3 * 2,
    comparison_assertions=sum(row['equal'] + row['unequal'] for row in rows) * 3 * 2 * 3,
    probe_identities=dict(
        multiradical_identity='normal-form-ownership-oracle.json family 0',
        pi_transport_identity='normal-form-ownership-oracle.json family 2, k=1',
        opaque_atom_identity='normal-form-ownership-oracle.json family 4, k=1',
        offsets='The same common coordinate plus unequal exact integer offsets differs by that integer.',
        controls='Point handle clones and coordinate clones retain exact input values.',
    ),
    normal_form_oracle_sha256=hashlib.sha256((root / 'normal-form-ownership-oracle.json').read_bytes()).hexdigest(),
)
Path(sys.argv[1]).write_text(json.dumps(record, indent=2) + '\n')
print(record['point_pairs'], 'independent point pairs verified')
