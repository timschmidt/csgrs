"""Retain the exact common PSLG cell and generate its native public regression."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
source = E / 'diagnostic-v2/trace.stderr'
points, faces = {}, []
for line in source.read_text().splitlines():
    if line == 'TRACE arrangement 1':
        break
    if line.startswith('TRACE point '):
        _, _, index, value = line.split(' ', 3)
        points[int(index)] = json.loads(value)
    if line.startswith('TRACE corefine ') and len(faces) < 3:
        match = re.fullmatch(r'TRACE corefine (\d+) points=(.*?) constraints=(.*?) triangles=(.*)', line)
        faces.append({'source_face': int(match[1]), 'point_ids': json.loads(match[2]),
                      'constraints': json.loads(match[3]), 'triangles': json.loads(match[4])})

def rational(value):
    record = value['rational']
    assert len(record['numerator']) <= 1 and len(record['denominator']) == 1
    return Fraction(record['sign'] * sum(record['numerator']), record['denominator'][0])

ids = sorted({index for face in faces for index in face['point_ids']})
scaled = {}
for index in ids:
    xyz = list(map(rational, points[index]))
    assert xyz[0] == 3
    xy = [value * 30 for value in xyz[1:]]
    assert all(value.denominator == 1 for value in xy)
    scaled[index] = list(map(int, xy))
cell = {23, 25, 32, 54, 74}
boundary = sorted([[23, 54], [23, 74], [25, 32], [25, 54], [32, 74]])
expected = sorted([[23, 32, 54], [23, 32, 74], [25, 32, 54]])
for face in faces:
    assert sorted(edge for edge in face['constraints'] if set(edge) <= cell) == boundary
    common = sorted(sorted(triangle) for triangle in face['triangles'] if set(triangle) <= cell)
    face['common_triangles'] = common
assert faces[0]['common_triangles'] == faces[2]['common_triangles'] == expected
assert faces[1]['common_triangles'] != expected
record = {'trace_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
          'projection': 'x=3; exact (y,z) coordinates scaled by 30',
          'points': scaled, 'faces': faces, 'common_cell': sorted(cell),
          'common_boundary': boundary, 'expected_greedy_triangles': expected,
          'common_cell_double_area': 1080, 'source_double_areas': [5400, 8100, 5400]}
fixture = E / 'fixtures/common-cell-v1.json'
fixture.parent.mkdir(exist_ok=True)
assert not fixture.exists()
fixture.write_text(json.dumps(record, indent=2) + '\n')
test = E.parents[2] / 'crates/hypertri/tests/corefinement_cells.rs'
assert not test.exists()
code = '''//! Common-cell regression extracted from the retained 26-box Boolean fixture.
//! Coordinates are exact integer multiples of its x=3 projection. Outside
//! constraints differ; the bounded five-vertex cell and its vertex order agree.

use hypertri::{Constraint, ExactPoint, PredicatePolicy, Real, TriangulationCertainty,
    TriangulationContext, cdt::constrained_triangulation_convex_hull};
use std::collections::BTreeSet;

const POINTS: &[(usize, [i64; 2])] = &[
'''
code += ''.join(f'    ({index}, {xy}),\n' for index, xy in scaled.items())
code += '''];

struct Face {
    ids: &'static [usize],
    constraints: &'static [[usize; 2]],
    double_area: i128,
}

fn faces() -> [Face; 3] {
    [
'''
for face, area in zip(faces, record['source_double_areas']):
    code += f'        Face {{ ids: &{face["point_ids"]}, constraints: &{face["constraints"]}, double_area: {area} }},\n'
code += '''    ]
}

fn coordinates(index: usize) -> [i64; 2] {
    POINTS.iter().find(|(id, _)| *id == index).unwrap().1
}

fn twice_area(triangle: [usize; 3]) -> i128 {
    let [a, b, c] = triangle.map(|index| coordinates(index).map(i128::from));
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

#[test]
fn common_cells_retain_one_triangulation_despite_surrounding_constraints() {
    let cell = [23, 25, 32, 54, 74];
    let expected = BTreeSet::from([[23, 32, 54], [23, 32, 74], [25, 32, 54]]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = TriangulationContext::new(policy);
        for face in faces() {
            let points = face.ids.iter().map(|&id| {
                let [x, y] = coordinates(id);
                ExactPoint::new(Real::from(x), Real::from(y))
            }).collect::<Vec<_>>();
            let original = face.constraints.iter().map(|edge| {
                Constraint::new(face.ids.binary_search(&edge[0]).unwrap(),
                    face.ids.binary_search(&edge[1]).unwrap())
            }).collect::<Vec<_>>();
            for order in 0..3 {
                let mut constraints = original.clone();
                if order == 1 { constraints.reverse(); }
                if order == 2 { constraints.rotate_left(3); }
                let outcome = constrained_triangulation_convex_hull(&context, &points, &constraints)
                    .expect("certified common-cell triangulation");
                assert_eq!(outcome.certainty, TriangulationCertainty::Certified);
                let triangles = outcome.value.iter().map(|triangle| triangle.map(|index| face.ids[index]))
                    .collect::<Vec<_>>();
                // Independent integer area oracle, separate from shared topology validation.
                assert!(triangles.iter().all(|&triangle| twice_area(triangle) > 0));
                assert_eq!(triangles.iter().map(|&triangle| twice_area(triangle)).sum::<i128>(), face.double_area);
                let common = triangles.into_iter().filter(|triangle| triangle.iter().all(|id| cell.contains(id)))
                    .map(|mut triangle| { triangle.sort_unstable(); triangle }).collect::<BTreeSet<_>>();
                assert_eq!(common, expected, "point IDs {:?}, constraint order {order}", face.ids);
                assert_eq!(common.iter().map(|&triangle| twice_area(triangle).abs()).sum::<i128>(), 1080);
            }
        }
    }
}
'''
test.write_text(code)
print(json.dumps({'fixture': str(fixture), 'test': str(test), 'common_boundary_identical': True,
                  'triangulations_disagree': True}))
