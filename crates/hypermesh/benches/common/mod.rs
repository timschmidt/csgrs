use hypermesh::{InputMesh, Point3, Real, Triangle};
use std::collections::BTreeMap;

pub fn r(value: i32) -> Real {
    value.into()
}

pub fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).expect("benchmark denominator is nonzero")
}

pub fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn cube_triangles() -> Vec<Triangle> {
    vec![
        Triangle::new(4, 5, 6),
        Triangle::new(4, 6, 7),
        Triangle::new(0, 3, 2),
        Triangle::new(0, 2, 1),
        Triangle::new(1, 2, 6),
        Triangle::new(1, 6, 5),
        Triangle::new(0, 4, 7),
        Triangle::new(0, 7, 3),
        Triangle::new(3, 7, 6),
        Triangle::new(3, 6, 2),
        Triangle::new(0, 1, 5),
        Triangle::new(0, 5, 4),
    ]
}

pub fn cube(center: [Real; 3], half_extent: Real) -> InputMesh {
    let min = [
        &center[0] - &half_extent,
        &center[1] - &half_extent,
        &center[2] - &half_extent,
    ];
    let max = [
        &center[0] + &half_extent,
        &center[1] + &half_extent,
        &center[2] + &half_extent,
    ];
    InputMesh::new(
        vec![
            p(min[0].clone(), min[1].clone(), min[2].clone()),
            p(max[0].clone(), min[1].clone(), min[2].clone()),
            p(max[0].clone(), max[1].clone(), min[2].clone()),
            p(min[0].clone(), max[1].clone(), min[2].clone()),
            p(min[0].clone(), min[1].clone(), max[2].clone()),
            p(max[0].clone(), min[1].clone(), max[2].clone()),
            p(max[0].clone(), max[1].clone(), max[2].clone()),
            p(min[0].clone(), max[1].clone(), max[2].clone()),
        ],
        cube_triangles(),
    )
}

pub fn octahedron(center: [Real; 3], radius: Real) -> InputMesh {
    let [cx, cy, cz] = center;
    InputMesh::new(
        vec![
            p(&cx + &radius, cy.clone(), cz.clone()),
            p(&cx - &radius, cy.clone(), cz.clone()),
            p(cx.clone(), &cy + &radius, cz.clone()),
            p(cx.clone(), &cy - &radius, cz.clone()),
            p(cx.clone(), cy.clone(), &cz + &radius),
            p(cx, cy, &cz - &radius),
        ],
        vec![
            Triangle::new(0, 2, 4),
            Triangle::new(2, 1, 4),
            Triangle::new(1, 3, 4),
            Triangle::new(3, 0, 4),
            Triangle::new(2, 0, 5),
            Triangle::new(1, 2, 5),
            Triangle::new(3, 1, 5),
            Triangle::new(0, 3, 5),
        ],
    )
}

pub fn cube_pair() -> [InputMesh; 2] {
    [
        cube([r(0), r(0), r(0)], r(1)),
        cube([q(1, 2), q(1, 2), q(1, 2)], r(1)),
    ]
}

fn subdivide_triangles(mut mesh: InputMesh, levels: usize) -> InputMesh {
    for _ in 0..levels {
        let mut edge_midpoints = BTreeMap::new();
        let mut triangles = Vec::with_capacity(mesh.triangles.len() * 4);
        for triangle in &mesh.triangles {
            let [a, b, c] = triangle.indices();
            let mut midpoint = |left: usize, right: usize| {
                let key = (left.min(right), left.max(right));
                *edge_midpoints.entry(key).or_insert_with(|| {
                    let left = &mesh.positions[left];
                    let right = &mesh.positions[right];
                    let two = r(2);
                    let point = Point3::new(
                        ((&left.x + &right.x) / &two).expect("nonzero midpoint divisor"),
                        ((&left.y + &right.y) / &two).expect("nonzero midpoint divisor"),
                        ((&left.z + &right.z) / &two).expect("nonzero midpoint divisor"),
                    );
                    let index = mesh.positions.len();
                    mesh.positions.push(point);
                    index
                })
            };
            let ab = midpoint(a, b);
            let bc = midpoint(b, c);
            let ca = midpoint(c, a);
            triangles.extend([
                Triangle::new(a, ab, ca),
                Triangle::new(ab, b, bc),
                Triangle::new(ca, bc, c),
                Triangle::new(ab, bc, ca),
            ]);
        }
        mesh.triangles = triangles;
    }
    mesh
}

pub fn subdivided_cube_pair(levels: usize) -> [InputMesh; 2] {
    let [left, right] = cube_pair();
    [
        subdivide_triangles(left, levels),
        subdivide_triangles(right, levels),
    ]
}

pub fn nested_cube_pair() -> [InputMesh; 2] {
    [
        cube([r(0), r(0), r(0)], r(2)),
        cube([r(0), r(0), r(0)], r(1)),
    ]
}

pub fn octahedron_pair() -> [InputMesh; 2] {
    [
        octahedron([r(0), r(0), r(0)], r(3)),
        octahedron([r(1), r(1), r(1)], r(3)),
    ]
}

pub fn nested_tool_cubes() -> Vec<InputMesh> {
    let mut meshes = Vec::with_capacity(6);
    meshes.push(cube([r(0), r(0), r(0)], r(8)));
    for x in [-6, -3, 0, 3, 6] {
        meshes.push(cube([r(x), r(0), r(0)], r(1)));
    }
    meshes
}
