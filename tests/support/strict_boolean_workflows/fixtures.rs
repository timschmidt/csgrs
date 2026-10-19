// Exact source boxes and explicitly invertible affine coordinate charts.
#[derive(Clone)]
struct Box3 {
    lo: [Q; 3],
    hi: [Q; 3],
}
impl Box3 {
    fn new(lo: [i64; 3], hi: [i64; 3]) -> Self {
        Self {
            lo: lo.map(q),
            hi: hi.map(q),
        }
    }
    fn contains(&self, p: &[Q; 3]) -> bool {
        (0..3).all(|i| self.lo[i] < p[i] && p[i] < self.hi[i])
    }
    fn mesh(&self) -> TriangleMesh {
        let size = sub(&self.hi, &self.lo).map(Real::from);
        let mesh = solid::cuboid(size[0].clone(), size[1].clone(), size[2].clone()).unwrap();
        mesh.translated(
            Real::from(self.lo[0].clone()),
            Real::from(self.lo[1].clone()),
            Real::from(self.lo[2].clone()),
        )
        .unwrap()
    }
}

fn fixture(name: &str) -> (Box3, Box3) {
    let a = Box3::new([0, 0, 0], [2, 2, 2]);
    let b = match name {
        "overlap" => Box3::new([1, 1, 1], [3, 3, 3]),
        "coplanar" => Box3::new([1, 0, 0], [3, 2, 2]),
        "face-touch" => Box3::new([2, 0, 0], [4, 2, 2]),
        "edge-touch" => Box3::new([2, 2, 0], [4, 4, 2]),
        "point-touch" => Box3::new([2, 2, 2], [4, 4, 4]),
        "disconnected" => Box3::new([3, 0, 0], [5, 2, 2]),
        "coincident" => a.clone(),
        "nested" => Box3 {
            lo: [
                Q::fraction(1, 2).unwrap(),
                Q::fraction(1, 2).unwrap(),
                Q::fraction(1, 2).unwrap(),
            ],
            hi: [
                Q::fraction(3, 2).unwrap(),
                Q::fraction(3, 2).unwrap(),
                Q::fraction(3, 2).unwrap(),
            ],
        },
        "near-coplanar" => {
            let e = Q::fraction(1, 1_000_000_000_000_u64).unwrap();
            Box3 {
                lo: [q(2) - &e, q(0), q(0)],
                hi: [q(4) - e, q(2), q(2)],
            }
        },
        _ => panic!("unknown fixture {name}"),
    };
    (a, b)
}

struct Frame {
    name: String,
    c: Real,
    shift: [Real; 3],
}
impl Frame {
    fn new(name: &str) -> Self {
        let shift = if name == "symbolic-translation" {
            [
                Real::pi(),
                Real::from(2).sqrt().unwrap(),
                Real::from(2).ln().unwrap(),
            ]
        } else {
            [Real::zero(), Real::zero(), Real::zero()]
        };
        let c = if name == "root-rotation" {
            (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap()
        } else {
            Real::zero()
        };
        Self {
            name: name.into(),
            c,
            shift,
        }
    }
    fn matrix(&self) -> Matrix4 {
        let rows = match self.name.as_str() {
            "identity" => return Matrix4::identity(),
            "shear" => [1, 1, 0, 3, 0, 1, 1, -2, 0, 0, 1, 5, 0, 0, 0, 1].map(Real::from),
            "reflection" => [-2, 0, 0, 7, 0, 3, 0, -5, 0, 0, 1, 2, 0, 0, 0, 1].map(Real::from),
            "root-rotation" => [
                self.c.clone(),
                -self.c.clone(),
                Real::zero(),
                Real::zero(),
                self.c.clone(),
                self.c.clone(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one(),
            ],
            "symbolic-translation" => return Matrix4::affine_translation(self.shift.clone()),
            _ => panic!("unknown frame"),
        };
        Matrix4::from_row_major(rows)
    }
    fn chart(&self, p: &Point3) -> Result<[Q; 3], String> {
        let [x, y, z] = [p.x.clone(), p.y.clone(), p.z.clone()];
        let row = match self.name.as_str() {
            "identity" => [x, y, z],
            "shear" => {
                let z = z - Real::from(5);
                let y = y + Real::from(2) - &z;
                [x - Real::from(3) - &y, y, z]
            },
            "reflection" => [
                (Real::from(7) - x) / Real::from(2),
                (y + Real::from(5)) / Real::from(3),
                Ok(z - Real::from(2)),
            ]
            .map(Result::unwrap),
            "root-rotation" => [&self.c * (&x + &y), &self.c * (&y - &x), z],
            "symbolic-translation" => {
                [x - &self.shift[0], y - &self.shift[1], z - &self.shift[2]]
            },
            _ => unreachable!(),
        };
        let mut out = Vec::new();
        for value in row {
            out.push(
                value
                    .exact_rational_normal_form()
                    .ok_or("oracle-undecided: chart coordinate has no proved rational form")?,
            );
        }
        Ok(out.try_into().unwrap())
    }
    fn apply(&self, mesh: &TriangleMesh) -> Result<GeometryOutcome<TriangleMesh>, String> {
        solid::try_transform_with_context(mesh, &self.matrix(), &GeometryContext::STRICT)
            .map_err(|e| format!("transform-error {:?}: {e}", e.kind()))
    }
}
