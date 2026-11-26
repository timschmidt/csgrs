// Exact source boxes and explicitly invertible affine coordinate charts.
/// Rational source box supplying independent inequalities and coordinate planes.
///
/// The named fixtures have strictly positive extents; field construction itself
/// does not validate that invariant. Mesh generation is the production path.
#[derive(Clone)]
struct Box3 {
    /// Lower axis bounds, excluded by the interior-membership oracle.
    lo: [Q; 3],
    /// Upper axis bounds, excluded by the interior-membership oracle.
    hi: [Q; 3],
}
impl Box3 {
    /// Converts integer bounds exactly, without checking their order.
    fn new(lo: [i64; 3], hi: [i64; 3]) -> Self {
        Self {
            lo: lo.map(q),
            hi: hi.map(q),
        }
    }
    /// Tests open-box membership using exact rational inequalities.
    ///
    /// The cell oracle queries points off every source plane; boundary ownership
    /// follows regularization rather than this strict inequality test alone.
    fn contains(&self, p: &[Q; 3]) -> bool {
        (0..3).all(|i| self.lo[i] < p[i] && p[i] < self.hi[i])
    }
    /// Constructs the corresponding translated exact cuboid; panics on failure.
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

/// Returns the fixed `[0,2]^3` box and one of nine named rational companions.
///
/// Cases distinguish overlap, shared planes, face/edge/point contact, separation,
/// coincidence, strict nesting and a `10^-12` overlap adjacent to a source face.
/// Unknown names panic. These finite fixtures do not enumerate arbitrary meshes.
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

/// A named invertible affine map paired with an explicitly authored inverse chart.
///
/// Rational, quadratic-algebraic and symbolic coefficients exercise exact
/// transformations while the oracle recovers rational source coordinates.
struct Frame {
    /// Selects one of the five hard-coded forward and inverse maps.
    name: String,
    /// Shared `sqrt(2)/2` coefficient for the 45-degree XY rotation; zero otherwise.
    c: Real,
    /// Symbolic translation `(pi, sqrt(2), ln(2))`; zero for the other maps.
    shift: [Real; 3],
}
impl Frame {
    /// Builds coefficients for the named frame; name validation is deferred to use.
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
    /// Returns the forward homogeneous matrix, panicking for an unknown name.
    ///
    /// Identity, shear, XY rotation and symbolic translation have determinant one;
    /// reflection also scales by `(-2,3,1)` and has determinant minus six.
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
    /// Applies the authored inverse map and requires proved rational coordinates.
    ///
    /// Failure of exact rational normalization is an oracle limitation, reported
    /// as `oracle-undecided`, without rounding or declaring the mesh incorrect.
    /// The supplied point must use this known frame; an unknown frame panics.
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
    /// Transforms under STRICT, retaining the outcome and formatting errors for tests.
    ///
    /// This helper does not itself assert certification; the workflow checks it.
    fn apply(&self, mesh: &TriangleMesh) -> Result<GeometryOutcome<TriangleMesh>, String> {
        solid::transform_with_context(mesh, &self.matrix(), GeometryContext::STRICT)
            .map_err(|e| format!("transform-error {:?}: {e}", e.kind()))
    }
}
