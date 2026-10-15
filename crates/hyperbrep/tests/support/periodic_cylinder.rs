//! Public-API fixtures: quarter-circle boundary edges with independently chosen
//! rectangular side patches. Keys are scoped to this builder, never welded by
//! approximate coordinates. The two periodic seam uses share one spatial edge.
use hyperbrep::*;
use hypercurve::{CircularArc2, Curve2, LineSeg2, Point2 as CurvePoint};
use std::collections::BTreeMap;

struct Grid {
    builder: ModelBuilder,
    vertices: BTreeMap<(usize, usize), VertexKey>,
    edges: BTreeMap<(bool, usize, usize), EdgeKey>,
    levels: usize,
}
impl Grid {
    fn angle(u: usize) -> Real {
        (Real::pi() * Real::from(u as i32) / Real::from(2)).unwrap()
    }
    fn height(&self, v: usize) -> Real {
        (Real::from(4 * v as i32) / Real::from(self.levels as i32)).unwrap()
    }
    fn point(&self, u: usize, v: usize) -> Point3 {
        let (x, y) = [(2, 0), (0, 2), (-2, 0), (0, -2)][u % 4];
        Point3::new(x.into(), y.into(), self.height(v))
    }
    fn vertex(&mut self, u: usize, v: usize) -> VertexKey {
        let key = (u % 4, v);
        if let Some(vertex) = self.vertices.get(&key) {
            return vertex.clone();
        }
        let vertex = self.builder.vertex(self.point(u, v).into()).unwrap();
        self.vertices.insert(key, vertex.clone());
        vertex
    }
    fn edge(&mut self, horizontal: bool, u: usize, v: usize) -> EdgeKey {
        let key = (horizontal, u % 4, v);
        if let Some(edge) = self.edges.get(&key) {
            return edge.clone();
        }
        let start = self.vertex(u, v);
        let end = self.vertex(u + usize::from(horizontal), v + usize::from(!horizontal));
        let (curve, domain) = if horizontal {
            let a = Self::angle(u);
            let b = Self::angle(u + 1);
            (
                Curve3::circle_arc(
                    Point3::new(Real::zero(), Real::zero(), self.height(v)),
                    Vector3::x(),
                    Vector3::y(),
                    Real::from(2),
                    a.clone(),
                    b.clone(),
                )
                .unwrap(),
                ParameterDomain::new(a, b).unwrap(),
            )
        } else {
            (
                Curve3::line(self.point(u, v), self.point(u, v + 1)).unwrap(),
                ParameterDomain::unit(),
            )
        };
        let curve = self.builder.curve(curve).unwrap();
        let edge = self
            .builder
            .edge(&start, &end, &curve, domain.into())
            .unwrap();
        self.edges.insert(key, edge.clone());
        edge
    }
    fn side_use(
        &mut self,
        horizontal: bool,
        u: usize,
        v: usize,
        forward: bool,
        shift: &Real,
    ) -> EdgeUseKey {
        let edge = self.edge(horizontal, u, v);
        let a = CurvePoint::new(Self::angle(u) + shift, self.height(v));
        let b = CurvePoint::new(
            Self::angle(u + usize::from(horizontal)) + shift,
            self.height(v + usize::from(!horizontal)),
        );
        let (a, b, direction) = if forward {
            (a, b, Direction::Forward)
        } else {
            (b, a, Direction::Reversed)
        };
        let correspondence = if horizontal {
            ParameterCorrespondence::affine(
                if forward {
                    Self::angle(1)
                } else {
                    -Self::angle(1)
                },
                Self::angle(u + usize::from(!forward)),
            )
            .unwrap()
        } else if forward {
            ParameterCorrespondence::identity()
        } else {
            ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap()
        };
        let pcurve = self
            .builder
            .pcurve(Pcurve::new(Curve2::from(LineSeg2::try_new(a, b).unwrap())))
            .unwrap();
        self.builder
            .edge_use(&edge, direction, &pcurve, correspondence)
            .unwrap()
    }
    fn cap(&mut self, top: bool) -> FaceKey {
        let v = if top { self.levels } else { 0 };
        let mut uses = Vec::new();
        for i in 0..4 {
            let u = if top { i } else { 3 - i };
            let edge = self.edge(true, u, v);
            let a = self.point(u, v);
            let b = self.point(u + 1, v);
            let a = CurvePoint::new(a.x, a.y);
            let b = CurvePoint::new(b.x, b.y);
            let (a, b) = if top { (a, b) } else { (b, a) };
            let arc = CircularArc2::try_from_center(
                a,
                b,
                CurvePoint::new(Real::zero(), Real::zero()),
                !top,
            )
            .unwrap();
            let pcurve = self.builder.pcurve(Pcurve::new(Curve2::from(arc))).unwrap();
            uses.push(
                self.builder
                    .edge_use(
                        &edge,
                        if top {
                            Direction::Forward
                        } else {
                            Direction::Reversed
                        },
                        &pcurve,
                        ParameterCorrespondence::angular_sweep(),
                    )
                    .unwrap(),
            );
        }
        let wire = self.builder.wire(&uses).unwrap();
        let surface = self
            .builder
            .surface(
                Surface::plane(
                    Point3::new(Real::zero(), Real::zero(), self.height(v)),
                    Vector3::x(),
                    Vector3::y(),
                )
                .unwrap(),
            )
            .unwrap();
        self.builder
            .face(
                &surface,
                if top {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
                &wire,
                &[],
            )
            .unwrap()
    }
}

/// Rectangles use U quarter-turn indices 0..4 and V height indices 0..levels.
/// This fixture deliberately permits bad rectangle lists for rejection tests;
/// local edge/face construction is exact before shell/solid validation runs.
pub fn build(rectangles: &[[usize; 4]], levels: usize, turns: i32) -> Result<Model, BuildError> {
    let mut grid = Grid {
        builder: ModelBuilder::new(),
        vertices: BTreeMap::new(),
        edges: BTreeMap::new(),
        levels,
    };
    let mut faces = vec![grid.cap(false), grid.cap(true)];
    let surface = grid
        .builder
        .surface(
            Surface::cylinder(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::from(2),
            )
            .unwrap(),
        )
        .unwrap();
    let shift = Real::tau() * Real::from(turns);
    for &[a, b, c, d] in rectangles {
        let mut uses = Vec::new();
        for u in a..b {
            uses.push(grid.side_use(true, u, c, true, &shift));
        }
        for v in c..d {
            uses.push(grid.side_use(false, b, v, true, &shift));
        }
        for u in (a..b).rev() {
            uses.push(grid.side_use(true, u, d, false, &shift));
        }
        for v in (c..d).rev() {
            uses.push(grid.side_use(false, a, v, false, &shift));
        }
        let wire = grid.builder.wire(&uses)?;
        faces.push(
            grid.builder
                .face(&surface, Orientation::Forward, &wire, &[])?,
        );
    }
    let shell = grid.builder.shell(&faces)?;
    grid.builder.solid(&shell, &[])?;
    Ok(grid.builder.finish().unwrap())
}

pub fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let loaded = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.counts(), model.counts());
    assert_eq!(loaded.to_json().unwrap(), json);
    loaded
}

pub fn check(model: &Model) {
    let solid = model.solids().next().unwrap();
    assert_eq!(solid.volume().unwrap(), Real::from(16) * Real::pi());
    let area = model
        .faces()
        .map(|f| f.area().unwrap())
        .fold(Real::zero(), |sum, x| sum + x);
    assert_eq!(area, Real::from(24) * Real::pi());
    for (x, y, z, location) in [
        (0, 0, 2, SolidPointLocation::Inside),
        (3, 0, 2, SolidPointLocation::Outside),
        (0, 0, 5, SolidPointLocation::Outside),
        (2, 0, 2, SolidPointLocation::Boundary),
        (0, 0, 0, SolidPointLocation::Boundary),
        (0, 2, 4, SolidPointLocation::Boundary),
    ] {
        assert_eq!(
            solid
                .classify_point(&Point3::new(x.into(), y.into(), z.into()))
                .unwrap(),
            location
        );
    }
}
