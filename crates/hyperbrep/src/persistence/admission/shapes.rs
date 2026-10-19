//! Shape summaries only; no coordinate, coefficient or geometry construction.

use super::*;

pub(super) struct Count(pub(super) usize);
impl<'de> Deserialize<'de> for Count {
    fn deserialize<D: Deserializer<'de>>(decoder: D) -> Result<Self, D::Error> {
        struct Counter;
        impl<'de> Visitor<'de> for Counter {
            type Value = Count;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("an array")
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Count, A::Error> {
                let mut count = 0;
                while seq.next_element::<IgnoredAny>()?.is_some() {
                    count += 1;
                }
                Ok(Count(count))
            }
        }
        decoder.deserialize_seq(Counter)
    }
}
impl Record for Count {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.topology(self.0)
    }
}

pub(super) struct Rows {
    rows: usize,
    columns: usize,
    elements: usize,
}
impl<'de> Deserialize<'de> for Rows {
    fn deserialize<D: Deserializer<'de>>(decoder: D) -> Result<Self, D::Error> {
        struct Counter;
        impl<'de> Visitor<'de> for Counter {
            type Value = Rows;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("a control lattice")
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Rows, A::Error> {
                let mut result = Rows {
                    rows: 0,
                    columns: 0,
                    elements: 0,
                };
                while let Some(Count(count)) = seq.next_element()? {
                    result.rows += 1;
                    result.columns = result.columns.max(count);
                    result.elements += count;
                }
                Ok(result)
            }
        }
        decoder.deserialize_seq(Counter)
    }
}
impl Rows {
    fn entries(&self) -> usize {
        self.rows + self.elements
    }
}

#[derive(Deserialize)]
pub(super) enum SpatialPoint {
    Coordinates(IgnoredAny),
    CurveImage { source: Box<Curve> },
    SurfaceImage(SurfacePoint),
}

impl Record for SpatialPoint {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Coordinates(_) => Ok(()),
            Self::CurveImage { source } => source.admit(budget),
            Self::SurfaceImage(point) => point.admit(budget),
        }
    }
}

#[derive(Deserialize)]
pub(super) struct SurfacePoint {
    source: Box<Surface>,
    parameter: Box<PlanarPoint>,
}

impl Record for SurfacePoint {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.record()?;
        self.source.admit(budget)?;
        budget.record()?;
        self.parameter.admit(budget)
    }
}

#[derive(Deserialize)]
pub(super) enum Vertex {
    Coordinates(IgnoredAny),
    CurveImage(IgnoredAny),
    SurfaceImage(SurfacePoint),
}

impl Record for Vertex {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Coordinates(_) | Self::CurveImage(_) => Ok(()),
            Self::SurfaceImage(point) => point.admit(budget),
        }
    }
}

#[derive(Deserialize)]
pub(super) enum Curve {
    AffineImage {
        source: Box<Curve>,
    },
    Rotation {
        origin: SpatialPoint,
    },
    Translation {
        origin: SpatialPoint,
    },
    PlanarImage {
        source: Box<Pcurve>,
    },
    Line(IgnoredAny),
    EllipseArc(IgnoredAny),
    RationalBezier {
        control_points: Count,
        weights: Count,
    },
    Nurbs {
        degree: usize,
        control_points: Count,
        weights: Count,
        knots: Count,
    },
}
impl Record for Curve {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::AffineImage { source } => {
                budget.record()?;
                source.admit(budget)
            }
            Self::Rotation { origin } | Self::Translation { origin } => {
                budget.record()?;
                origin.admit(budget)
            }
            Self::PlanarImage { source } => {
                budget.record()?;
                source.admit(budget)
            }
            Self::Line(_) | Self::EllipseArc(_) => Ok(()),
            Self::RationalBezier {
                control_points,
                weights,
            } => {
                budget.degree(control_points.0.saturating_sub(1))?;
                budget.geometry(control_points.0 + weights.0)
            }
            Self::Nurbs {
                degree,
                control_points,
                weights,
                knots,
            } => {
                budget.degree(degree)?;
                budget.geometry(control_points.0 + weights.0 + knots.0)
            }
        }
    }
}

#[derive(Deserialize)]
pub(super) enum Pcurve {
    PolynomialBranch(IgnoredAny),
    Chord {
        start: Box<PlanarPoint>,
        end: Box<PlanarPoint>,
    },
    QuadraticBezier(Count),
    CubicBezier(Count),
    AnalyticParallel {
        source: ParallelSource,
    },
    SourceRange(IgnoredAny),
    Line(IgnoredAny),
    CircularArc(IgnoredAny),
    RationalBezier {
        homogeneous_controls: Count,
    },
    RationalQuadraticBezier {
        control_points: Count,
        weights: Count,
    },
    Nurbs {
        degree: usize,
        homogeneous_controls: Count,
        knots: Count,
    },
}
impl Record for Pcurve {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Chord { start, end } => {
                budget.record()?;
                start.admit(budget)?;
                budget.record()?;
                end.admit(budget)
            }
            Self::QuadraticBezier(count) => {
                budget.degree(2)?;
                budget.geometry(count.0)
            }
            Self::CubicBezier(count) => {
                budget.degree(3)?;
                budget.geometry(count.0)
            }
            Self::AnalyticParallel { source } => {
                budget.record()?;
                source.admit(budget)
            }
            Self::PolynomialBranch(_)
            | Self::SourceRange(_)
            | Self::Line(_)
            | Self::CircularArc(_) => Ok(()),
            Self::RationalBezier {
                homogeneous_controls,
            } => {
                budget.degree(homogeneous_controls.0.saturating_sub(1))?;
                budget.geometry(homogeneous_controls.0)
            }
            Self::RationalQuadraticBezier {
                control_points,
                weights,
            } => {
                budget.degree(2)?;
                budget.geometry(control_points.0 + weights.0)
            }
            Self::Nurbs {
                degree,
                homogeneous_controls,
                knots,
            } => {
                budget.degree(degree)?;
                budget.geometry(homogeneous_controls.0 + knots.0)
            }
        }
    }
}

#[derive(Deserialize)]
pub(super) struct PolynomialSource {
    equation: Rows,
    construction: PolynomialConstruction,
    evaluation: FiberControls,
}

#[derive(Deserialize)]
struct PolynomialConstruction {
    max_axis_degree: usize,
    max_projection_degree: usize,
    max_refinement_rounds: usize,
    fiber: FiberControls,
}

#[derive(Deserialize)]
struct FiberControls {
    max_subdivision_depth: usize,
    refinement_steps: usize,
}

impl FiberControls {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.solver_steps(self.max_subdivision_depth)?;
        budget.solver_steps(self.refinement_steps)
    }
}

impl Record for PolynomialSource {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.degree(self.equation.rows.saturating_sub(1))?;
        budget.degree(self.equation.columns.saturating_sub(1))?;
        budget.geometry(self.equation.entries())?;
        budget.degree(self.construction.max_axis_degree)?;
        budget.degree(self.construction.max_projection_degree)?;
        budget.solver_steps(self.construction.max_refinement_rounds)?;
        self.construction.fiber.admit(budget)?;
        self.evaluation.admit(budget)
    }
}

#[derive(Deserialize)]
pub(super) enum ParallelSource {
    Quadratic(Count),
    Cubic(Count),
    Rational { homogeneous_controls: Count },
}
impl Record for ParallelSource {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        let (degree, count) = match self {
            Self::Quadratic(count) => (2, count.0),
            Self::Cubic(count) => (3, count.0),
            Self::Rational {
                homogeneous_controls,
            } => (
                homogeneous_controls.0.saturating_sub(1),
                homogeneous_controls.0,
            ),
        };
        budget.degree(degree)?;
        budget.geometry(count)
    }
}

#[derive(Deserialize)]
pub(super) enum Surface {
    Plane(IgnoredAny),
    Cylinder(IgnoredAny),
    Sphere(IgnoredAny),
    Cone(IgnoredAny),
    Torus(IgnoredAny),
    Extrusion {
        profile: Curve,
    },
    Revolution {
        profile: Curve,
    },
    RationalBezier {
        control_points: Rows,
        weights: Rows,
    },
    Nurbs {
        u_degree: usize,
        v_degree: usize,
        control_points: Rows,
        weights: Rows,
        u_knots: Count,
        v_knots: Count,
    },
}
impl Record for Surface {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Plane(_)
            | Self::Cylinder(_)
            | Self::Sphere(_)
            | Self::Cone(_)
            | Self::Torus(_) => Ok(()),
            Self::Extrusion { profile } | Self::Revolution { profile } => {
                budget.record()?;
                profile.admit(budget)
            }
            Self::RationalBezier {
                control_points,
                weights,
            } => {
                budget.degree(control_points.rows.saturating_sub(1))?;
                budget.degree(control_points.columns.saturating_sub(1))?;
                budget.geometry(control_points.entries() + weights.entries())
            }
            Self::Nurbs {
                u_degree,
                v_degree,
                control_points,
                weights,
                u_knots,
                v_knots,
            } => {
                budget.degree(u_degree)?;
                budget.degree(v_degree)?;
                budget
                    .geometry(control_points.entries() + weights.entries() + u_knots.0 + v_knots.0)
            }
        }
    }
}

#[derive(Deserialize)]
pub(super) enum Selection {
    AffineForm { point: Box<PlanarPoint> },
    Root { polynomial: Count },
    Image(IgnoredAny),
}
impl Record for Selection {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Root { polynomial } => {
                budget.degree(polynomial.0.saturating_sub(1))?;
                budget.geometry(polynomial.0)?;
            }
            Self::AffineForm { point } => {
                budget.record()?;
                point.admit(budget)?;
            }
            Self::Image(_) => {}
        }
        Ok(())
    }
}

#[derive(Deserialize)]
pub(super) enum PlanarPoint {
    Coordinates(IgnoredAny),
    PolynomialFiber {
        equation: Rows,
    },
    Rational {
        homogeneous_controls: Count,
    },
    RationalExpression {
        x_numerator: Count,
        y_numerator: Count,
        denominator: Count,
    },
    Parallel {
        source: ParallelSource,
    },
    Similarity {
        source: Box<PlanarPoint>,
    },
    SupportingLineIntersection {
        first: Box<[PlanarPoint; 2]>,
        second: Box<[PlanarPoint; 2]>,
    },
}
impl Record for PlanarPoint {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        match self {
            Self::Coordinates(_) => Ok(()),
            Self::PolynomialFiber { equation } => {
                budget.degree(equation.rows.saturating_sub(1))?;
                budget.degree(equation.columns.saturating_sub(1))?;
                budget.geometry(equation.entries())
            }
            Self::Rational {
                homogeneous_controls,
            } => {
                budget.degree(homogeneous_controls.0.saturating_sub(1))?;
                budget.geometry(homogeneous_controls.0)
            }
            Self::RationalExpression {
                x_numerator,
                y_numerator,
                denominator,
            } => {
                for count in [x_numerator.0, y_numerator.0, denominator.0] {
                    budget.degree(count.saturating_sub(1))?;
                    budget.geometry(count)?;
                }
                Ok(())
            }
            Self::Parallel { source } => {
                budget.record()?;
                source.admit(budget)
            }
            Self::Similarity { source } => {
                budget.record()?;
                source.admit(budget)
            }
            Self::SupportingLineIntersection { first, second } => {
                for point in (*first).into_iter().chain(*second) {
                    budget.record()?;
                    point.admit(budget)?;
                }
                Ok(())
            }
        }
    }
}

#[derive(Deserialize)]
pub(super) struct Wire {
    edge_uses: Count,
}
#[derive(Deserialize)]
pub(super) struct Face {
    inner: Count,
}
#[derive(Deserialize)]
pub(super) struct Solid {
    voids: Count,
}
impl Record for Wire {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.topology(self.edge_uses.0)
    }
}
impl Record for Face {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.topology(self.inner.0)
    }
}
impl Record for Solid {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E> {
        budget.topology(self.voids.0)
    }
}
