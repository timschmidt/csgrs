//! Exact scene construction helpers.

use hypercurve::{
    BezierFlatteningCertificate, BezierFlatteningOptions, CertifiedCurvePolyline2, Classification,
    Curve2, CurveCertainty, CurveContext, CurvePath2, CurveRegion2, CurveRegionLoopRole,
};
use hyperlattice::{Point3, Real};
use hypermesh::TriangleMesh;
use hypertri::{ExactPoint, TriangulationContext, TriangulationOutcome};

use crate::error::{Error, Result};
use crate::geometry::{Color3, ExactMesh, ExactVertex, Primitive};

/// Exact line mesh accompanied by Hypercurve's certified source-chord bound.
///
/// The mesh remains a presentation adapter: its exact chord vertices and
/// certificate may be inspected, but neither the line mesh nor a later GPU
/// export is promoted back into source geometry or CAM evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedCurveLineMesh {
    mesh: ExactMesh,
    certificate: BezierFlatteningCertificate,
    source_fragment_count: usize,
}

impl CertifiedCurveLineMesh {
    /// Borrow the exact independent-line mesh.
    pub const fn mesh(&self) -> &ExactMesh {
        &self.mesh
    }

    /// Consume this adapter and return its exact independent-line mesh.
    pub fn into_mesh(self) -> ExactMesh {
        self.mesh
    }

    /// Return the certified maximum distance from each source span to its chords.
    pub const fn max_error(&self) -> &Real {
        self.certificate.max_error()
    }

    /// Return the number of certified chord segments.
    pub const fn segment_count(&self) -> usize {
        self.certificate.segment_count()
    }

    /// Return the deepest certified recursive subdivision used.
    pub const fn max_depth(&self) -> usize {
        self.certificate.max_depth()
    }

    /// Return the number of promoted source Bezier/conic fragments covered.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }
}

/// Per-loop proof and vertex range for a certified region-boundary display mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedCurveRegionLoopLineEvidence {
    role: CurveRegionLoopRole,
    certificate: BezierFlatteningCertificate,
    source_fragment_count: usize,
    first_vertex: usize,
    vertex_count: usize,
}

impl CertifiedCurveRegionLoopLineEvidence {
    /// Return the authoritative material/hole role retained by Hypercurve.
    pub const fn role(&self) -> CurveRegionLoopRole {
        self.role
    }

    /// Return the certified maximum source-curve-to-chord display error.
    pub const fn max_error(&self) -> &Real {
        self.certificate.max_error()
    }

    /// Return the number of certified chords in this loop.
    pub const fn segment_count(&self) -> usize {
        self.certificate.segment_count()
    }

    /// Return the deepest exact-predicate subdivision used in this loop.
    pub const fn max_depth(&self) -> usize {
        self.certificate.max_depth()
    }

    /// Return the number of native Hypercurve fragments covered by this loop.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }

    /// Return this loop's first vertex in the combined line mesh.
    pub const fn first_vertex(&self) -> usize {
        self.first_vertex
    }

    /// Return this loop's vertex count in the combined line mesh.
    pub const fn vertex_count(&self) -> usize {
        self.vertex_count
    }
}

/// Exact region-boundary line mesh with retained topology and subdivision evidence.
///
/// Material and hole roles come from Hypercurve's retained region topology;
/// they are never inferred from display chords. The aggregate mesh remains a
/// one-way presentation adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedCurveRegionLineMesh {
    mesh: ExactMesh,
    loop_evidence: Vec<CertifiedCurveRegionLoopLineEvidence>,
    path_materialization_certainty: CurveCertainty,
    role_certainty: CurveCertainty,
}

impl CertifiedCurveRegionLineMesh {
    /// Borrow the combined exact independent-line mesh.
    pub const fn mesh(&self) -> &ExactMesh {
        &self.mesh
    }

    /// Consume the adapter and return its exact independent-line mesh.
    pub fn into_mesh(self) -> ExactMesh {
        self.mesh
    }

    /// Borrow one role and subdivision record per retained region loop.
    pub fn loop_evidence(&self) -> &[CertifiedCurveRegionLoopLineEvidence] {
        &self.loop_evidence
    }

    /// Return certainty consumed while materializing retained boundary paths.
    pub const fn path_materialization_certainty(&self) -> CurveCertainty {
        self.path_materialization_certainty
    }

    /// Return certainty consumed while classifying material and hole loops.
    pub const fn role_certainty(&self) -> CurveCertainty {
        self.role_certainty
    }

    /// Return the aggregate certified chord count across all loops.
    pub fn segment_count(&self) -> usize {
        self.loop_evidence
            .iter()
            .map(CertifiedCurveRegionLoopLineEvidence::segment_count)
            .sum()
    }
}

/// Segment one exact Hypercurve curve into an exact line mesh with retained evidence.
pub fn curve_line_mesh(
    curve: &Curve2,
    options: &BezierFlatteningOptions,
    policy: &CurveContext,
    z: Real,
    color: Color3,
) -> Result<CertifiedCurveLineMesh> {
    let polyline = match curve.segment_certified(options, policy)? {
        Classification::Decided(polyline) => polyline,
        Classification::Uncertain(reason) => {
            return Err(Error::CurveSegmentationUncertain { reason });
        }
    };
    certified_curve_polyline_mesh(polyline, z, color)
}

/// Segment one connected exact Hypercurve path into an exact line mesh with retained evidence.
pub fn curve_path_line_mesh(
    path: &CurvePath2,
    options: &BezierFlatteningOptions,
    policy: &CurveContext,
    z: Real,
    color: Color3,
) -> Result<CertifiedCurveLineMesh> {
    let polyline = match path.segment_certified(options, policy)? {
        Classification::Decided(polyline) => polyline,
        Classification::Uncertain(reason) => {
            return Err(Error::CurveSegmentationUncertain { reason });
        }
    };
    certified_curve_polyline_mesh(polyline, z, color)
}

/// Segment every exact Hypercurve region boundary into one role-colored line mesh.
///
/// Boundary materialization, material/hole roles, and every chord are certified
/// independently under `policy`. Any unresolved step is returned as explicit
/// uncertainty. Role colors are render attributes and never topology inputs.
pub fn curve_region_line_mesh(
    region: &CurveRegion2,
    options: &BezierFlatteningOptions,
    policy: &CurveContext,
    z: Real,
    material_color: Color3,
    hole_color: Color3,
) -> Result<CertifiedCurveRegionLineMesh> {
    let path_outcome = region.materialized_boundary_paths(policy)?;
    let path_materialization_certainty = path_outcome.certainty;
    let paths = match path_outcome.value {
        Classification::Decided(paths) => paths,
        Classification::Uncertain(reason) => {
            return Err(Error::CurveRegionUncertain {
                operation: "boundary materialization",
                reason,
            });
        }
    };

    let role_outcome = region.loop_roles(policy)?;
    let role_certainty = role_outcome.certainty;
    let roles = match role_outcome.value {
        Classification::Decided(roles) => roles,
        Classification::Uncertain(reason) => {
            return Err(Error::CurveRegionUncertain {
                operation: "loop-role classification",
                reason,
            });
        }
    };
    if paths.len() != roles.len() {
        return Err(Error::CurveRegionLoopCountMismatch {
            paths: paths.len(),
            roles: roles.len(),
        });
    }

    let mut mesh = ExactMesh::empty(Primitive::Lines);
    let mut loop_evidence = Vec::new();
    loop_evidence
        .try_reserve_exact(paths.len())
        .map_err(|_| Error::VertexCountOverflow { count: usize::MAX })?;
    for (path, role) in paths.iter().zip(roles) {
        let color = match role {
            CurveRegionLoopRole::Material => material_color,
            CurveRegionLoopRole::Hole => hole_color,
        };
        let polyline = match path.segment_certified(options, policy)? {
            Classification::Decided(polyline) => polyline,
            Classification::Uncertain(reason) => {
                return Err(Error::CurveSegmentationUncertain { reason });
            }
        };
        let certified = certified_curve_polyline_mesh(polyline, z.clone(), color)?;
        let first_vertex = mesh.vertex_count();
        let vertex_count = certified.mesh.vertex_count();
        let final_vertex_count = first_vertex
            .checked_add(vertex_count)
            .ok_or(Error::VertexCountOverflow { count: usize::MAX })?;
        mesh.vertices_mut()
            .try_reserve_exact(vertex_count)
            .map_err(|_| Error::VertexCountOverflow {
                count: final_vertex_count,
            })?;
        mesh.vertices_mut()
            .extend_from_slice(certified.mesh.vertices());
        loop_evidence.push(CertifiedCurveRegionLoopLineEvidence {
            role,
            certificate: certified.certificate,
            source_fragment_count: certified.source_fragment_count,
            first_vertex,
            vertex_count,
        });
    }

    Ok(CertifiedCurveRegionLineMesh {
        mesh,
        loop_evidence,
        path_materialization_certainty,
        role_certainty,
    })
}

fn certified_curve_polyline_mesh(
    polyline: CertifiedCurvePolyline2,
    z: Real,
    color: Color3,
) -> Result<CertifiedCurveLineMesh> {
    let vertex_count = polyline
        .certificate()
        .segment_count()
        .checked_mul(2)
        .ok_or(Error::VertexCountOverflow { count: usize::MAX })?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| Error::VertexCountOverflow {
            count: vertex_count,
        })?;
    for segment in polyline.points().windows(2) {
        for point in segment {
            vertices.push(ExactVertex::new(
                Point3::new(point.x().clone(), point.y().clone(), z.clone()),
                color,
            ));
        }
    }
    Ok(CertifiedCurveLineMesh {
        mesh: ExactMesh::new(Primitive::Lines, vertices),
        certificate: polyline.certificate().clone(),
        source_fragment_count: polyline.source_fragment_count(),
    })
}

/// Expand a native exact Hypermesh triangle mesh into a flat colored scene mesh.
///
/// The source positions remain [`Real`] values. Triangle indexing is checked
/// before any scene value is produced, and primitive-float projection remains
/// deferred to the normal Hypergraphics render boundary.
pub fn triangle_mesh(mesh: &TriangleMesh, color: Color3) -> Result<ExactMesh> {
    let vertex_count = mesh
        .triangles
        .len()
        .checked_mul(3)
        .ok_or(Error::VertexCountOverflow { count: usize::MAX })?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| Error::VertexCountOverflow {
            count: vertex_count,
        })?;

    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        for (corner, index) in triangle.indices().into_iter().enumerate() {
            let position =
                mesh.positions
                    .get(index)
                    .ok_or(Error::SourceTriangleIndexOutOfBounds {
                        triangle: triangle_index,
                        corner,
                        index,
                        vertex_count: mesh.positions.len(),
                    })?;
            vertices.push(ExactVertex::new(position.clone(), color));
        }
    }

    Ok(ExactMesh::new(Primitive::Triangles, vertices))
}

/// Build an exact RGB axis gizmo.
pub fn axes_mesh(length: Real, z_base: Real) -> Result<ExactMesh> {
    let red = Color3::RED;
    let green = Color3::GREEN;
    let blue = Color3::BLUE;
    let zero = Real::zero();
    let mut mesh = ExactMesh::empty(Primitive::Lines);

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        red,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(length.clone(), zero.clone(), z_base.clone()),
        red,
    ));

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        green,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), length.clone(), z_base.clone()),
        green,
    ));

    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero.clone(), z_base.clone()),
        blue,
    ));
    mesh.push(ExactVertex::new(
        Point3::new(zero.clone(), zero, length + z_base),
        blue,
    ));

    Ok(mesh)
}

/// Build exact XY grid lines centered at the origin.
///
/// `half_steps` is unsigned loop cardinality, while `step` remains an exact
/// hyperreal spacing. Allocation failure is reported instead of panicking.
pub fn grid_mesh(half_steps: u32, step: Real, color: Color3) -> Result<ExactMesh> {
    let coordinate_count_u64 = u64::from(half_steps) * 2 + 1;
    let vertex_count_u64 = coordinate_count_u64 * 4;
    let coordinate_count = usize::try_from(coordinate_count_u64)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let vertex_count =
        usize::try_from(vertex_count_u64).map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut coordinates = Vec::new();
    coordinates
        .try_reserve_exact(coordinate_count)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut vertices = Vec::new();
    vertices
        .try_reserve_exact(vertex_count)
        .map_err(|_| Error::GridSizeOverflow { half_steps })?;
    let mut mesh = ExactMesh::new(Primitive::Lines, vertices);
    let z = Real::zero();
    let half_steps = i64::from(half_steps);
    let extent = Real::from(half_steps) * step.clone();
    coordinates.extend((-half_steps..=half_steps).map(|index| Real::from(index) * step.clone()));

    for x in &coordinates {
        mesh.push(ExactVertex::new(
            Point3::new(x.clone(), -extent.clone(), z.clone()),
            color,
        ));
        mesh.push(ExactVertex::new(
            Point3::new(x.clone(), extent.clone(), z.clone()),
            color,
        ));
    }

    for y in &coordinates {
        mesh.push(ExactVertex::new(
            Point3::new(-extent.clone(), y.clone(), z.clone()),
            color,
        ));
        mesh.push(ExactVertex::new(
            Point3::new(extent.clone(), y.clone(), z.clone()),
            color,
        ));
    }

    Ok(mesh)
}

/// Triangulate an exact 2D polygon with `hypertri` and lift it to a flat z plane.
pub fn polygon_surface_mesh(
    context: &TriangulationContext,
    vertices: &[ExactPoint],
    hole_indices: &[usize],
    z: Real,
    color: Color3,
) -> Result<TriangulationOutcome<ExactMesh>> {
    Ok(
        hypertri::earcut(context, vertices, hole_indices)?.map(|indices| {
            let mut out = ExactMesh::new(Primitive::Triangles, Vec::with_capacity(indices.len()));
            for index in indices {
                let point = &vertices[index];
                out.push(ExactVertex::new(
                    Point3::new(point.x.clone(), point.y.clone(), z.clone()),
                    color,
                ));
            }
            out
        }),
    )
}

#[cfg(test)]
mod tests {
    use hypercurve::{
        CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveGeometry2, CurvePath2,
        CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2 as CurvePoint2,
    };
    use hyperlattice::Point3;
    use hyperlimit::PredicatePolicy;
    use hypermesh::{Triangle, TriangleMesh};
    use hyperreal::Real;
    use hypertri::{Point2, TriangulationContext};

    use super::*;

    #[test]
    fn native_triangle_mesh_stays_exact_until_render_export() {
        let one_third = Real::from(hyperreal::Rational::fraction(1, 3).unwrap());
        let source = TriangleMesh::new(
            vec![
                Point3::new(Real::zero(), Real::zero(), Real::zero()),
                Point3::new(one_third.clone(), Real::zero(), Real::zero()),
                Point3::new(Real::zero(), one_third.clone(), Real::zero()),
            ],
            vec![Triangle::new(0, 1, 2)],
        );
        let scene = triangle_mesh(&source, Color3::RED).unwrap();

        assert_eq!(scene.triangle_count(), 1);
        assert_eq!(scene.vertices()[1].position.x, one_third);
        assert_eq!(scene.to_render_vertices64().unwrap().len(), 3);
    }

    #[test]
    fn native_triangle_mesh_rejects_bad_source_indices() {
        let source = TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)]);

        assert!(matches!(
            triangle_mesh(&source, Color3::RED),
            Err(Error::SourceTriangleIndexOutOfBounds {
                triangle: 0,
                corner: 1,
                index: 1,
                vertex_count: 1,
            })
        ));
    }

    #[test]
    fn certified_curve_mesh_retains_exact_vertices_and_error_evidence() {
        let one_third = Real::from(hyperreal::Rational::fraction(1, 3).unwrap());
        let curve = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
            CurvePoint2::from_values(0, 0),
            CurvePoint2::new(one_third.clone(), Real::from(2)),
            CurvePoint2::new(Real::from(2), one_third.clone()),
            CurvePoint2::from_values(3, 0),
        )));
        let max_error = Real::from(hyperreal::Rational::fraction(1, 32).unwrap());
        let options =
            BezierFlatteningOptions::try_new(max_error.clone(), 16, &CurveContext::STRICT).unwrap();

        let certified = curve_line_mesh(
            &curve,
            &options,
            &CurveContext::STRICT,
            Real::from(5),
            Color3::GREEN,
        )
        .unwrap();

        assert_eq!(certified.mesh().primitive(), Primitive::Lines);
        assert_eq!(
            certified.mesh().vertex_count(),
            certified.segment_count() * 2
        );
        assert!(certified.segment_count() > 1);
        assert_eq!(certified.max_error(), &max_error);
        assert_eq!(certified.source_fragment_count(), 1);
        assert_eq!(certified.mesh().vertices()[0].position.x, Real::zero());
        assert_eq!(certified.mesh().vertices()[0].position.z, Real::from(5));
        assert_eq!(
            certified.mesh().vertices().last().unwrap().position.x,
            Real::from(3)
        );
    }

    #[test]
    fn certified_curve_path_mesh_covers_connected_source_fragments() {
        let join = CurvePoint2::from_values(2, 0);
        let line = Curve2::new(CurveGeometry2::Line(
            LineSeg2::try_new(CurvePoint2::from_values(0, 0), join.clone()).unwrap(),
        ));
        let cubic = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
            join,
            CurvePoint2::from_values(3, 2),
            CurvePoint2::from_values(4, -2),
            CurvePoint2::from_values(5, 0),
        )));
        let path = CurvePath2::try_new(vec![line, cubic]).unwrap();
        let options = BezierFlatteningOptions::try_new(
            Real::from(hyperreal::Rational::fraction(1, 16).unwrap()),
            16,
            &CurveContext::STRICT,
        )
        .unwrap();

        let certified = curve_path_line_mesh(
            &path,
            &options,
            &CurveContext::STRICT,
            Real::zero(),
            Color3::BLUE,
        )
        .unwrap();

        assert_eq!(certified.source_fragment_count(), 2);
        assert_eq!(certified.mesh().vertices()[0].position.x, Real::zero());
        assert_eq!(
            certified.mesh().vertices().last().unwrap().position.x,
            Real::from(5)
        );
    }

    #[test]
    fn certified_curve_mesh_reports_exhausted_subdivision() {
        let curve = Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
            CurvePoint2::from_values(0, 0),
            CurvePoint2::from_values(0, 100),
            CurvePoint2::from_values(100, 100),
            CurvePoint2::from_values(100, 0),
        )));
        let options = BezierFlatteningOptions::try_new(
            Real::from(hyperreal::Rational::fraction(1, 1_000_000).unwrap()),
            1,
            &CurveContext::STRICT,
        )
        .unwrap();

        assert!(matches!(
            curve_line_mesh(
                &curve,
                &options,
                &CurveContext::STRICT,
                Real::zero(),
                Color3::RED,
            ),
            Err(Error::CurveSegmentationUncertain {
                reason: hypercurve::UncertaintyReason::Unsupported,
            })
        ));
    }

    #[test]
    fn certified_region_mesh_retains_material_hole_roles_and_loop_evidence() {
        let line =
            |start, end| Curve2::new(CurveGeometry2::Line(LineSeg2::try_new(start, end).unwrap()));
        let outer = CurvePath2::try_new(vec![
            line(
                CurvePoint2::from_values(0, 0),
                CurvePoint2::from_values(4, 0),
            ),
            Curve2::new(CurveGeometry2::CubicBezier(CubicBezier2::new(
                CurvePoint2::from_values(4, 0),
                CurvePoint2::from_values(6, 1),
                CurvePoint2::from_values(6, 3),
                CurvePoint2::from_values(4, 4),
            ))),
            line(
                CurvePoint2::from_values(4, 4),
                CurvePoint2::from_values(0, 4),
            ),
            line(
                CurvePoint2::from_values(0, 4),
                CurvePoint2::from_values(0, 0),
            ),
        ])
        .unwrap();
        let hole = CurvePath2::try_new(vec![
            line(
                CurvePoint2::from_values(1, 1),
                CurvePoint2::from_values(1, 2),
            ),
            line(
                CurvePoint2::from_values(1, 2),
                CurvePoint2::from_values(2, 2),
            ),
            line(
                CurvePoint2::from_values(2, 2),
                CurvePoint2::from_values(2, 1),
            ),
            line(
                CurvePoint2::from_values(2, 1),
                CurvePoint2::from_values(1, 1),
            ),
        ])
        .unwrap();
        let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &[outer, hole],
            &[CurveRegionLoopRole::Material, CurveRegionLoopRole::Hole],
            &[FillRule::NonZero, FillRule::NonZero],
            &CurveContext::STRICT,
        )
        .unwrap()
        .into_value();
        let max_error = Real::from(hyperreal::Rational::fraction(1, 64).unwrap());
        let options =
            BezierFlatteningOptions::try_new(max_error.clone(), 16, &CurveContext::STRICT).unwrap();
        let material_color = Color3::GREEN;
        let hole_color = Color3::RED;

        let certified = curve_region_line_mesh(
            &region,
            &options,
            &CurveContext::STRICT,
            Real::zero(),
            material_color,
            hole_color,
        )
        .unwrap();

        assert_eq!(
            certified.path_materialization_certainty(),
            CurveCertainty::Certified
        );
        assert_eq!(certified.role_certainty(), CurveCertainty::Certified);
        assert_eq!(certified.loop_evidence().len(), 2);
        assert_eq!(
            certified.loop_evidence()[0].role(),
            CurveRegionLoopRole::Material
        );
        assert_eq!(
            certified.loop_evidence()[1].role(),
            CurveRegionLoopRole::Hole
        );
        assert_eq!(certified.loop_evidence()[0].max_error(), &max_error);
        assert!(certified.loop_evidence()[0].segment_count() > 4);
        assert_eq!(
            certified.mesh().vertex_count(),
            certified.segment_count() * 2
        );
        assert_eq!(
            certified.mesh().vertices()[certified.loop_evidence()[0].first_vertex()].color,
            material_color
        );
        assert_eq!(
            certified.mesh().vertices()[certified.loop_evidence()[1].first_vertex()].color,
            hole_color
        );
    }

    #[test]
    fn grid_keeps_exact_vertex_count() {
        let color = Color3::new(0.2, 0.2, 0.2).unwrap();
        let mesh = grid_mesh(2, Real::from(5), color).unwrap();
        assert_eq!(mesh.vertex_count(), 20);
    }

    #[test]
    fn polygon_surface_uses_hypertri_indices() {
        let vertices = vec![
            Point2::new(Real::from(0), Real::from(0)),
            Point2::new(Real::from(1), Real::from(0)),
            Point2::new(Real::from(1), Real::from(1)),
            Point2::new(Real::from(0), Real::from(1)),
        ];
        let color = Color3::new(0.8, 0.4, 0.1).unwrap();
        let context = TriangulationContext::new(PredicatePolicy::STRICT);
        let mesh = polygon_surface_mesh(&context, &vertices, &[], Real::zero(), color)
            .unwrap()
            .into_value();
        assert_eq!(mesh.primitive(), Primitive::Triangles);
        assert_eq!(mesh.triangle_count(), 2);
    }
}
