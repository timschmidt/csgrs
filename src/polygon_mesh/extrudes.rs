//! Profile extrusion and polygon-section lofting for [`PolygonMesh`](super::PolygonMesh).

use std::fmt::Debug;

use hypercurve::{
    Classification, FiniteProjectionOptions, FiniteTriangle2, triangulate_finite_rings,
};
use hyperlattice::{Point3, Real, Vector3};
use hyperreal::RealSign;

use crate::errors::ValidationError;
use crate::hyper_math::{hreal_sign, hunit_vector3};
use crate::sketch::Profile;
use crate::vertex::Vertex;

use super::{Polygon, PolygonMesh};

impl Profile {
    /// Extrude this profile into retained planar polygon faces along +Z.
    pub fn extrude_polygon_mesh<M: Clone + Debug + Send + Sync>(
        &self,
        height: Real,
        metadata: M,
    ) -> PolygonMesh<M> {
        self.extrude_vector_polygon_mesh(
            Vector3::from_xyz(Real::zero(), Real::zero(), height),
            metadata,
        )
    }

    /// Extrude this profile into retained planar polygon faces along `direction`.
    ///
    /// Caps remain triangles because they arise from polygon-with-holes
    /// triangulation. Every ruled boundary cell is retained as one quad; no
    /// intermediate [`crate::mesh::Mesh`] is constructed.
    pub fn extrude_vector_polygon_mesh<M: Clone + Debug + Send + Sync>(
        &self,
        direction: Vector3,
        metadata: M,
    ) -> PolygonMesh<M> {
        let Some(direction_unit) = hunit_vector3(&direction) else {
            return PolygonMesh::empty();
        };
        let points_down = matches!(hreal_sign(&direction.0[2]), Some(RealSign::Negative));
        let bottom_normal = -direction_unit.clone();
        let top_normal = direction_unit;
        let mut polygons = Vec::new();
        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");

        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        for profile in profiles {
            let Some(exterior) =
                orient_region_ring(profile.material().points().to_vec(), true)
            else {
                continue;
            };
            let Some(holes) = profile
                .holes()
                .iter()
                .map(|hole| orient_region_ring(hole.points().to_vec(), false))
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
            let triangles = finite_triangles_to_points(
                triangulate_finite_rings(&exterior, &hole_refs).unwrap_or_default(),
            );

            for triangle in &triangles {
                let [a, b, c] = if points_down {
                    triangle.clone()
                } else {
                    [
                        triangle[2].clone(),
                        triangle[1].clone(),
                        triangle[0].clone(),
                    ]
                };
                polygons.push(Polygon::new(
                    [a, b, c]
                        .into_iter()
                        .map(|point| {
                            Profile::apply_origin_transform_vertex(
                                Vertex::new(point, bottom_normal.clone()),
                                self.origin_transform.clone(),
                            )
                        })
                        .collect(),
                    metadata.clone(),
                ));
            }

            for triangle in &triangles {
                let translated = triangle.clone().map(|point| point + &direction);
                let [a, b, c] = if points_down {
                    [
                        translated[2].clone(),
                        translated[1].clone(),
                        translated[0].clone(),
                    ]
                } else {
                    translated
                };
                polygons.push(Polygon::new(
                    [a, b, c]
                        .into_iter()
                        .map(|point| {
                            Profile::apply_origin_transform_vertex(
                                Vertex::new(point, top_normal.clone()),
                                self.origin_transform.clone(),
                            )
                        })
                        .collect(),
                    metadata.clone(),
                ));
            }

            push_ring_sides(
                self,
                &exterior,
                &direction,
                points_down,
                &metadata,
                &mut polygons,
            );
            for hole in &holes {
                push_ring_sides(self, hole, &direction, points_down, &metadata, &mut polygons);
            }
        }

        for wire in self.project_wire_polylines(&options) {
            push_ring_sides(
                self,
                wire.points(),
                &direction,
                points_down,
                &metadata,
                &mut polygons,
            );
        }
        PolygonMesh::from_polygons(polygons)
    }
}

impl<M: Clone + Debug + Send + Sync> PolygonMesh<M> {
    /// Loft a closed polygon mesh through corresponding planar sections.
    ///
    /// End sections retain their original face boundary. Potentially warped
    /// side cells are emitted as two triangles so every output face remains
    /// planar.
    pub fn loft(sections: &[Polygon<M>]) -> Result<Self, ValidationError> {
        if sections.len() < 2 {
            return Err(ValidationError::FieldLessThan {
                name: "sections",
                min: 2,
            });
        }
        let vertex_count = sections[0].vertices().len();
        if vertex_count < 3 {
            return Err(ValidationError::InvalidArguments);
        }
        if let Some(section) = sections
            .iter()
            .find(|section| section.vertices().len() != vertex_count)
        {
            return Err(ValidationError::MismatchedVertexCount {
                left: vertex_count,
                right: section.vertices().len(),
            });
        }

        let sum_positions = |section: &Polygon<M>| {
            section
                .vertices()
                .iter()
                .fold(Vector3::zero(), |sum, vertex| {
                    sum + vertex.position.to_vector()
                })
        };
        let travel = sum_positions(sections.last().expect("two sections exist"))
            - sum_positions(&sections[0]);
        let Some(orientation) = hreal_sign(&sections[0].plane().normal().dot(&travel)) else {
            return Err(ValidationError::InvalidArguments);
        };
        if orientation == RealSign::Zero {
            return Err(ValidationError::InvalidArguments);
        }
        let forward_winding = orientation == RealSign::Positive;

        let mut bottom = sections[0].clone();
        if forward_winding {
            bottom.flip();
        }
        let mut top = sections.last().expect("two sections exist").clone();
        if !forward_winding {
            top.flip();
        }
        let mut polygons = vec![bottom, top];
        for pair in sections.windows(2) {
            for index in 0..vertex_count {
                let next = (index + 1) % vertex_count;
                let mut first = vec![
                    pair[0].vertices()[index].clone(),
                    pair[0].vertices()[next].clone(),
                    pair[1].vertices()[next].clone(),
                ];
                let mut second = vec![
                    pair[0].vertices()[index].clone(),
                    pair[1].vertices()[next].clone(),
                    pair[1].vertices()[index].clone(),
                ];
                if !forward_winding {
                    first.reverse();
                    second.reverse();
                }
                polygons.push(Polygon::new(first, pair[0].metadata().clone()));
                polygons.push(Polygon::new(second, pair[0].metadata().clone()));
            }
        }
        Ok(Self::from_polygons(polygons))
    }
}

fn push_ring_sides<M: Clone + Debug + Send + Sync>(
    profile: &Profile,
    ring: &[[f64; 2]],
    direction: &Vector3,
    reverse: bool,
    metadata: &M,
    polygons: &mut Vec<Polygon<M>>,
) {
    for edge in ring.windows(2) {
        let Some(bottom_a) = finite_point(edge[0]) else {
            continue;
        };
        let Some(bottom_b) = finite_point(edge[1]) else {
            continue;
        };
        let top_a = bottom_a.clone() + direction;
        let top_b = bottom_b.clone() + direction;
        let edge_direction = &bottom_b - &bottom_a;
        let Ok(normal) = edge_direction.cross(direction).normalize() else {
            continue;
        };
        let normal = if reverse { -normal } else { normal };
        let points = if reverse {
            [bottom_b, bottom_a, top_a, top_b]
        } else {
            [bottom_a, bottom_b, top_b, top_a]
        };
        polygons.push(Polygon::new(
            points
                .into_iter()
                .map(|point| {
                    Profile::apply_origin_transform_vertex(
                        Vertex::new(point, normal.clone()),
                        profile.origin_transform.clone(),
                    )
                })
                .collect(),
            metadata.clone(),
        ));
    }
}

fn finite_point(point: [f64; 2]) -> Option<Point3> {
    Some(Point3::new(
        Real::try_from(point[0]).ok()?,
        Real::try_from(point[1]).ok()?,
        Real::zero(),
    ))
}

fn finite_triangles_to_points(triangles: Vec<FiniteTriangle2>) -> Vec<[Point3; 3]> {
    triangles
        .into_iter()
        .filter_map(|triangle| {
            Some([
                finite_point(triangle[0])?,
                finite_point(triangle[1])?,
                finite_point(triangle[2])?,
            ])
        })
        .collect()
}

fn orient_region_ring(points: Vec<[f64; 2]>, counterclockwise: bool) -> Option<Vec<[f64; 2]>> {
    if points.iter().flatten().any(|value| !value.is_finite()) {
        return None;
    }
    let mut points = points;
    while points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    points.dedup();
    if points.len() < 3 {
        return None;
    }
    let exact = points
        .iter()
        .map(|point| {
            Some(hyperlimit::Point2::new(
                Real::try_from(point[0]).ok()?,
                Real::try_from(point[1]).ok()?,
            ))
        })
        .collect::<Option<Vec<_>>>()?;
    let is_counterclockwise = match hyperlimit::ring_area_sign(&exact).value()? {
        hyperlimit::Sign::Positive => true,
        hyperlimit::Sign::Negative => false,
        hyperlimit::Sign::Zero => return None,
    };
    if is_counterclockwise != counterclockwise {
        points.reverse();
    }
    points.push(points[0]);
    Some(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square_section(z: i32, width: i32) -> Polygon<&'static str> {
        Polygon::new(
            [[0, 0], [width, 0], [width, width], [0, width]]
                .into_iter()
                .map(|[x, y]| {
                    Vertex::new(
                        Point3::new(Real::from(x), Real::from(y), Real::from(z)),
                        Vector3::z(),
                    )
                })
                .collect(),
            "loft",
        )
    }

    #[test]
    fn profile_extrusion_retains_side_quads_and_converts_to_triangles() {
        let profile = Profile::square(Real::from(2));
        let polygon_mesh = profile.extrude_polygon_mesh(Real::from(3), "extrude");

        assert_eq!(polygon_mesh.polygons.len(), 8);
        assert_eq!(
            polygon_mesh
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 4)
                .count(),
            4
        );
        let mesh = polygon_mesh.triangulate();
        assert_eq!(mesh.polygons.len(), 12);
        assert!(mesh.polygons.iter().all(|face| face.vertices().len() == 3));
    }

    #[test]
    fn polygon_loft_retains_caps_and_planar_triangular_sides() {
        let sections = [square_section(0, 2), square_section(3, 3)];
        let loft = PolygonMesh::loft(&sections).unwrap();

        assert_eq!(loft.polygons.len(), 10);
        assert_eq!(loft.polygons[0].vertices().len(), 4);
        assert_eq!(loft.polygons[1].vertices().len(), 4);
        assert!(
            loft.polygons[2..]
                .iter()
                .all(|polygon| polygon.vertices().len() == 3)
        );
        assert_eq!(loft.triangulate().polygons.len(), 12);
    }
}
