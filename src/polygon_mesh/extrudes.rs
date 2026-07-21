//! Profile extrusion and polygon-section lofting for [`PolygonMesh`](super::PolygonMesh).

use std::fmt::Debug;

use hypercurve::{
    Classification, FiniteProjectionOptions, FiniteTriangle2, triangulate_finite_rings,
};
use hyperlattice::{Matrix4, Point3, Real, Vector3};
use hyperreal::RealSign;

use crate::errors::ValidationError;
use crate::hyper_math::{
    hreal_sign, hreal_try_cmp, hrotation_between_vectors, htranslation_matrix, hunit_vector3,
};
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

    /// Extrude along +Z while linearly varying rotation and XY scale.
    ///
    /// Potentially warped side cells are split into planar triangles. End
    /// caps use the same polygon-with-holes triangulation as linear extrusion.
    pub fn extrude_twisted_polygon_mesh<M: Clone + Debug + Send + Sync>(
        &self,
        height: Real,
        twist_degrees: Real,
        end_scale: [Real; 2],
        slices: usize,
        metadata: M,
    ) -> Result<PolygonMesh<M>, ValidationError> {
        if slices < 1 {
            return Err(ValidationError::FieldLessThan {
                name: "slices",
                min: 1,
            });
        }
        let Some(height_sign) = hreal_sign(&height) else {
            return Err(ValidationError::InvalidArguments);
        };
        if height_sign == RealSign::Zero
            || hreal_sign(&twist_degrees).is_none()
            || end_scale.iter().any(|scale| {
                !matches!(
                    hreal_try_cmp(scale, Real::zero()),
                    Some(std::cmp::Ordering::Equal | std::cmp::Ordering::Greater)
                )
            })
        {
            return Err(ValidationError::InvalidArguments);
        }

        #[derive(Clone)]
        struct Slice {
            sin: Real,
            cos: Real,
            scale_x: Real,
            scale_y: Real,
            height: Real,
        }

        let mut slice_parameters = Vec::with_capacity(slices + 1);
        for index in 0..=slices {
            let fraction = (Real::from(index as u64) / Real::from(slices as u64))
                .map_err(|_| ValidationError::InvalidArguments)?;
            let radians = (twist_degrees.clone() * fraction.clone() * Real::pi()
                / Real::from(180_u16))
            .map_err(|_| ValidationError::InvalidArguments)?;
            slice_parameters.push(Slice {
                sin: radians.clone().sin(),
                cos: radians.cos(),
                scale_x: Real::one() + (end_scale[0].clone() - Real::one()) * fraction.clone(),
                scale_y: Real::one() + (end_scale[1].clone() - Real::one()) * fraction.clone(),
                height: height.clone() * fraction,
            });
        }

        let map_point = |point: [f64; 2], slice: &Slice| -> Option<Point3> {
            let x = Real::try_from(point[0]).ok()? * slice.scale_x.clone();
            let y = Real::try_from(point[1]).ok()? * slice.scale_y.clone();
            Some(Point3::new(
                x.clone() * slice.cos.clone() - y.clone() * slice.sin.clone(),
                x * slice.sin.clone() + y * slice.cos.clone(),
                slice.height.clone(),
            ))
        };
        let height_positive = height_sign == RealSign::Positive;
        let mut polygons = Vec::new();
        let options = finite_projection_options();

        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        for profile in profiles {
            let Some(material) =
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
            emit_twisted_ring(
                &material,
                &slice_parameters,
                &map_point,
                !height_positive,
                &metadata,
                &mut polygons,
            );
            for hole in &holes {
                emit_twisted_ring(
                    hole,
                    &slice_parameters,
                    &map_point,
                    !height_positive,
                    &metadata,
                    &mut polygons,
                );
            }

            let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
            for triangle in triangulate_finite_rings(&material, &hole_refs).unwrap_or_default()
            {
                let bottom = triangle
                    .map(|point| map_point(point, &slice_parameters[0]))
                    .into_iter()
                    .collect::<Option<Vec<_>>>();
                if let Some(bottom) = bottom {
                    push_planar_face(bottom, height_positive, &metadata, &mut polygons);
                }
                let top = triangle
                    .map(|point| map_point(point, &slice_parameters[slices]))
                    .into_iter()
                    .collect::<Option<Vec<_>>>();
                if let Some(top) = top {
                    push_planar_face(top, !height_positive, &metadata, &mut polygons);
                }
            }
        }
        for wire in self.project_wire_polylines(&options) {
            emit_twisted_ring(
                wire.points(),
                &slice_parameters,
                &map_point,
                !height_positive,
                &metadata,
                &mut polygons,
            );
        }
        Ok(PolygonMesh::from_polygons(polygons))
    }

    /// Revolve this profile around the Y axis into retained planar faces.
    pub fn revolve_polygon_mesh<M: Clone + Debug + Send + Sync>(
        &self,
        angle_degs: Real,
        segments: usize,
        metadata: M,
    ) -> Result<PolygonMesh<M>, ValidationError> {
        if segments < 2 {
            return Err(ValidationError::FieldLessThan {
                name: "segments",
                min: 2,
            });
        }
        let Some(angle_sign) = hreal_sign(&angle_degs) else {
            return Err(ValidationError::InvalidArguments);
        };
        if angle_sign == RealSign::Zero
            || matches!(
                hreal_try_cmp(angle_degs.clone().abs(), Real::from(360_u16)),
                Some(std::cmp::Ordering::Greater) | None
            )
        {
            return Err(ValidationError::InvalidArguments);
        }
        let full_revolve = matches!(
            hreal_try_cmp(angle_degs.clone().abs(), Real::from(360_u16)),
            Some(std::cmp::Ordering::Equal)
        );
        let angle_positive = angle_sign == RealSign::Positive;
        let slice_count = if full_revolve { segments } else { segments + 1 };
        let samples = (0..slice_count)
            .map(|slice| {
                let fraction =
                    (Real::from(slice as u64) / Real::from(segments as u64)).ok()?;
                let radians =
                    (angle_degs.clone() * Real::pi() * fraction / Real::from(180_u16)).ok()?;
                Some((radians.clone().sin(), radians.cos()))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or(ValidationError::InvalidArguments)?;
        let map_point = |point: [f64; 2], sample: &(Real, Real)| -> Option<Point3> {
            let radius = Real::try_from(point[0]).ok()?;
            Some(Point3::new(
                radius.clone() * sample.1.clone(),
                Real::try_from(point[1]).ok()?,
                radius * sample.0.clone(),
            ))
        };
        let mut polygons = Vec::new();
        let options = finite_projection_options();
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        for profile in profiles {
            let Some(material) =
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
            let Some(radial_positive) = radial_orientation(&material) else {
                return Err(ValidationError::InvalidArguments);
            };
            if holes
                .iter()
                .any(|hole| radial_orientation(hole) != Some(radial_positive))
            {
                return Err(ValidationError::InvalidArguments);
            }
            emit_revolved_ring(
                &material,
                angle_positive == radial_positive,
                full_revolve,
                segments,
                &samples,
                &map_point,
                &metadata,
                &mut polygons,
            );
            for hole in &holes {
                emit_revolved_ring(
                    hole,
                    angle_positive == radial_positive,
                    full_revolve,
                    segments,
                    &samples,
                    &map_point,
                    &metadata,
                    &mut polygons,
                );
            }
            if !full_revolve {
                let reverse_start = angle_positive == radial_positive;
                let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
                for triangle in
                    triangulate_finite_rings(&material, &hole_refs).unwrap_or_default()
                {
                    let start = triangle
                        .map(|point| map_point(point, &samples[0]))
                        .into_iter()
                        .collect::<Option<Vec<_>>>();
                    if let Some(start) = start {
                        push_planar_face(start, reverse_start, &metadata, &mut polygons);
                    }
                    let end = triangle
                        .map(|point| map_point(point, samples.last().expect("end sample")))
                        .into_iter()
                        .collect::<Option<Vec<_>>>();
                    if let Some(end) = end {
                        push_planar_face(end, !reverse_start, &metadata, &mut polygons);
                    }
                }
            }
        }
        for wire in self.project_wire_polylines(&options) {
            let Some(radial_positive) = radial_orientation(wire.points()) else {
                return Err(ValidationError::InvalidArguments);
            };
            emit_revolved_ring(
                wire.points(),
                angle_positive == radial_positive,
                full_revolve,
                segments,
                &samples,
                &map_point,
                &metadata,
                &mut polygons,
            );
        }
        Ok(PolygonMesh::from_polygons(polygons))
    }

    /// Sweep this profile along a point path using rotation-minimizing frames.
    ///
    /// Open paths receive triangulated end caps. Closed paths omit caps.
    /// Arbitrary side cells are emitted as two planar triangles.
    pub fn sweep_polygon_mesh<M: Clone + Debug + Send + Sync>(
        &self,
        path: &[Point3],
        metadata: M,
    ) -> PolygonMesh<M> {
        if path.len() < 2 {
            return PolygonMesh::empty();
        }
        let mut canonical_path = Vec::with_capacity(path.len());
        for point in path {
            if canonical_path
                .last()
                .is_some_and(|previous| points_equal(previous, point))
            {
                continue;
            }
            canonical_path.push(point.clone());
        }
        if canonical_path.len() < 2 {
            return PolygonMesh::empty();
        }
        let path_is_closed =
            points_equal(&canonical_path[0], canonical_path.last().expect("path end"));
        if path_is_closed {
            canonical_path.pop();
        }
        if canonical_path.len() < if path_is_closed { 3 } else { 2 } {
            return PolygonMesh::empty();
        }
        let path = canonical_path;

        let point_count = path.len();
        let segment_count = if path_is_closed {
            point_count
        } else {
            point_count - 1
        };
        let mut segment_directions = Vec::with_capacity(segment_count);
        for index in 0..segment_count {
            let Some(direction) =
                hunit_vector3(&(&path[(index + 1) % point_count] - &path[index]))
            else {
                return PolygonMesh::empty();
            };
            segment_directions.push(direction);
        }
        let mut tangents = Vec::with_capacity(point_count);
        for index in 0..point_count {
            let tangent = if !path_is_closed && index == 0 {
                segment_directions[0].clone()
            } else if !path_is_closed && index == point_count - 1 {
                segment_directions[segment_count - 1].clone()
            } else {
                let incoming =
                    &segment_directions[(index + segment_count - 1) % segment_count];
                let outgoing = &segment_directions[index % segment_count];
                let Some(tangent) = hunit_vector3(&(incoming + outgoing)) else {
                    return PolygonMesh::empty();
                };
                tangent
            };
            tangents.push(tangent);
        }

        let Some(mut orientation) = hrotation_between_vectors(&Vector3::z(), &tangents[0])
        else {
            return PolygonMesh::empty();
        };
        let Some(first_translation) = htranslation_matrix(&path[0].to_vector()) else {
            return PolygonMesh::empty();
        };
        let mut transforms = Vec::with_capacity(point_count);
        transforms.push(first_translation * orientation.clone());
        for index in 1..point_count {
            let Some(rotation) =
                hrotation_between_vectors(&tangents[index - 1], &tangents[index])
            else {
                return PolygonMesh::empty();
            };
            orientation = rotation * orientation;
            let Some(translation) = htranslation_matrix(&path[index].to_vector()) else {
                return PolygonMesh::empty();
            };
            transforms.push(translation * orientation.clone());
        }

        struct Ring {
            slices: Vec<Vec<Point3>>,
        }
        let mut rings = Vec::new();
        let mut add_ring = |coordinates: Vec<[f64; 2]>| {
            if coordinates.len() < 2 {
                return;
            }
            let slices = transforms
                .iter()
                .map(|transform| {
                    coordinates
                        .iter()
                        .filter_map(|point| transform_finite_point(*point, transform))
                        .collect::<Vec<_>>()
                })
                .collect::<Vec<_>>();
            if slices.iter().all(|slice| slice.len() == coordinates.len()) {
                rings.push(Ring { slices });
            }
        };

        let options = finite_projection_options();
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => Vec::new(),
        };
        let mut cap_profiles = Vec::new();
        for profile in profiles {
            let Some(material) =
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
            add_ring(material.clone());
            for hole in &holes {
                add_ring(hole.clone());
            }
            cap_profiles.push((material, holes));
        }
        for wire in self.project_wire_polylines(&options) {
            add_ring(wire.into_points());
        }
        if rings.is_empty() {
            return PolygonMesh::empty();
        }

        let mut polygons = Vec::new();
        let path_steps = if path_is_closed {
            point_count
        } else {
            point_count - 1
        };
        for ring in &rings {
            let edge_count = ring.slices[0].len() - 1;
            for path_index in 0..path_steps {
                let next_path = (path_index + 1) % point_count;
                for edge in 0..edge_count {
                    let points = [
                        ring.slices[path_index][edge].clone(),
                        ring.slices[path_index][edge + 1].clone(),
                        ring.slices[next_path][edge + 1].clone(),
                        ring.slices[next_path][edge].clone(),
                    ];
                    push_planar_face(
                        vec![points[0].clone(), points[1].clone(), points[2].clone()],
                        false,
                        &metadata,
                        &mut polygons,
                    );
                    push_planar_face(
                        vec![points[0].clone(), points[2].clone(), points[3].clone()],
                        false,
                        &metadata,
                        &mut polygons,
                    );
                }
            }
        }

        if !path_is_closed {
            for (material, holes) in cap_profiles {
                let hole_refs = holes.iter().map(Vec::as_slice).collect::<Vec<_>>();
                for triangle in
                    triangulate_finite_rings(&material, &hole_refs).unwrap_or_default()
                {
                    let start = triangle
                        .map(|point| transform_finite_point(point, &transforms[0]))
                        .into_iter()
                        .collect::<Option<Vec<_>>>();
                    if let Some(start) = start {
                        push_planar_face(start, true, &metadata, &mut polygons);
                    }
                    let end = triangle
                        .map(|point| {
                            transform_finite_point(
                                point,
                                transforms.last().expect("open path end transform"),
                            )
                        })
                        .into_iter()
                        .collect::<Option<Vec<_>>>();
                    if let Some(end) = end {
                        push_planar_face(end, false, &metadata, &mut polygons);
                    }
                }
            }
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

fn finite_projection_options() -> FiniteProjectionOptions {
    FiniteProjectionOptions::try_new(1e-3)
        .expect("positive finite projection tolerance is valid")
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    let left = hyperlimit::Point3::new(left.x.clone(), left.y.clone(), left.z.clone());
    let right = hyperlimit::Point3::new(right.x.clone(), right.y.clone(), right.z.clone());
    matches!(hyperlimit::point3_equal(&left, &right).value(), Some(true))
}

fn transform_finite_point(point: [f64; 2], transform: &Matrix4) -> Option<Point3> {
    materialize_sample_point(transform.transform_point3(&finite_point(point)?).ok()?)
}

fn materialize_sample_point(point: Point3) -> Option<Point3> {
    Some(Point3::new(
        Real::try_from(point.x.to_f64_lossy()?).ok()?,
        Real::try_from(point.y.to_f64_lossy()?).ok()?,
        Real::try_from(point.z.to_f64_lossy()?).ok()?,
    ))
}

fn emit_twisted_ring<T, M, Map>(
    ring: &[[f64; 2]],
    slices: &[T],
    map_point: &Map,
    reverse: bool,
    metadata: &M,
    polygons: &mut Vec<Polygon<M>>,
) where
    M: Clone + Debug + Send + Sync,
    Map: Fn([f64; 2], &T) -> Option<Point3>,
{
    for edge in ring.windows(2) {
        for slice in 0..slices.len() - 1 {
            let Some(points) = [
                map_point(edge[0], &slices[slice]),
                map_point(edge[1], &slices[slice]),
                map_point(edge[1], &slices[slice + 1]),
                map_point(edge[0], &slices[slice + 1]),
            ]
            .into_iter()
            .collect::<Option<Vec<_>>>() else {
                continue;
            };
            push_planar_face(
                vec![points[0].clone(), points[1].clone(), points[2].clone()],
                reverse,
                metadata,
                polygons,
            );
            push_planar_face(
                vec![points[0].clone(), points[2].clone(), points[3].clone()],
                reverse,
                metadata,
                polygons,
            );
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_revolved_ring<M, Map>(
    ring: &[[f64; 2]],
    forward: bool,
    full_revolve: bool,
    segments: usize,
    samples: &[(Real, Real)],
    map_point: &Map,
    metadata: &M,
    polygons: &mut Vec<Polygon<M>>,
) where
    M: Clone + Debug + Send + Sync,
    Map: Fn([f64; 2], &(Real, Real)) -> Option<Point3>,
{
    for edge in ring.windows(2) {
        for slice in 0..segments {
            let next = if full_revolve {
                (slice + 1) % samples.len()
            } else {
                slice + 1
            };
            let Some(points) = [
                map_point(edge[0], &samples[slice]),
                map_point(edge[1], &samples[slice]),
                map_point(edge[1], &samples[next]),
                map_point(edge[0], &samples[next]),
            ]
            .into_iter()
            .collect::<Option<Vec<_>>>() else {
                continue;
            };
            push_planar_face(points, !forward, metadata, polygons);
        }
    }
}

fn push_planar_face<M: Clone + Debug + Send + Sync>(
    mut points: Vec<Point3>,
    reverse: bool,
    metadata: &M,
    polygons: &mut Vec<Polygon<M>>,
) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(&points[0], points.last().expect("face end")) {
        points.pop();
    }
    if points.len() < 3 {
        return;
    }
    let base = points[0].to_vector();
    let nondegenerate = (1..points.len() - 1).any(|left| {
        (left + 1..points.len()).any(|right| {
            let normal = (points[left].to_vector() - base.clone())
                .cross(&(points[right].to_vector() - base.clone()));
            matches!(hreal_sign(&normal.dot(&normal)), Some(RealSign::Positive))
        })
    });
    if !nondegenerate {
        return;
    }
    if reverse {
        points.reverse();
    }
    let mut polygon = Polygon::new(
        points
            .into_iter()
            .map(|point| Vertex::new(point, Vector3::zeros()))
            .collect(),
        metadata.clone(),
    );
    polygon.set_new_normal();
    polygons.push(polygon);
}

fn radial_orientation(ring: &[[f64; 2]]) -> Option<bool> {
    let has_negative = ring.iter().any(|point| point[0] < 0.0);
    let has_positive = ring.iter().any(|point| point[0] > 0.0);
    match (has_negative, has_positive) {
        (false, false) | (true, true) => None,
        (false, true) => Some(true),
        (true, false) => Some(false),
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
    use crate::csg::CSG;

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
        let profile = Profile::square(Real::from(2)).translate(
            Real::from(-1),
            Real::from(-1),
            Real::zero(),
        );
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

    #[test]
    fn twisted_polygon_extrusion_emits_planar_triangles_directly() {
        let profile = Profile::square(Real::from(2));
        let twisted = profile
            .extrude_twisted_polygon_mesh(
                Real::from(4),
                Real::from(90),
                [Real::one(), Real::one()],
                2,
                "twisted",
            )
            .expect("valid twisted extrusion");

        assert_eq!(twisted.polygons.len(), 20);
        assert!(twisted.polygons.iter().all(|polygon| {
            polygon.vertices().len() == 3 && polygon.metadata() == &"twisted"
        }));
        assert!(twisted.triangulate().is_manifold());
    }

    #[test]
    fn full_polygon_revolve_retains_quad_surface_cells() {
        let profile = Profile::rectangle(Real::one(), Real::one()).translate(
            Real::one(),
            Real::zero(),
            Real::zero(),
        );
        let revolved = profile
            .revolve_polygon_mesh(Real::from(360), 8, "revolve")
            .expect("valid full revolution");

        assert_eq!(revolved.polygons.len(), 32);
        assert!(revolved.polygons.iter().all(|polygon| {
            polygon.vertices().len() == 4 && polygon.metadata() == &"revolve"
        }));
        let mesh = revolved.triangulate();
        assert_eq!(mesh.polygons.len(), 64);
        assert!(mesh.is_manifold());
    }

    #[test]
    fn partial_polygon_revolve_adds_triangulated_end_caps() {
        let profile = Profile::rectangle(Real::one(), Real::one()).translate(
            Real::one(),
            Real::zero(),
            Real::zero(),
        );
        let revolved = profile
            .revolve_polygon_mesh(Real::from(180), 4, ())
            .expect("valid partial revolution");

        assert_eq!(revolved.polygons.len(), 20);
        assert_eq!(
            revolved
                .polygons
                .iter()
                .filter(|polygon| polygon.vertices().len() == 3)
                .count(),
            4
        );
        assert!(revolved.triangulate().is_manifold());
    }

    #[test]
    fn polygon_twist_and_revolve_validate_parameters() {
        let profile = Profile::square(Real::one());
        assert!(matches!(
            profile.extrude_twisted_polygon_mesh(
                Real::one(),
                Real::zero(),
                [Real::one(), Real::one()],
                0,
                (),
            ),
            Err(ValidationError::FieldLessThan {
                name: "slices",
                min: 1
            })
        ));
        assert!(matches!(
            profile.revolve_polygon_mesh(Real::from(361), 8, ()),
            Err(ValidationError::InvalidArguments)
        ));
    }

    #[test]
    fn open_polygon_sweep_emits_triangular_sides_and_caps() {
        let profile = Profile::square(Real::from(2)).translate(
            Real::from(-1),
            Real::from(-1),
            Real::zero(),
        );
        let path = [
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), Real::from(2)),
            Point3::new(Real::from(2), Real::zero(), Real::from(2)),
        ];
        let swept = profile.sweep_polygon_mesh(&path, "sweep");

        assert_eq!(swept.polygons.len(), 20);
        assert!(swept.polygons.iter().all(|polygon| {
            polygon.vertices().len() == 3 && polygon.metadata() == &"sweep"
        }));
        assert!(swept.triangulate().is_manifold());
    }

    #[test]
    fn polygon_sweep_rejects_degenerate_paths() {
        let profile = Profile::square(Real::one());
        let point = Point3::origin();

        assert!(
            profile
                .sweep_polygon_mesh(std::slice::from_ref(&point), ())
                .polygons
                .is_empty()
        );
        assert!(
            profile
                .sweep_polygon_mesh(&[point.clone(), point], ())
                .polygons
                .is_empty()
        );
    }
}
