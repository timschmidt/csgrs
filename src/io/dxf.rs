//! DXF import support for converting 2D CAD entities into `csgrs` geometry.

use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::sketch::Profile;
use crate::triangulated::Triangulated3D;
use crate::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};
use hyperreal::RealSign;
use std::error::Error;
use std::fmt::Debug;

use dxf::Drawing;
use dxf::entities::*;
use hypercurve::Point2;
use std::io::Cursor;

fn real_from_f64(value: f64) -> Result<Real, hyperreal::Problem> {
    Real::try_from(value)
}

fn real_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .unwrap_or(0.0)
}

fn real_positive(value: &Real) -> bool {
    matches!(value.refine_sign_until(128), Some(RealSign::Positive))
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    #[doc = " Import a Mesh object from DXF data."]
    #[doc = ""]
    #[doc = " ## Parameters"]
    #[doc = " - `dxf_data`: A byte slice containing the DXF file data."]
    #[doc = " - `metadata`: metadata that will be attached to all polygons of the resulting `Mesh`"]
    #[doc = ""]
    #[doc = " ## Returns"]
    #[doc = " A `Result` containing the Mesh object or an error if parsing fails."]
    ///
    /// DXF entity coordinates are treated as primitive file-boundary data.
    /// Points and normals are promoted through hyperlattice adapters before
    /// becoming mesh vertices, so malformed or non-finite entity data is
    /// skipped instead of being sanitized into CAD topology. This follows Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn from_dxf(dxf_data: &[u8], metadata: M) -> Result<Mesh<M>, Box<dyn Error>> {
        let drawing = Drawing::load(&mut Cursor::new(dxf_data))?;
        let mut polygons = Vec::new();

        for entity in drawing.entities() {
            match &entity.specific {
                EntityType::Line(_line) => {},
                EntityType::Polyline(polyline) => {
                    if polyline.is_closed() {
                        let mut verts = Vec::new();
                        for vertex in polyline.vertices() {
                            let point = Point3::new(
                                real_from_f64(vertex.location.x)?,
                                real_from_f64(vertex.location.y)?,
                                real_from_f64(vertex.location.z)?,
                            );
                            verts.push(Vertex::new(point, Vector3::z()));
                        }
                        if verts.len() >= 3 {
                            polygons.push(Polygon::new(verts, metadata.clone()));
                        }
                    }
                },
                EntityType::Circle(circle) => {
                    let center = Point3::new(
                        real_from_f64(circle.center.x)?,
                        real_from_f64(circle.center.y)?,
                        real_from_f64(circle.center.z)?,
                    );
                    let radius = real_from_f64(circle.radius)?;
                    if !real_positive(&radius) {
                        continue;
                    }
                    let segments = 32;
                    let mut verts = Vec::with_capacity(segments + 1);
                    let Ok(normal) = Vector3::from_xyz(
                        real_from_f64(circle.normal.x)?,
                        real_from_f64(circle.normal.y)?,
                        real_from_f64(circle.normal.z)?,
                    )
                    .normalize_checked() else {
                        continue;
                    };
                    for i in 0..segments {
                        let theta = Real::from(2_u8)
                            * Real::pi()
                            * (Real::from(i as u64) / Real::from(segments as u64))?;
                        let x = center.x.clone() + radius.clone() * theta.clone().cos();
                        let y = center.y.clone() + radius.clone() * theta.sin();
                        let z = center.z.clone();
                        verts.push(Vertex::new(Point3::new(x, y, z), normal.clone()));
                    }
                    polygons.push(Polygon::new(verts, metadata.clone()));
                },
                EntityType::Solid(solid) => {
                    let thickness = real_from_f64(solid.thickness)?;
                    let extrusion_direction = Vector3::from_xyz(
                        real_from_f64(solid.extrusion_direction.x)?,
                        real_from_f64(solid.extrusion_direction.y)?,
                        real_from_f64(solid.extrusion_direction.z)?,
                    );
                    let points = [
                        Point2::new(
                            real_from_f64(solid.first_corner.x)?,
                            real_from_f64(solid.first_corner.y)?,
                        ),
                        Point2::new(
                            real_from_f64(solid.second_corner.x)?,
                            real_from_f64(solid.second_corner.y)?,
                        ),
                        Point2::new(
                            real_from_f64(solid.third_corner.x)?,
                            real_from_f64(solid.third_corner.y)?,
                        ),
                        Point2::new(
                            real_from_f64(solid.fourth_corner.x)?,
                            real_from_f64(solid.fourth_corner.y)?,
                        ),
                    ];
                    let ring = [
                        points[0].clone(),
                        points[1].clone(),
                        points[2].clone(),
                        points[3].clone(),
                        points[0].clone(),
                    ];
                    let extruded = Profile::polygon_points(&ring, metadata.clone())
                        .extrude_vector(extrusion_direction * thickness)
                        .polygons;
                    polygons.extend(extruded);
                },
                _ => {},
            }
        }

        Ok(Mesh::from_polygons(&polygons, metadata))
    }
}

#[doc = " Export any `Triangulated3D` shape to DXF format."]
#[doc = ""]
#[doc = " # Returns"]
#[doc = " A `Result` containing the DXF file as a byte vector or an error if exporting fails."]
pub fn to_dxf<T: Triangulated3D>(shape: &T) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut drawing = Drawing::new();

    shape.visit_triangles(|tri| {
        #[allow(clippy::unnecessary_cast)]
        let face = dxf::entities::Face3D::new(
            dxf::Point::new(
                real_f64(&tri[0].position.x),
                real_f64(&tri[0].position.y),
                real_f64(&tri[0].position.z),
            ),
            dxf::Point::new(
                real_f64(&tri[1].position.x),
                real_f64(&tri[1].position.y),
                real_f64(&tri[1].position.z),
            ),
            dxf::Point::new(
                real_f64(&tri[2].position.x),
                real_f64(&tri[2].position.y),
                real_f64(&tri[2].position.z),
            ),
            // OBJ/CSG triangles have 3 distinct vertices; DXF Face3D needs 4, so repeat the last.
            dxf::Point::new(
                real_f64(&tri[2].position.x),
                real_f64(&tri[2].position.y),
                real_f64(&tri[2].position.z),
            ),
        );
        let entity = dxf::entities::Entity::new(dxf::entities::EntityType::Face3D(face));
        drawing.add_entity(entity);
    });

    let mut buffer = Vec::new();
    drawing.save(&mut buffer)?;
    Ok(buffer)
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    pub fn to_dxf(&self) -> Result<Vec<u8>, Box<dyn Error>> {
        self::to_dxf(self)
    }
}
