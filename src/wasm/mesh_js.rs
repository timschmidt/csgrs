//! JavaScript wrapper for [`Mesh`].

use crate::csg::CSG;
use crate::hyper_math::Real;
use crate::mesh::{Mesh, plane::Plane};
use crate::wasm::{
    finite_matrix4, js_metadata, matrix_js::Matrix4Js, plane_js::PlaneJs, point_js::Point3Js,
    polygon_js::PolygonJs, sketch_js::SketchJs, vector_js::Vector3Js,
};
use crate::wasm::{point3_from_js_or_origin, real_from_js, real_from_js_or_zero, real_to_js};
use hyperlattice::{Point3, Vector3};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use serde_json::Value as JsonValue;
use serde_wasm_bindgen::from_value;
use wasm_bindgen::prelude::*;

fn wasm_projected_polygon_normal(points: &[Point3]) -> Option<Vector3> {
    (1..points.len().saturating_sub(1)).find_map(|i| {
        (points[i].clone() - points[0].clone())
            .cross(&(points[i + 1].clone() - points[0].clone()))
            .normalize_checked()
            .ok()
    })
}

fn wasm_point3_boundary(x: f64, y: f64, z: f64) -> Point3 {
    Point3Js::new(x, y, z).inner
}

#[wasm_bindgen]
pub struct MeshJs {
    pub(crate) inner: Mesh<Option<JsonValue>>,
}

#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl MeshJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: Mesh::empty(),
        }
    }

    #[wasm_bindgen(js_name=fromPolygons)]
    pub fn from_polygons(polygons: Vec<PolygonJs>) -> MeshJs {
        let poly_vec: Vec<_> = polygons.iter().map(|p| p.inner.clone()).collect();
        let mesh = Mesh::from_polygons(poly_vec);
        MeshJs { inner: mesh }
    }

    #[wasm_bindgen(js_name=fromPointsWithHoles)]
    pub fn from_points_with_holes(
        outer_points: Vec<f64>,
        hole_arrays: Vec<Float64Array>,
        metadata: JsValue,
    ) -> Result<MeshJs, JsValue> {
        use crate::vertex::Vertex;

        fn parse_points(data: &[f64], label: &str) -> Result<Vec<Point3>, JsValue> {
            if !data.len().is_multiple_of(3) {
                return Err(JsValue::from_str(&format!(
                    "{label} must contain x/y/z triples"
                )));
            }

            let mut points = Vec::with_capacity(data.len() / 3);
            for chunk in data.chunks_exact(3) {
                let point = point3_from_js_or_origin(chunk[0], chunk[1], chunk[2]);
                if point == Point3::origin()
                    && (chunk[0] != 0.0 || chunk[1] != 0.0 || chunk[2] != 0.0)
                {
                    return Err(JsValue::from_str(&format!(
                        "{label} contains a non-finite coordinate"
                    )));
                }
                points.push(point);
            }
            Ok(points)
        }

        fn push_ring(
            ring: &[Point3],
            origin: Point3,
            u: Vector3,
            v: Vector3,
            vertices: &mut Vec<Vertex>,
            flat_2d: &mut Vec<Real>,
            normal: Vector3,
        ) -> Result<(), JsValue> {
            for point in ring {
                let offset = point - &origin;
                flat_2d.push(offset.dot(&u));
                flat_2d.push(offset.dot(&v));
                vertices.push(Vertex::new(point.clone(), normal.clone()));
            }
            Ok(())
        }

        let outer = parse_points(&outer_points, "outer_points")?;
        if outer.len() < 3 {
            return Err(JsValue::from_str(
                "outer_points must contain at least three points",
            ));
        }

        let Some(normal) = wasm_projected_polygon_normal(&outer) else {
            return Err(JsValue::from_str("outer_points are degenerate"));
        };

        let origin = outer[0].clone();
        let Ok((u, v)) = normal.orthonormal_basis_checked() else {
            return Err(JsValue::from_str("outer_points are degenerate"));
        };
        let mut vertices = Vec::new();
        let mut flat_2d = Vec::new();
        let mut holes = Vec::with_capacity(hole_arrays.len());

        push_ring(
            &outer,
            origin.clone(),
            u.clone(),
            v.clone(),
            &mut vertices,
            &mut flat_2d,
            normal.clone(),
        )?;

        for (index, hole_array) in hole_arrays.into_iter().enumerate() {
            let hole = parse_points(&hole_array.to_vec(), &format!("hole_arrays[{index}]"))?;
            if hole.len() < 3 {
                return Err(JsValue::from_str(&format!(
                    "hole_arrays[{index}] must contain at least three points"
                )));
            }
            holes.push(vertices.len());
            push_ring(
                &hole,
                origin.clone(),
                u.clone(),
                v.clone(),
                &mut vertices,
                &mut flat_2d,
                normal.clone(),
            )?;
        }

        use crate::mesh::Polygon;

        let exact_points = flat_2d
            .chunks_exact(2)
            .map(|xy| Ok(hypertri::Point2::new(xy[0].clone(), xy[1].clone())))
            .collect::<Result<Vec<_>, JsValue>>()?;

        let indices = hypertri::earcut(&exact_points, &holes)
            .map_err(|err| JsValue::from_str(&format!("hypertri earcut failed: {err:?}")))?;
        let meta = js_metadata(metadata)?;
        let mut polygons = Vec::with_capacity(indices.len() / 3);

        for tri in indices.chunks_exact(3) {
            let i0 = tri[0];
            let i1 = tri[1];
            let i2 = tri[2];
            if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
                continue;
            }

            let a = vertices[i0].clone();
            let b = vertices[i1].clone();
            let c = vertices[i2].clone();
            if (b.position.clone() - a.position.clone())
                .cross(&(c.position.clone() - a.position.clone()))
                .normalize_checked()
                .is_err()
            {
                continue;
            }
            polygons.push(Polygon::new(vec![a, b, c], meta.clone()));
        }

        Ok(MeshJs {
            inner: Mesh::from_polygons(polygons).with_metadata(meta),
        })
    }

    /// Return an interleaved array of vertex positions (x,y,z)*.
    #[wasm_bindgen(js_name = positions)]
    pub fn positions(&self) -> Float64Array {
        let obj = self.to_arrays();
        let position = Reflect::get(&obj, &"positions".into()).unwrap();
        position.dyn_into::<Float64Array>().unwrap()
    }

    /// Return an interleaved array of vertex normals (nx,ny,nz)*.
    #[wasm_bindgen(js_name = normals)]
    pub fn normals(&self) -> Float64Array {
        let obj = self.to_arrays();
        let norms = Reflect::get(&obj, &"normals".into()).unwrap();
        norms.dyn_into::<Float64Array>().unwrap()
    }

    /// Return triangle indices (u32).
    #[wasm_bindgen(js_name = indices)]
    pub fn indices(&self) -> Uint32Array {
        let obj = self.to_arrays();
        let idx = Reflect::get(&obj, &"indices".into()).unwrap();
        idx.dyn_into::<Uint32Array>().unwrap()
    }

    pub fn polygons(&self) -> Vec<PolygonJs> {
        self.inner
            .polygons
            .iter()
            .cloned()
            .map(|inner| PolygonJs { inner })
            .collect()
    }

    /// Number of triangles (handy to sanity-check).
    #[wasm_bindgen(js_name = triangleCount)]
    pub fn triangle_count(&self) -> u32 {
        self.inner.triangulate().polygons.len() as u32
    }

    #[wasm_bindgen(js_name = vertexCount)]
    pub fn vertex_count(&self) -> u32 {
        let mut vertices = Vec::new();
        for poly in &self.inner.polygons {
            for vertex in &poly.vertices {
                if !vertices.iter().any(|v: &Point3| v == &vertex.position) {
                    vertices.push(vertex.position.clone());
                }
            }
        }
        vertices.len() as u32
    }

    /// Convert a mesh to arrays of positions, normals, and indices
    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self) -> js_sys::Object {
        let tri = &self.inner.triangulate();

        let tri_count = tri.polygons.len();
        let mut positions = Vec::with_capacity(tri_count * 3 * 3);
        let mut normals = Vec::with_capacity(tri_count * 3 * 3);
        let mut indices = Vec::with_capacity(tri_count * 3);

        let mut idx: u32 = 0;
        for p in &tri.polygons {
            for v in &p.vertices {
                positions.extend_from_slice(&[
                    real_to_js(&v.position.x),
                    real_to_js(&v.position.y),
                    real_to_js(&v.position.z),
                ]);
                normals.extend_from_slice(&[
                    real_to_js(&v.normal.0[0]),
                    real_to_js(&v.normal.0[1]),
                    real_to_js(&v.normal.0[2]),
                ]);
            }
            indices.extend_from_slice(&[idx, idx + 1, idx + 2]);
            idx += 3;
        }

        let obj = Object::new();
        Reflect::set(
            &obj,
            &"positions".into(),
            &Float64Array::from(positions.as_slice()).into(),
        )
        .unwrap();
        Reflect::set(
            &obj,
            &"normals".into(),
            &Float64Array::from(normals.as_slice()).into(),
        )
        .unwrap();
        Reflect::set(
            &obj,
            &"indices".into(),
            &Uint32Array::from(indices.as_slice()).into(),
        )
        .unwrap();

        obj
    }

    #[wasm_bindgen(js_name=vertices)]
    pub fn vertices(&self) -> JsValue {
        let verts = self.inner.vertices();
        let js_array = js_sys::Array::new();

        for v in verts {
            let js_vert = Object::new();

            // Wrap position and normal in Point3Js / Vector3Js
            let pos_js = Point3Js::from(v.position);
            let norm_js = Vector3Js::from(v.normal);

            Reflect::set(&js_vert, &"position".into(), &JsValue::from(pos_js)).unwrap();
            Reflect::set(&js_vert, &"normal".into(), &JsValue::from(norm_js)).unwrap();

            js_array.push(&js_vert);
        }

        js_array.into()
    }

    #[wasm_bindgen(js_name = containsVertex)]
    pub fn contains_vertex(&self, p: &Point3Js) -> bool {
        let point: Point3 = p.into();
        self.inner.contains_vertex(&point)
    }

    #[wasm_bindgen(js_name = containsVertexComponents)]
    pub fn contains_vertex_components(&self, x: f64, y: f64, z: f64) -> bool {
        let point = Point3Js::new(x, y, z);
        self.inner.contains_vertex(&point.inner)
    }

    // Boolean Operations
    #[wasm_bindgen(js_name = union)]
    pub fn union(&self, other: &MeshJs) -> Result<Self, JsValue> {
        self.inner
            .try_union(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = difference)]
    pub fn difference(&self, other: &MeshJs) -> Result<Self, JsValue> {
        self.inner
            .try_difference(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = intersection)]
    pub fn intersection(&self, other: &MeshJs) -> Result<Self, JsValue> {
        self.inner
            .try_intersection(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = xor)]
    pub fn xor(&self, other: &MeshJs) -> Result<Self, JsValue> {
        self.inner
            .try_xor(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    // Transformations
    #[wasm_bindgen(js_name = transform)]
    pub fn transform(&self, mat: &Matrix4Js) -> Self {
        Self {
            inner: self.inner.transform(&mat.inner),
        }
    }

    #[wasm_bindgen(js_name = transformComponents)]
    pub fn transform_components(
        &self,
        m00: f64,
        m01: f64,
        m02: f64,
        m03: f64,
        m10: f64,
        m11: f64,
        m12: f64,
        m13: f64,
        m20: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m30: f64,
        m31: f64,
        m32: f64,
        m33: f64,
    ) -> Self {
        let raw = [
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        ];
        let mut values: [Real; 16] = std::array::from_fn(|_| Real::zero());
        for (slot, value) in values.iter_mut().zip(raw) {
            let Some(value) = real_from_js(value) else {
                return Self {
                    inner: self.inner.clone(),
                };
            };
            *slot = value;
        }
        let Some(matrix) = finite_matrix4(values) else {
            return Self {
                inner: self.inner.clone(),
            };
        };
        Self {
            inner: self.inner.transform(&matrix),
        }
    }

    #[wasm_bindgen(js_name = translate)]
    pub fn translate(&self, offset: &Vector3Js) -> Self {
        let v: Vector3 = offset.into();
        Self {
            inner: self
                .inner
                .translate(v.0[0].clone(), v.0[1].clone(), v.0[2].clone()),
        }
    }

    #[wasm_bindgen(js_name = translateComponents)]
    pub fn translate_components(&self, dx: f64, dy: f64, dz: f64) -> Self {
        Self {
            inner: self.inner.translate(
                real_from_js_or_zero(dx),
                real_from_js_or_zero(dy),
                real_from_js_or_zero(dz),
            ),
        }
    }

    #[wasm_bindgen(js_name = rotate)]
    pub fn rotate(&self, rx: f64, ry: f64, rz: f64) -> Self {
        Self {
            inner: self.inner.rotate(
                real_from_js_or_zero(rx),
                real_from_js_or_zero(ry),
                real_from_js_or_zero(rz),
            ),
        }
    }

    #[wasm_bindgen(js_name = rotateQuaternion)]
    pub fn rotate_quaternion(&self, _w: f64, _x: f64, _y: f64, _z: f64) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }

    #[wasm_bindgen(js_name = scale)]
    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> Self {
        Self {
            inner: self.inner.scale(
                real_from_js_or_zero(sx),
                real_from_js_or_zero(sy),
                real_from_js_or_zero(sz),
            ),
        }
    }

    #[wasm_bindgen(js_name = center)]
    pub fn center(&self) -> Self {
        Self {
            inner: self.inner.center(),
        }
    }

    pub fn mirror(&self, plane: &PlaneJs) -> Self {
        Self {
            inner: self.inner.mirror(plane.inner.clone()),
        }
    }

    #[wasm_bindgen(js_name = float)]
    pub fn float(&self) -> Self {
        Self {
            inner: self.inner.float(),
        }
    }

    #[wasm_bindgen(js_name = inverse)]
    pub fn inverse(&self) -> Self {
        Self {
            inner: self.inner.inverse(),
        }
    }

    pub fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }

    #[wasm_bindgen(js_name=convexHull)]
    pub fn convex_hull(&self, metadata: JsValue) -> Result<Self, JsValue> {
        let metadata = js_metadata(metadata)?;
        Ok(Self {
            inner: self.inner.convex_hull(metadata),
        })
    }

    #[wasm_bindgen(js_name=minkowskiSum)]
    pub fn minkowski_sum(&self, other: &MeshJs, metadata: JsValue) -> Result<Self, JsValue> {
        let metadata = js_metadata(metadata)?;
        Ok(Self {
            inner: self.inner.minkowski_sum(&other.inner, metadata),
        })
    }

    #[wasm_bindgen(js_name=flatten)]
    pub fn flatten(&self) -> SketchJs {
        let sketch = self.inner.flatten();
        SketchJs { inner: sketch }
    }

    #[wasm_bindgen(js_name = slice)]
    pub fn slice(&self, plane: &PlaneJs) -> SketchJs {
        let sketch = self.inner.slice(plane.into());
        SketchJs { inner: sketch }
    }

    #[wasm_bindgen(js_name=sliceComponents)]
    pub fn slice_components(
        &self,
        normal_x: f64,
        normal_y: f64,
        normal_z: f64,
        offset: f64,
    ) -> SketchJs {
        let plane = PlaneJs::from_normal_components(normal_x, normal_y, normal_z, offset);
        let sketch = self.inner.slice(plane.inner);
        SketchJs { inner: sketch }
    }

    #[wasm_bindgen(js_name = splitByPlane)]
    pub fn split_by_plane(&self, plane: &PlaneJs) -> Vec<MeshJs> {
        let plane: Plane = plane.into();
        let mut front_polys = Vec::new();
        let mut back_polys = Vec::new();

        for polygon in &self.inner.polygons {
            let (coplanar_front, coplanar_back, mut front, mut back) =
                plane.split_polygon(polygon);
            front_polys.extend(coplanar_front);
            back_polys.extend(coplanar_back);
            front_polys.append(&mut front);
            back_polys.append(&mut back);
        }

        vec![
            MeshJs {
                inner: Mesh::from_polygons(front_polys),
            },
            MeshJs {
                inner: Mesh::from_polygons(back_polys),
            },
        ]
    }

    #[wasm_bindgen(js_name = intersectPolyline)]
    pub fn intersect_polyline_js(&self, points: Vec<Point3Js>) -> Vec<Point3Js> {
        let polyline: Vec<Point3> = points.into_iter().map(|point| point.inner).collect();
        self.inner
            .intersect_polyline(&polyline)
            .into_iter()
            .map(Point3Js::from)
            .collect()
    }

    #[wasm_bindgen(js_name=laplacianSmooth)]
    pub fn laplacian_smooth(
        &self,
        lambda: f64,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Self {
        let smoothed = self.inner.laplacian_smooth(
            real_from_js_or_zero(lambda),
            iterations,
            preserve_boundaries,
        );
        Self { inner: smoothed }
    }

    #[wasm_bindgen(js_name=taubinSmooth)]
    pub fn taubin_smooth(
        &self,
        lambda: f64,
        mu: f64,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Self {
        let smoothed = self.inner.taubin_smooth(
            real_from_js_or_zero(lambda),
            real_from_js_or_zero(mu),
            iterations,
            preserve_boundaries,
        );
        Self { inner: smoothed }
    }

    // Distribute functions
    #[wasm_bindgen(js_name = distributeLinear)]
    pub fn distribute_linear(
        &self,
        count: usize,
        direction: &Vector3Js,
        spacing: f64,
    ) -> Self {
        let dir: Vector3 = direction.into();
        Self {
            inner: self
                .inner
                .distribute_linear(count, dir, real_from_js_or_zero(spacing)),
        }
    }

    #[wasm_bindgen(js_name = distributeLinearComponents)]
    pub fn distribute_linear_components(
        &self,
        count: usize,
        dx: f64,
        dy: f64,
        dz: f64,
        spacing: f64,
    ) -> Self {
        let direction = Vector3Js::new(dx, dy, dz);
        Self {
            inner: self.inner.distribute_linear(
                count,
                direction.inner,
                real_from_js_or_zero(spacing),
            ),
        }
    }

    #[wasm_bindgen(js_name=distributeArc)]
    pub fn distribute_arc(
        &self,
        count: usize,
        radius: f64,
        start_angle: f64,
        end_angle: f64,
    ) -> Self {
        Self {
            inner: self.inner.distribute_arc(
                count,
                real_from_js_or_zero(radius),
                real_from_js_or_zero(start_angle),
                real_from_js_or_zero(end_angle),
            ),
        }
    }

    #[wasm_bindgen(js_name=distributeGrid)]
    pub fn distribute_grid(
        &self,
        rows: usize,
        cols: usize,
        row_spacing: f64,
        col_spacing: f64,
    ) -> Self {
        Self {
            inner: self.inner.distribute_grid(
                rows,
                cols,
                real_from_js_or_zero(row_spacing),
                real_from_js_or_zero(col_spacing),
            ),
        }
    }

    // Bounding Box
    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> JsValue {
        let bb = self.inner.bounding_box();
        let min_js = Point3Js::from(bb.mins);
        let max_js = Point3Js::from(bb.maxs);

        let obj = Object::new();
        Reflect::set(&obj, &"min".into(), &JsValue::from(min_js)).unwrap();
        Reflect::set(&obj, &"max".into(), &JsValue::from(max_js)).unwrap();
        obj.into()
    }

    #[wasm_bindgen(js_name=invalidateBoundingBox)]
    pub fn invalidate_bounding_box(&mut self) {
        self.inner.invalidate_bounding_box();
    }

    // IO Operations
    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        self.inner
            .to_stl_binary("mesh")
            .map_err(|e| JsValue::from_str(&format!("STL export failed: {:?}", e)))
    }

    #[wasm_bindgen(js_name = toSTLASCII)]
    pub fn to_stl_ascii(&self) -> Result<String, JsValue> {
        self.inner
            .to_stl_ascii("mesh")
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = toAMF)]
    pub fn to_amf(&self, object_name: &str, units: &str) -> Result<String, JsValue> {
        self.inner
            .to_amf(object_name, units)
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = toAMFWithColor)]
    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        r: f64,
        g: f64,
        b: f64,
    ) -> Result<String, JsValue> {
        self.inner
            .to_amf_with_color(
                object_name,
                units,
                (
                    real_from_js_or_zero(r),
                    real_from_js_or_zero(g),
                    real_from_js_or_zero(b),
                ),
            )
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = toGLTF)]
    pub fn to_gltf(&self, object_name: &str) -> Result<String, JsValue> {
        self.inner
            .to_gltf(object_name)
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name=fromSketch)]
    pub fn from_sketch(sketch_js: &SketchJs, metadata: JsValue) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        let mesh = Mesh::from_profile(sketch_js.inner.clone(), metadata);
        Ok(Self { inner: mesh })
    }

    // Metadata
    #[wasm_bindgen(js_name=filterPolygonsByMetadata)]
    pub fn filter_polygons_by_metadata(&self, needle: JsValue) -> Result<MeshJs, JsValue> {
        let meta = js_metadata(needle)?;
        let mesh = self.inner.filter_polygons_by_metadata(&meta);
        Ok(MeshJs { inner: mesh })
    }

    // Mass Properties
    #[wasm_bindgen(js_name = massProperties)]
    pub fn mass_properties(&self, density: f64) -> JsValue {
        let Ok((mass, com, _frame)) =
            self.inner.mass_properties(real_from_js_or_zero(density))
        else {
            return JsValue::NULL;
        };
        let obj = Object::new();

        let com_js = Point3Js::from(com);

        Reflect::set(&obj, &"mass".into(), &real_to_js(&mass).into()).unwrap();
        Reflect::set(&obj, &"centerOfMass".into(), &JsValue::from(com_js)).unwrap();
        obj.into()
    }

    // Subdivision
    #[wasm_bindgen(js_name = subdivideTriangles)]
    pub fn subdivide_triangles(&self, levels: u32) -> Self {
        if levels == 0 {
            return Self {
                inner: self.inner.clone(),
            };
        }
        let levels_nonzero = std::num::NonZeroU32::new(levels).unwrap();
        Self {
            inner: self.inner.subdivide_triangles(levels_nonzero),
        }
    }

    #[wasm_bindgen(js_name = renormalize)]
    pub fn renormalize(&self) -> Self {
        let mut inner = self.inner.clone();
        inner.renormalize();
        Self { inner }
    }

    #[wasm_bindgen(js_name = triangulate)]
    pub fn triangulate(&self) -> Self {
        Self {
            inner: self.inner.triangulate(),
        }
    }

    // 3D Shapes
    #[wasm_bindgen(js_name = cube)]
    pub fn cube(size: f64, metadata: JsValue) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::cube(real_from_js_or_zero(size), meta),
        })
    }

    #[wasm_bindgen(js_name = sphere)]
    pub fn sphere(
        radius: f64,
        segments_u: usize,
        segments_v: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::sphere(real_from_js_or_zero(radius), segments_u, segments_v, meta),
        })
    }

    #[wasm_bindgen(js_name = cylinder)]
    pub fn cylinder(
        radius: f64,
        height: f64,
        segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::cylinder(
                real_from_js_or_zero(radius),
                real_from_js_or_zero(height),
                segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = cuboid)]
    pub fn cuboid(
        width: f64,
        length: f64,
        height: f64,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::cuboid(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                real_from_js_or_zero(height),
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = frustum_ptp)]
    pub fn frustum_ptp(
        start: &Point3Js,
        end: &Point3Js,
        radius1: f64,
        radius2: f64,
        segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let start: Point3 = start.into();
        let end: Point3 = end.into();
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::frustum_ptp(
                start,
                end,
                real_from_js_or_zero(radius1),
                real_from_js_or_zero(radius2),
                segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = frustum_ptpComponents)]
    pub fn frustum_ptp_components(
        start_x: f64,
        start_y: f64,
        start_z: f64,
        end_x: f64,
        end_y: f64,
        end_z: f64,
        radius1: f64,
        radius2: f64,
        segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let start: Point3 = wasm_point3_boundary(start_x, start_y, start_z);
        let end: Point3 = wasm_point3_boundary(end_x, end_y, end_z);
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::frustum_ptp(
                start,
                end,
                real_from_js_or_zero(radius1),
                real_from_js_or_zero(radius2),
                segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = frustum)]
    pub fn frustum(
        radius1: f64,
        radius2: f64,
        height: f64,
        segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::frustum(
                real_from_js_or_zero(radius1),
                real_from_js_or_zero(radius2),
                real_from_js_or_zero(height),
                segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = polyhedron)]
    pub fn polyhedron(
        points: JsValue,
        faces: JsValue,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let points_vec: Vec<[f64; 3]> = from_value(points)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse points: {:?}", e)))?;
        let faces_vec: Vec<Vec<usize>> = from_value(faces)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse faces: {:?}", e)))?;

        let points_3d: Vec<[Real; 3]> = points_vec
            .into_iter()
            .enumerate()
            .map(|(index, [x, y, z])| {
                let point = match (real_from_js(x), real_from_js(y), real_from_js(z)) {
                    (Some(x), Some(y), Some(z)) => [x, y, z],
                    _ => {
                        return Err(JsValue::from_str(&format!(
                            "Polyhedron point {index} contains non-finite coordinates"
                        )));
                    },
                };
                Ok(point)
            })
            .collect::<Result<_, _>>()?;

        let faces_ref: Vec<&[usize]> = faces_vec.iter().map(|f| f.as_slice()).collect();

        let meta = js_metadata(metadata)?;

        let mesh = Mesh::polyhedron(&points_3d, &faces_ref, meta)
            .map_err(|e| JsValue::from_str(&format!("Polyhedron creation failed: {:?}", e)))?;

        Ok(Self { inner: mesh })
    }

    #[wasm_bindgen(js_name = egg)]
    pub fn egg(
        width: f64,
        length: f64,
        revolve_segments: usize,
        outline_segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::egg(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                revolve_segments,
                outline_segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = teardrop)]
    pub fn teardrop(
        width: f64,
        length: f64,
        revolve_segments: usize,
        shape_segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::teardrop(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                revolve_segments,
                shape_segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = teardropCylinder)]
    pub fn teardrop_cylinder(
        width: f64,
        length: f64,
        height: f64,
        shape_segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::teardrop_cylinder(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                real_from_js_or_zero(height),
                shape_segments,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = ellipsoid)]
    pub fn ellipsoid(
        rx: f64,
        ry: f64,
        rz: f64,
        segments: usize,
        stacks: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::ellipsoid(
                real_from_js_or_zero(rx),
                real_from_js_or_zero(ry),
                real_from_js_or_zero(rz),
                segments,
                stacks,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = arrow)]
    pub fn arrow(
        start: &Point3Js,
        direction: &Vector3Js,
        segments: usize,
        orientation: bool,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let start: Point3 = start.into();
        let dir: Vector3 = direction.into();
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::arrow(start, dir, segments, orientation, meta),
        })
    }

    #[wasm_bindgen(js_name = octahedron)]
    pub fn octahedron(radius: f64, metadata: JsValue) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::octahedron(real_from_js_or_zero(radius), meta),
        })
    }

    #[wasm_bindgen(js_name = icosahedron)]
    pub fn icosahedron(radius: f64, metadata: JsValue) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::icosahedron(real_from_js_or_zero(radius), meta),
        })
    }

    #[wasm_bindgen(js_name = torus)]
    pub fn torus(
        major_r: f64,
        minor_r: f64,
        segments_major: usize,
        segments_minor: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::torus(
                real_from_js_or_zero(major_r),
                real_from_js_or_zero(minor_r),
                segments_major,
                segments_minor,
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name = spurGearInvolute)]
    pub fn spur_gear_involute(
        module_: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        clearance: f64,
        backlash: f64,
        segments_per_flank: usize,
        thickness: f64,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        Ok(Self {
            inner: Mesh::spur_gear_involute(
                real_from_js_or_zero(module_),
                teeth,
                real_from_js_or_zero(pressure_angle_deg),
                real_from_js_or_zero(clearance),
                real_from_js_or_zero(backlash),
                segments_per_flank,
                real_from_js_or_zero(thickness),
                meta,
            ),
        })
    }

    #[wasm_bindgen(js_name=gyroid)]
    pub fn gyroid(
        &self,
        resolution: u32,
        scale: f64,
        iso_value: f64,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        let gyroid_mesh = self.inner.gyroid(
            resolution.try_into().unwrap(),
            real_from_js_or_zero(scale),
            real_from_js_or_zero(iso_value),
            meta,
        );
        Ok(Self { inner: gyroid_mesh })
    }

    #[wasm_bindgen(js_name=schwarzP)]
    pub fn schwarz_p(
        &self,
        resolution: u32,
        scale: f64,
        iso_value: f64,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        let schwarzp_mesh = self.inner.schwarz_p(
            resolution.try_into().unwrap(),
            real_from_js_or_zero(scale),
            real_from_js_or_zero(iso_value),
            meta,
        );
        Ok(Self {
            inner: schwarzp_mesh,
        })
    }

    #[wasm_bindgen(js_name=schwarzD)]
    pub fn schwarz_d(
        &self,
        resolution: u32,
        scale: f64,
        iso_value: f64,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let meta = js_metadata(metadata)?;
        let schwarzd_mesh = self.inner.schwarz_d(
            resolution.try_into().unwrap(),
            real_from_js_or_zero(scale),
            real_from_js_or_zero(iso_value),
            meta,
        );
        Ok(Self {
            inner: schwarzd_mesh,
        })
    }

    // #[wasm_bindgen(js_name=metaballs)]
    // pub fn metaballs(balls: JsValue, resolution_x: u32, resolution_y: u32, resolution_z: u32, iso_value: f64, padding: f64) -> Result<Self, JsValue> {
    // Parse the list of MetaBallJs objects or raw data.
    // let balls_vec: Vec<MetaBallJs> = from_value(balls).unwrap_or_else(|_| vec![]);
    // let meta_balls: Vec<MetaBall> = balls_vec.into_iter().map(|b| b.inner).collect();
    //
    // let resolution = (resolution_x.try_into().unwrap(), resolution_y.try_into().unwrap(), resolution_z.try_into().unwrap());
    // let metaball_mesh = Mesh::metaballs(&meta_balls, resolution, iso_value, padding, None);
    // Self { inner: metaball_mesh }
    // }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wasm::tolerance;

    fn r(value: f64) -> Real {
        real_from_js(value).expect("finite wasm test scalar")
    }

    #[test]
    fn mesh_js_transform_components_rejects_nonfinite_matrix() {
        let mesh = MeshJs {
            inner: Mesh::cube(Real::one(), None),
        };
        let original = mesh.inner.bounding_box();

        let transformed = mesh.transform_components(
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            f64::NAN,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );

        assert_eq!(transformed.inner.bounding_box(), original);
    }

    #[test]
    fn mesh_js_component_vectors_reuse_hyper_boundary_wrappers() {
        let mesh = MeshJs {
            inner: Mesh::cube(Real::one(), None),
        };
        let original = mesh.inner.bounding_box();

        assert!(!mesh.contains_vertex_components(f64::NAN, 0.0, 0.0));
        let distributed =
            mesh.distribute_linear_components(4, f64::NAN, f64::INFINITY, 0.0, 1.0);
        assert_eq!(distributed.inner.bounding_box(), original);

        let sliced = mesh.slice_components(f64::NAN, f64::INFINITY, 0.0, f64::NAN);
        let bbox = sliced.inner.bounding_box();
        assert!(bbox.mins.x.is_finite());
        assert!(bbox.maxs.x.is_finite());
    }

    #[test]
    fn mesh_js_frustum_ptp_components_reuse_hyperreal_point_boundary() {
        let start = wasm_point3_boundary(f64::NAN, 0.0, f64::INFINITY);
        let end = wasm_point3_boundary(0.0, 0.0, 1.0);
        let mesh = Mesh::<()>::frustum_ptp(start, end, r(0.25), r(0.5), 8, ());
        for vertex in mesh.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
        }
    }

    #[test]
    #[cfg(target_arch = "wasm32")]
    fn mesh_js_frustum_ptp_components_exported_path_reuses_hyperreal_point_boundary() {
        let mesh = MeshJs::frustum_ptp_components(
            f64::NAN,
            0.0,
            f64::INFINITY,
            0.0,
            0.0,
            1.0,
            0.25,
            0.5,
            8,
            JsValue::NULL,
        );
        for vertex in mesh.inner.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
        }
    }

    #[test]
    fn mesh_js_from_points_projection_uses_hyperlattice_normal_checks() {
        let points = [
            point3_from_js_or_origin(0.0, 0.0, 0.0),
            Point3::new(tolerance() * 0.25, Real::zero(), Real::zero()),
            Point3::new(tolerance() * 0.5, Real::zero(), Real::zero()),
            point3_from_js_or_origin(0.0, 1.0, 0.0),
        ];

        let normal = wasm_projected_polygon_normal(&points).unwrap();
        assert!(normal.0.iter().all(|value| value.is_finite()));
        assert!(normal.dot(&Vector3::z()) > 1.0 - tolerance());

        let degenerate = [
            point3_from_js_or_origin(0.0, 0.0, 0.0),
            Point3::new(tolerance() * 0.25, Real::zero(), Real::zero()),
            Point3::new(tolerance() * 0.5, Real::zero(), Real::zero()),
        ];
        assert!(wasm_projected_polygon_normal(&degenerate).is_none());

        let hostile = [
            point3_from_js_or_origin(0.0, 0.0, 0.0),
            point3_from_js_or_origin(f64::NAN, 0.0, 0.0),
            point3_from_js_or_origin(0.0, 1.0, 0.0),
        ];
        assert!(wasm_projected_polygon_normal(&hostile).is_none());
    }
}
