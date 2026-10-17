//! JavaScript wrapper around native [`hypermesh::TriangleMesh`].

use crate::solid::{self, SolidExt};
use crate::wasm::{
    context_js::{GeometryBoolResultJs, GeometryBoundsResultJs, GeometryQueryStatusJs},
    curve_js::GeometryMeshResultJs,
    geometry_context,
    io_error_js::io_error,
};
use crate::wasm::{
    matrix_js::Matrix4Js, plane_js::PlaneJs, point_js::Point3Js, real_from_js, real_to_js,
};
use hyperlattice::Point3;
use hypermesh::{Triangle, TriangleMesh};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use wasm_bindgen::prelude::*;

fn mesh_error(error: hypermesh::HypermeshError) -> JsValue {
    geometry_error(error.into())
}

pub(super) fn geometry_error(error: crate::errors::ValidationError) -> JsValue {
    geometry_exception(&error.to_string(), error.kind())
}

fn geometry_exception(message: &str, kind: crate::errors::ValidationErrorKind) -> JsValue {
    let exception = js_sys::Error::new(message);
    exception.set_name("GeometryError");
    Reflect::set(
        &exception,
        &JsValue::from_str("code"),
        &JsValue::from_str(kind.as_str()),
    )
    .expect("new Error object accepts a code property");
    exception.into()
}

pub(super) fn geometry_scalar(value: f64, name: &str) -> Result<crate::Real, JsValue> {
    real_from_js(value).ok_or_else(|| {
        geometry_exception(
            &format!("{name} must be finite"),
            crate::errors::ValidationErrorKind::InvalidInput,
        )
    })
}

fn geometry_count(value: f64, name: &str) -> Result<usize, JsValue> {
    if value.is_finite()
        && (0.0..=f64::from(u32::MAX)).contains(&value)
        && value.fract() == 0.0
    {
        Ok(value as usize)
    } else {
        Err(geometry_exception(
            &format!("{name} must be a finite integer between 0 and {}", u32::MAX),
            crate::errors::ValidationErrorKind::InvalidInput,
        ))
    }
}

fn face_normal(points: &[Point3; 3]) -> Option<hyperlattice::Vector3> {
    (&points[1] - &points[0])
        .unit_cross_checked(&(&points[2] - &points[0]))
        .ok()
}

/// Native triangle geometry exposed to JavaScript.
///
/// Native modeling failures throw ordinary `GeometryError` objects with a stable
/// `code` from `GeometryValidationError` and the native diagnostic in `message`.
/// Boolean, hull, membership and subdivision errors formerly thrown as strings
/// now use this same category-bearing error contract. Errors are owned by
/// JavaScript and remain usable after mesh handles are freed. Interchange methods
/// expose their format failures as `GeometryIoError`.
#[wasm_bindgen]
pub struct MeshJs {
    pub(crate) inner: TriangleMesh,
}

impl From<TriangleMesh> for MeshJs {
    fn from(inner: TriangleMesh) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl MeshJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        solid::empty().into()
    }

    /// Import finite coordinate triples and in-range index triples.
    ///
    /// Invalid buffers throw `GeometryError` with code `invalid_input` before
    /// constructing a native mesh. Topology is validated by consuming operations.
    #[wasm_bindgen(js_name = fromArrays)]
    pub fn from_arrays(positions: Vec<f64>, indices: Vec<u32>) -> Result<Self, JsValue> {
        if !positions.len().is_multiple_of(3) || !indices.len().is_multiple_of(3) {
            return Err(geometry_exception(
                "positions and indices must contain complete triples",
                crate::errors::ValidationErrorKind::InvalidInput,
            ));
        }
        let positions = positions
            .as_chunks::<3>()
            .0
            .iter()
            .map(|row| {
                Some(Point3::new(
                    real_from_js(row[0])?,
                    real_from_js(row[1])?,
                    real_from_js(row[2])?,
                ))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| {
                geometry_exception(
                    "positions must be finite",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;
        let triangles = indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|row| {
                let [a, b, c] = [
                    usize::try_from(row[0]).ok()?,
                    usize::try_from(row[1]).ok()?,
                    usize::try_from(row[2]).ok()?,
                ];
                (a < positions.len() && b < positions.len() && c < positions.len())
                    .then(|| Triangle::new(a, b, c))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| {
                geometry_exception(
                    "an index is outside the position array",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;
        Ok(TriangleMesh::new(positions, triangles).into())
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.triangles.is_empty()
    }

    #[wasm_bindgen(js_name = triangleCount)]
    pub fn triangle_count(&self) -> u32 {
        u32::try_from(self.inner.triangles.len()).unwrap_or(u32::MAX)
    }

    #[wasm_bindgen(js_name = vertexCount)]
    pub fn vertex_count(&self) -> u32 {
        u32::try_from(self.inner.positions.len()).unwrap_or(u32::MAX)
    }

    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self) -> Object {
        let positions = self
            .inner
            .positions
            .iter()
            .flat_map(|point| [&point.x, &point.y, &point.z].map(real_to_js))
            .collect::<Vec<_>>();
        let indices = self
            .inner
            .triangles
            .iter()
            .flat_map(|triangle| triangle.indices().map(|index| index as u32))
            .collect::<Vec<_>>();
        let mut normals = vec![0.0; positions.len()];
        let mut counts = vec![0_u32; self.inner.positions.len()];
        for triangle in self.inner.triangles.iter() {
            let [a, b, c] = triangle.indices();
            let points = [
                self.inner.positions[a].clone(),
                self.inner.positions[b].clone(),
                self.inner.positions[c].clone(),
            ];
            let Some(normal) = face_normal(&points) else {
                continue;
            };
            for index in [a, b, c] {
                normals[index * 3] += real_to_js(&normal.0[0]);
                normals[index * 3 + 1] += real_to_js(&normal.0[1]);
                normals[index * 3 + 2] += real_to_js(&normal.0[2]);
                counts[index] += 1;
            }
        }
        for (normal, count) in normals.as_chunks_mut::<3>().0.iter_mut().zip(counts) {
            if count != 0 {
                let length =
                    (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
                        .sqrt();
                if length > 0.0 {
                    normal.iter_mut().for_each(|component| *component /= length);
                }
            }
        }
        let object = Object::new();
        Reflect::set(
            &object,
            &"positions".into(),
            &Float64Array::from(positions.as_slice()),
        )
        .expect("plain object accepts positions");
        Reflect::set(
            &object,
            &"normals".into(),
            &Float64Array::from(normals.as_slice()),
        )
        .expect("plain object accepts normals");
        Reflect::set(
            &object,
            &"indices".into(),
            &Uint32Array::from(indices.as_slice()),
        )
        .expect("plain object accepts indices");
        object
    }

    pub fn positions(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"positions".into())
            .expect("positions exist")
            .unchecked_into()
    }

    pub fn normals(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"normals".into())
            .expect("normals exist")
            .unchecked_into()
    }

    pub fn indices(&self) -> Uint32Array {
        Reflect::get(&self.to_arrays(), &"indices".into())
            .expect("indices exist")
            .unchecked_into()
    }

    /// Strict interior of the regularized nonzero-winding solid. Internal
    /// source faces follow represented-set membership; its boundary is excluded.
    /// Canonical empty geometry returns false. Malformed closed-solid input
    /// throws GeometryError with code invalid_input, even
    /// when the point is outside its bounds or on a source triangle.
    #[wasm_bindgen(js_name = containsPoint)]
    pub fn contains_point(&self, point: &Point3Js) -> Result<bool, JsValue> {
        solid::contains_point(&self.inner, &point.inner).map_err(mesh_error)
    }

    pub fn union(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_union(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    pub fn difference(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_difference(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    pub fn intersection(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_intersection(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    pub fn xor(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_xor(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = unionWithContext)]
    pub fn union_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_union_with_context(&other.inner, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = differenceWithContext)]
    pub fn difference_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_difference_with_context(&other.inner, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = intersectionWithContext)]
    pub fn intersection_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_intersection_with_context(&other.inner, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = xorWithContext)]
    pub fn xor_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_xor_with_context(&other.inner, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = transformWithContext)]
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4Js,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_transform_with_context(&self.inner, &matrix.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = translateWithContext)]
    pub fn translate_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_transform_with_context(
            &self.inner,
            &hyperlattice::Matrix4::affine_translation([
                geometry_scalar(x, "x")?,
                geometry_scalar(y, "y")?,
                geometry_scalar(z, "z")?,
            ]),
            &context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = rotateWithContext)]
    pub fn rotate_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_rotate_with_context(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
            &context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = scaleWithContext)]
    pub fn scale_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_scale_with_context(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
            &context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = centerWithContext)]
    pub fn center_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_center_with_context(&self.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = floatOnZWithContext)]
    pub fn float_on_z_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_float_with_context(&self.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = mirrorWithContext)]
    pub fn mirror_with_context(
        &self,
        plane: &PlaneJs,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::try_mirror_with_context(&self.inner, &plane.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = convexHullWithContext)]
    pub fn convex_hull_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::convex_hull_with_context(&self.inner, &context)
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = minkowskiSumWithContext)]
    pub fn minkowski_sum_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::minkowski_sum_with_context(&self.inner, &other.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Represented-set sum with count limits; these are not time or heap-byte bounds.
    /// Counts must be finite integers from zero through `u32::MAX`.
    #[wasm_bindgen(js_name = minkowskiSumWithLimits)]
    pub fn minkowski_sum_with_limits(
        &self,
        other: &Self,
        approximate_512: bool,
        max_cells: f64,
        max_pair_sums: f64,
        max_pair_points: f64,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        let options = hypermesh::MinkowskiOptions {
            max_cells: geometry_count(max_cells, "maxCells")?,
            max_pair_sums: geometry_count(max_pair_sums, "maxPairSums")?,
            max_pair_points: geometry_count(max_pair_points, "maxPairPoints")?,
            cancellation: None,
        };
        solid::minkowski_sum_with_options(&self.inner, &other.inner, &options, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Convex hull of stored-position sums, including unused positions.
    #[wasm_bindgen(js_name = minkowskiHullWithContext)]
    pub fn minkowski_hull_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::minkowski_hull_with_context(&self.inner, &other.inner, &context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Containment with the certainty of both source validation and the query.
    /// Malformed input throws GeometryError; unresolved predicates keep Unknown.
    #[wasm_bindgen(js_name = containsPointWithContext)]
    pub fn contains_point_with_context(
        &self,
        point: &Point3Js,
        approximate_512: bool,
    ) -> Result<GeometryBoolResultJs, JsValue> {
        match solid::contains_point_with_context(
            &self.inner,
            &point.inner,
            &geometry_context(approximate_512),
        ) {
            Ok(outcome) => Ok(outcome.map(Some).into()),
            Err(
                error @ (hypermesh::HypermeshError::UnknownClassification
                | hypermesh::HypermeshError::PredicateUndecided { .. }),
            ) => Ok(GeometryBoolResultJs::unresolved(
                GeometryQueryStatusJs::Unknown,
                error.to_string(),
            )),
            Err(error) => Err(mesh_error(error)),
        }
    }

    #[wasm_bindgen(js_name = boundingBoxWithContext)]
    pub fn bounding_box_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryBoundsResultJs, JsValue> {
        solid::try_bounding_box_with_context(&self.inner, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(geometry_error)
    }

    pub fn transform(&self, matrix: &Matrix4Js) -> Result<Self, JsValue> {
        solid::try_transform(&self.inner, &matrix.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    pub fn translate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        Ok(self
            .inner
            .translated(
                geometry_scalar(x, "x")?,
                geometry_scalar(y, "y")?,
                geometry_scalar(z, "z")?,
            )
            .map_err(geometry_error)?
            .into())
    }

    pub fn rotate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::try_rotate(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    pub fn scale(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::try_scale(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    pub fn center(&self) -> Result<Self, JsValue> {
        solid::try_center(&self.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = floatOnZ)]
    pub fn float_on_z(&self) -> Result<Self, JsValue> {
        solid::try_float(&self.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    pub fn inverse(&self) -> Self {
        solid::inverse(&self.inner).into()
    }

    pub fn mirror(&self, plane: &PlaneJs) -> Result<Self, JsValue> {
        solid::try_mirror(&self.inner, &plane.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = convexHull)]
    pub fn convex_hull(&self) -> Result<Self, JsValue> {
        solid::convex_hull(&self.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    #[wasm_bindgen(js_name = minkowskiSum)]
    pub fn minkowski_sum(&self, other: &Self) -> Result<Self, JsValue> {
        solid::minkowski_sum(&self.inner, &other.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Strict convex hull of stored-position sums, explicitly convexifying gaps.
    #[wasm_bindgen(js_name = minkowskiHull)]
    pub fn minkowski_hull(&self, other: &Self) -> Result<Self, JsValue> {
        solid::minkowski_hull(&self.inner, &other.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> Result<JsValue, JsValue> {
        let Some(bounds) = solid::try_bounding_box(&self.inner).map_err(geometry_error)?
        else {
            return Ok(JsValue::UNDEFINED);
        };
        let object = Object::new();
        Reflect::set(
            &object,
            &"min".into(),
            &JsValue::from(Point3Js::from(bounds.mins.clone())),
        )
        .expect("plain object accepts minimum");
        Reflect::set(
            &object,
            &"max".into(),
            &JsValue::from(Point3Js::from(bounds.maxs.clone())),
        )
        .expect("plain object accepts maximum");
        Ok(object.into())
    }

    /// Import binary or ASCII STL through strict native closed-PWN validation.
    ///
    /// Canonical empty geometry is accepted. Coordinates parse as binary32 and
    /// are promoted exactly; validation certifies that parsed geometry, not the
    /// original exact construction or analytic source. Manifoldness is a separate
    /// claim. Failure throws `GeometryIoError`; native validation remains in its
    /// typed `cause`, and no mesh is returned. The byte buffer is not retained.
    #[wasm_bindgen(js_name = fromSTL)]
    pub fn from_stl(data: &[u8]) -> Result<Self, JsValue> {
        crate::io::stl::from_stl(data)
            .map(Into::into)
            .map_err(|error| io_error(error, "STL"))
    }

    /// Serialize binary32 STL, rejecting collapsed projected triangles.
    ///
    /// Failure throws an ordinary `GeometryIoError` object with a native `code`,
    /// `format`, and variant fields. This replaces the former string exception;
    /// inspect `message` for text and `code` for the machine-readable variant.
    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        crate::io::stl::to_stl_binary(&self.inner, "mesh")
            .map_err(|error| io_error(error, "STL"))
    }

    /// Serialize exact decimal spellings of the finite binary64 projection.
    ///
    /// Failure throws `GeometryIoError` with queryable native fields rather than
    /// a string. Reimport subsequently quantizes coordinates to binary32.
    #[wasm_bindgen(js_name = toSTLASCII)]
    pub fn to_stl_ascii(&self) -> Result<String, JsValue> {
        crate::io::stl::to_stl_ascii(&self.inner, "mesh")
            .map_err(|error| io_error(error, "STL"))
    }

    pub fn subdivide(&self, levels: u32) -> Result<Self, JsValue> {
        let levels = std::num::NonZeroU32::new(levels).ok_or_else(|| {
            geometry_exception(
                "levels must be positive",
                crate::errors::ValidationErrorKind::InvalidInput,
            )
        })?;
        solid::subdivide(&self.inner, levels)
            .map(Into::into)
            .map_err(mesh_error)
    }

    pub fn cube(size: f64) -> Result<Self, JsValue> {
        Ok(solid::cube(geometry_scalar(size, "size")?)
            .map_err(geometry_error)?
            .into())
    }

    pub fn cuboid(width: f64, length: f64, height: f64) -> Result<Self, JsValue> {
        Ok(solid::cuboid(
            geometry_scalar(width, "width")?,
            geometry_scalar(length, "length")?,
            geometry_scalar(height, "height")?,
        )
        .map_err(geometry_error)?
        .into())
    }

    pub fn sphere(radius: f64, segments: usize, stacks: usize) -> Result<Self, JsValue> {
        Ok(
            solid::sphere(geometry_scalar(radius, "radius")?, segments, stacks)
                .map_err(geometry_error)?
                .into(),
        )
    }

    pub fn cylinder(radius: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(solid::cylinder(
            geometry_scalar(radius, "radius")?,
            geometry_scalar(height, "height")?,
            segments,
        )
        .map_err(geometry_error)?
        .into())
    }

    pub fn frustum(
        bottom_radius: f64,
        top_radius: f64,
        height: f64,
        segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::frustum(
            geometry_scalar(bottom_radius, "bottomRadius")?,
            geometry_scalar(top_radius, "topRadius")?,
            geometry_scalar(height, "height")?,
            segments,
        )
        .map_err(geometry_error)?
        .into())
    }

    pub fn ellipsoid(
        rx: f64,
        ry: f64,
        rz: f64,
        segments: usize,
        stacks: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::ellipsoid(
            geometry_scalar(rx, "rx")?,
            geometry_scalar(ry, "ry")?,
            geometry_scalar(rz, "rz")?,
            segments,
            stacks,
        )
        .map_err(geometry_error)?
        .into())
    }

    pub fn torus(
        major_radius: f64,
        minor_radius: f64,
        major_segments: usize,
        minor_segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::torus(
            geometry_scalar(major_radius, "majorRadius")?,
            geometry_scalar(minor_radius, "minorRadius")?,
            major_segments,
            minor_segments,
        )
        .map_err(geometry_error)?
        .into())
    }
}

impl Default for MeshJs {
    fn default() -> Self {
        Self::new()
    }
}
