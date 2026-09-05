//! JavaScript wrapper for native Hypermesh planes.

use crate::context::GeometryDecisions;
use crate::wasm::{
    context_js::{GeometryMatrixResultJs, GeometryPlaneResultJs},
    geometry_context,
    point_js::Point3Js,
    point3_from_js, real_from_js_named, real_to_js,
    vector_js::Vector3Js,
};
use hyperlattice::Vector3;
use hypermesh::Plane;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    #[wasm_bindgen(js_name = FromComponents)]
    pub fn from_components(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> Result<Self, JsValue> {
        Self::from_components_with_context(ax, ay, az, bx, by, bz, cx, cy, cz, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    #[wasm_bindgen(js_name = FromComponentsWithContext)]
    pub fn from_components_with_context(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        Self::from_points_with_context(
            &point3_from_js(ax, ay, az)?.into(),
            &point3_from_js(bx, by, bz)?.into(),
            &point3_from_js(cx, cy, cz)?.into(),
            approximate_512,
        )
    }

    #[wasm_bindgen(js_name = FromPoints)]
    pub fn from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Result<Self, JsValue> {
        Self::from_points_with_context(a, b, c, false).map(GeometryPlaneResultJs::into_plane)
    }

    #[wasm_bindgen(js_name = FromPointsWithContext)]
    pub fn from_points_with_context(
        a: &Point3Js,
        b: &Point3Js,
        c: &Point3Js,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let decisions = GeometryDecisions::new(&geometry_context(approximate_512));
        let nondegenerate = decisions.consume_mesh(
            Plane::points_are_nondegenerate(
                decisions.mesh_context(),
                &a.inner,
                &b.inner,
                &c.inner,
            )
            .map_err(|error| JsValue::from_str(&error.to_string()))?,
        );
        if !nondegenerate {
            return Err(JsValue::from_str("plane points must be non-collinear"));
        }
        Ok(decisions
            .finish(Plane::from_points(&a.inner, &b.inner, &c.inner))
            .into())
    }

    #[wasm_bindgen(js_name = FromNormalComponents)]
    pub fn from_normal_components(
        nx: f64,
        ny: f64,
        nz: f64,
        offset: f64,
    ) -> Result<Self, JsValue> {
        Self::from_normal_components_with_context(nx, ny, nz, offset, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    #[wasm_bindgen(js_name = FromNormalComponentsWithContext)]
    pub fn from_normal_components_with_context(
        nx: f64,
        ny: f64,
        nz: f64,
        offset: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let normal = Vector3::from_xyz(
            real_from_js_named(nx, "nx")?,
            real_from_js_named(ny, "ny")?,
            real_from_js_named(nz, "nz")?,
        );
        Self::from_normal_with_context(&normal.into(), offset, approximate_512)
    }

    #[wasm_bindgen(js_name = FromNormal)]
    pub fn from_normal(normal: &Vector3Js, offset: f64) -> Result<Self, JsValue> {
        Self::from_normal_with_context(normal, offset, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    #[wasm_bindgen(js_name = FromNormalWithContext)]
    pub fn from_normal_with_context(
        normal: &Vector3Js,
        offset: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let decisions = GeometryDecisions::new(&geometry_context(approximate_512));
        let [x, y, z] = normal.inner.0.clone();
        validate_normal(&[x.clone(), y.clone(), z.clone()], &decisions)?;
        Ok(decisions
            .finish(Plane::from_coefficients(
                x,
                y,
                z,
                real_from_js_named(offset, "offset")?,
            ))
            .into())
    }

    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(Vector3::from_xyz(
            self.inner.normal.x.clone(),
            self.inner.normal.y.clone(),
            self.inner.normal.z.clone(),
        ))
    }

    pub fn offset(&self) -> f64 {
        real_to_js(&self.inner.offset)
    }

    pub fn flip(&mut self) {
        self.inner = self.inner.inverted();
    }

    #[wasm_bindgen(js_name = reflectionMatrix)]
    pub fn reflection_matrix(&self) -> Result<crate::wasm::matrix_js::Matrix4Js, JsValue> {
        self.reflection_matrix_with_context(false)
            .map(GeometryMatrixResultJs::into_matrix)
    }

    #[wasm_bindgen(js_name = reflectionMatrixWithContext)]
    pub fn reflection_matrix_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMatrixResultJs, JsValue> {
        let decisions = GeometryDecisions::new(&geometry_context(approximate_512));
        self.inner
            .reflection_matrix(decisions.mesh_context())
            .map(|outcome| decisions.finish(decisions.consume_mesh(outcome)).into())
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }
}

impl From<Plane> for PlaneJs {
    fn from(inner: Plane) -> Self {
        Self { inner }
    }
}

fn validate_normal(
    normal: &[hyperlattice::Real; 3],
    decisions: &GeometryDecisions,
) -> Result<(), JsValue> {
    let squared = normal.iter().fold(hyperlattice::Real::zero(), |sum, value| {
        sum + value.clone() * value.clone()
    });
    match decisions
        .decide(
            hyperlimit::classify_real_sign(&squared, decisions.predicate_policy()),
            "plane normal",
        )
        .map_err(|error| JsValue::from_str(&error.to_string()))?
    {
        hyperlimit::Sign::Positive => Ok(()),
        hyperlimit::Sign::Negative | hyperlimit::Sign::Zero => {
            Err(JsValue::from_str("plane normal must be non-zero"))
        },
    }
}
