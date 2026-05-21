//! Create `Mesh`s by meshing signed distance fields ([sdf](https://en.wikipedia.org/wiki/Signed_distance_function)) within a bounding box.

use crate::float_types::{
    F32, HReal, Real, hreal_from_f32, hreal_from_f64, hreal_sign, hreal_to_f64,
    htriangle_area2_exceeds_epsilon, hvector3_from_point3, hvector3_from_vector3,
};
use crate::mesh::Mesh;
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use fast_surface_nets::{SurfaceNetsBuffer, surface_nets};
use hyperlimit::Point3 as HPoint3;
use hyperreal::RealSign;
use hypersdf::{
    PreparedSdf, SdfExpr, SdfMeshPreviewReport, SdfPreviewGrid, SdfSamplingPrecision,
};
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

/// Diagnostics captured while sampling and meshing an SDF.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SdfDiagnostics {
    pub resolution: (u32, u32, u32),
    pub sample_count: usize,
    pub finite_sample_count: usize,
    pub non_finite_sample_count: usize,
    pub negative_sample_count: usize,
    pub zero_sample_count: usize,
    pub positive_sample_count: usize,
    pub min_finite_value: Option<Real>,
    pub max_finite_value: Option<Real>,
    pub crossing_cell_count: usize,
    pub surface_nets_vertex_count: usize,
    pub surface_nets_index_count: usize,
    pub emitted_triangle_count: usize,
    pub skipped_non_finite_triangle_count: usize,
    pub degenerate_triangle_count: usize,
    pub hypersdf_preview: Option<SdfMeshPreviewReport>,
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Return a Mesh created by meshing a signed distance field within a bounding box
    ///
    /// ```
    /// # use csgrs::{mesh::Mesh, float_types::Real};
    /// # use nalgebra::Point3;
    /// // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
    /// let my_sdf = |p: &Point3<Real>| (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() - 1.5;
    ///
    /// let resolution = (60, 60, 60);
    /// let min_pt = Point3::new(-2.0, -2.0, -2.0);
    /// let max_pt = Point3::new( 2.0,  2.0,  2.0);
    /// let iso_value = 0.0; // Typically zero for SDF-based surfaces
    ///
    ///    let mesh_shape = Mesh::<()>::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, ());
    ///
    ///    // Now `mesh_shape` is your polygon mesh as a Mesh you can union, subtract, or export:
    ///    let _ = std::fs::write("stl/sdf_sphere.stl", mesh_shape.to_stl_binary("sdf_sphere").unwrap());
    pub fn sdf<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M>
    where
        // F is a closure or function that takes a 3D point and returns the signed distance.
        // Must be `Sync`/`Send` if you want to parallelize the sampling.
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        Self::sdf_with_diagnostics(sdf, resolution, min_pt, max_pt, iso_value, metadata).0
    }

    /// Return a mesh preview generated from a retained [`hypersdf::SdfExpr`].
    ///
    /// This is the preferred SDF entry point while csgrs is being reduced to a
    /// CAD composition layer over Hyper geometry crates. The continuous field,
    /// metric facts, exact sampling grid, and preview-only topology report are
    /// owned by `hypersdf`; csgrs only lowers the reported scalar samples into
    /// its transitional Surface Nets mesh carrier.
    pub fn sdf_expr(
        expr: SdfExpr,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> Mesh<M> {
        Self::sdf_expr_with_diagnostics(expr, resolution, min_pt, max_pt, iso_value, metadata)
            .0
    }

    /// Return a mesh preview and diagnostics generated from a retained
    /// [`hypersdf::SdfExpr`].
    pub fn sdf_expr_with_diagnostics(
        expr: SdfExpr,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> (Mesh<M>, SdfDiagnostics) {
        let prepared = hypersdf::prepare(expr);
        Self::prepared_sdf_with_diagnostics(
            &prepared, resolution, min_pt, max_pt, iso_value, metadata,
        )
    }

    /// Return a mesh preview and diagnostics generated from a prepared
    /// [`hypersdf::PreparedSdf`].
    ///
    /// `hypersdf` keeps continuous-field ownership and reports that sampled
    /// scalars are preview-only. csgrs consumes that report to compose a mesh
    /// preview, in the same separation recommended by Yap's exact geometric
    /// computation model (<https://doi.org/10.1016/0925-7721(95)00040-2>) and
    /// by Gibson's Surface Nets preview meshing work, "Constrained Elastic
    /// Surface Nets," MERL TR99-24, 1999.
    pub fn prepared_sdf_with_diagnostics(
        prepared: &PreparedSdf,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> (Mesh<M>, SdfDiagnostics) {
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
        let mut diagnostics = SdfDiagnostics {
            resolution: (nx, ny, nz),
            sample_count: (nx * ny * nz) as usize,
            ..SdfDiagnostics::default()
        };

        let Some(grid) = hypersdf_grid(min_pt, max_pt, nx, ny, nz) else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        };

        let Ok(iso_value) = hreal_from_f64(iso_value) else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        };

        let Ok(grid_report) = prepared.sample_grid_preview(grid, SdfSamplingPrecision::F64)
        else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        };

        diagnostics.hypersdf_preview = Some(SdfMeshPreviewReport::surface_nets_diagnostic(
            grid_report.clone(),
        ));
        let mut field_values =
            SdfSampleField::with_capacity(grid_report.samples.samples.len());
        for sample in &grid_report.samples.samples {
            let value = sample.value.and_then(|value| hreal_from_f64(value).ok());
            push_sdf_sample(&mut diagnostics, &mut field_values, value, &iso_value);
        }

        mesh_from_sampled_field(
            field_values,
            min_pt,
            max_pt,
            nx,
            ny,
            nz,
            metadata,
            diagnostics,
        )
    }

    /// Return a Mesh created by meshing a signed distance field and diagnostics
    /// describing the sampled field and generated triangle stream.
    ///
    /// Surface-net vertices still arrive from `fast_surface_nets` as primitive
    /// floats, but bounding inputs, iso values, triangle coordinates, normals,
    /// and degenerate-triangle classification are lifted through
    /// `hyperlattice`/`hyperreal` before topology diagnostics are recorded.
    /// This preserves csgrs as a CAD composition layer over hyper geometry
    /// types and follows Yap's exact-geometric-computation boundary model
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The sampled contouring
    /// method is Gibson's constrained elastic surface-net construction:
    /// Gibson, "Constrained Elastic Surface Nets," MERL TR99-24, 1999.
    pub fn sdf_with_diagnostics<F>(
        sdf: F,
        resolution: (usize, usize, usize),
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        iso_value: Real,
        metadata: M,
    ) -> (Mesh<M>, SdfDiagnostics)
    where
        F: Fn(&Point3<Real>) -> Real + Sync + Send,
    {
        // Early return if resolution is degenerate
        let nx = resolution.0.max(2) as u32;
        let ny = resolution.1.max(2) as u32;
        let nz = resolution.2.max(2) as u32;
        let mut diagnostics = SdfDiagnostics {
            resolution: (nx, ny, nz),
            sample_count: (nx * ny * nz) as usize,
            ..SdfDiagnostics::default()
        };

        if hvector3_from_point3(&min_pt).is_none()
            || hvector3_from_point3(&max_pt).is_none()
            || hreal_from_f64(iso_value).is_err()
        {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        }

        // Allocate storage for field values:
        let array_size = (nx * ny * nz) as usize;
        let mut field_values = SdfSampleField::with_capacity(array_size);
        let Some(grid) = SamplingGrid::from_bounds(min_pt, max_pt, nx, ny, nz, iso_value)
        else {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
            diagnostics.positive_sample_count = diagnostics.sample_count;
            return (Mesh::empty(metadata), diagnostics);
        };

        // Sample the SDF at each grid cell with optimized iteration pattern:
        // **Mathematical Foundation**: For SDF f(p), we sample at regular intervals
        // and store (f(p) - iso_value) so surface_nets finds zero-crossings at iso_value.
        // **Optimization**: Linear memory access pattern with better cache locality.
        #[allow(clippy::unnecessary_cast)]
        for i in 0..(nx * ny * nz) {
            let iz = i / (nx * ny);
            let remainder = i % (nx * ny);
            let iy = remainder / nx;
            let ix = remainder % nx;

            let Some(p) = grid.point_at(ix, iy, iz) else {
                push_sdf_sample(&mut diagnostics, &mut field_values, None, &grid.iso);
                continue;
            };
            let sdf_val = sdf(&p);

            push_sdf_sample(
                &mut diagnostics,
                &mut field_values,
                hreal_from_f64(sdf_val).ok(),
                &grid.iso,
            );
        }

        mesh_from_sampled_field(
            field_values,
            min_pt,
            max_pt,
            nx,
            ny,
            nz,
            metadata,
            diagnostics,
        )
    }
}

#[derive(Clone, Debug)]
struct SdfSampleField {
    hyper_values: Vec<HReal>,
    surface_nets_values: Vec<F32>,
}

impl SdfSampleField {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            hyper_values: Vec::with_capacity(capacity),
            surface_nets_values: Vec::with_capacity(capacity),
        }
    }

    fn push_hyper_sample(&mut self, shifted: HReal) -> bool {
        let Some(surface_value) = surface_nets_scalar(&shifted) else {
            self.push_nonfinite_sample();
            return false;
        };
        self.hyper_values.push(shifted);
        self.surface_nets_values.push(surface_value);
        true
    }

    fn push_nonfinite_sample(&mut self) {
        let sentinel = hreal_from_f64(1.0e10).expect("finite SDF sentinel");
        self.hyper_values.push(sentinel);
        self.surface_nets_values.push(1.0e10_f32);
    }
}

#[derive(Clone, Debug)]
struct SamplingGrid {
    origin: HPoint3,
    step: HPoint3,
    iso: HReal,
}

impl SamplingGrid {
    fn from_bounds(
        min_pt: Point3<Real>,
        max_pt: Point3<Real>,
        nx: u32,
        ny: u32,
        nz: u32,
        iso_value: Real,
    ) -> Option<Self> {
        let origin = hpoint3_from_point3(&min_pt)?;
        let max = hpoint3_from_point3(&max_pt)?;
        let step = HPoint3::new(
            ((max.x - origin.x.clone()) / hreal_from_f64(nx as Real - 1.0).ok()?).ok()?,
            ((max.y - origin.y.clone()) / hreal_from_f64(ny as Real - 1.0).ok()?).ok()?,
            ((max.z - origin.z.clone()) / hreal_from_f64(nz as Real - 1.0).ok()?).ok()?,
        );
        Some(Self {
            origin,
            step,
            iso: hreal_from_f64(iso_value).ok()?,
        })
    }

    fn point_at(&self, ix: u32, iy: u32, iz: u32) -> Option<Point3<Real>> {
        let x =
            self.origin.x.clone() + self.step.x.clone() * hreal_from_f64(ix as Real).ok()?;
        let y =
            self.origin.y.clone() + self.step.y.clone() * hreal_from_f64(iy as Real).ok()?;
        let z =
            self.origin.z.clone() + self.step.z.clone() * hreal_from_f64(iz as Real).ok()?;
        point3_from_hpoint3(&HPoint3::new(x, y, z))
    }

    fn point_from_surface_position(&self, position: [F32; 3]) -> Option<Point3<Real>> {
        let x =
            self.origin.x.clone() + self.step.x.clone() * hreal_from_f32(position[0]).ok()?;
        let y =
            self.origin.y.clone() + self.step.y.clone() * hreal_from_f32(position[1]).ok()?;
        let z =
            self.origin.z.clone() + self.step.z.clone() * hreal_from_f32(position[2]).ok()?;
        point3_from_hpoint3(&HPoint3::new(x, y, z))
    }
}

fn push_sdf_sample(
    diagnostics: &mut SdfDiagnostics,
    field_values: &mut SdfSampleField,
    value: Option<HReal>,
    iso_value: &HReal,
) {
    if let Some(sdf_val) = value {
        let shifted = sdf_val.clone() - iso_value.clone();
        if !field_values.push_hyper_sample(shifted.clone()) {
            diagnostics.non_finite_sample_count += 1;
            diagnostics.positive_sample_count += 1;
            return;
        }
        let Some(sdf_val_f64) = hreal_to_f64(&sdf_val) else {
            diagnostics.non_finite_sample_count += 1;
            diagnostics.positive_sample_count += 1;
            return;
        };
        diagnostics.finite_sample_count += 1;
        diagnostics.min_finite_value = Some(
            diagnostics
                .min_finite_value
                .map_or(sdf_val_f64, |current| current.min(sdf_val_f64)),
        );
        diagnostics.max_finite_value = Some(
            diagnostics
                .max_finite_value
                .map_or(sdf_val_f64, |current| current.max(sdf_val_f64)),
        );
        match hreal_sign(&shifted) {
            Some(RealSign::Negative) => diagnostics.negative_sample_count += 1,
            Some(RealSign::Positive) => diagnostics.positive_sample_count += 1,
            Some(RealSign::Zero) => diagnostics.zero_sample_count += 1,
            None => {
                diagnostics.non_finite_sample_count += 1;
                diagnostics.positive_sample_count += 1;
            },
        }
    } else {
        diagnostics.non_finite_sample_count += 1;
        diagnostics.positive_sample_count += 1;
        field_values.push_nonfinite_sample();
    }
}

fn mesh_from_sampled_field<M: Clone + Debug + Send + Sync>(
    field_values: SdfSampleField,
    min_pt: Point3<Real>,
    max_pt: Point3<Real>,
    nx: u32,
    ny: u32,
    nz: u32,
    metadata: M,
    mut diagnostics: SdfDiagnostics,
) -> (Mesh<M>, SdfDiagnostics) {
    let Some(grid) = SamplingGrid::from_bounds(min_pt, max_pt, nx, ny, nz, 0.0) else {
        diagnostics.non_finite_sample_count = diagnostics.sample_count;
        diagnostics.positive_sample_count = diagnostics.sample_count;
        return (Mesh::empty(metadata), diagnostics);
    };

    // The shape describing our discrete grid for Surface Nets:
    #[derive(Clone, Copy)]
    struct GridShape {
        nx: u32,
        ny: u32,
        nz: u32,
    }

    impl fast_surface_nets::ndshape::Shape<3> for GridShape {
        type Coord = u32;

        #[inline]
        fn as_array(&self) -> [Self::Coord; 3] {
            [self.nx, self.ny, self.nz]
        }

        fn size(&self) -> Self::Coord {
            self.nx * self.ny * self.nz
        }

        fn usize(&self) -> usize {
            (self.nx * self.ny * self.nz) as usize
        }

        fn linearize(&self, coords: [Self::Coord; 3]) -> u32 {
            let [x, y, z] = coords;
            (z * self.ny + y) * self.nx + x
        }

        fn delinearize(&self, i: u32) -> [Self::Coord; 3] {
            let x = i % self.nx;
            let yz = i / self.nx;
            let y = yz % self.ny;
            let z = yz / self.ny;
            [x, y, z]
        }
    }

    let shape = GridShape { nx, ny, nz };
    diagnostics.crossing_cell_count =
        count_crossing_cells(&field_values.hyper_values, nx, ny, nz);

    // `SurfaceNetsBuffer` collects the positions, normals, and triangle indices
    let mut sn_buffer = SurfaceNetsBuffer::default();

    // The max valid coordinate in each dimension
    let max_x = nx - 1;
    let max_y = ny - 1;
    let max_z = nz - 1;

    // Run surface nets
    surface_nets(
        &field_values.surface_nets_values,
        &shape,
        [0, 0, 0],
        [max_x, max_y, max_z],
        &mut sn_buffer,
    );
    diagnostics.surface_nets_vertex_count = sn_buffer.positions.len();
    diagnostics.surface_nets_index_count = sn_buffer.indices.len();

    // Convert the resulting triangles into Mesh polygons
    let mut triangles = Vec::with_capacity(sn_buffer.indices.len() / 3);

    for tri in sn_buffer.indices.chunks_exact(3) {
        let i0 = tri[0] as usize;
        let i1 = tri[1] as usize;
        let i2 = tri[2] as usize;

        let p0i = sn_buffer.positions[i0];
        let p1i = sn_buffer.positions[i1];
        let p2i = sn_buffer.positions[i2];

        let Some(p0) = grid.point_from_surface_position(p0i) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };
        let Some(p1) = grid.point_from_surface_position(p1i) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };
        let Some(p2) = grid.point_from_surface_position(p2i) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };

        // Retrieve precomputed normal from Surface Nets:
        let n0 = sn_buffer.normals[i0];
        let n1 = sn_buffer.normals[i1];
        let n2 = sn_buffer.normals[i2];

        let Some(n0v) = vector3_from_f32_boundary(n0) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };
        let Some(n1v) = vector3_from_f32_boundary(n1) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };
        let Some(n2v) = vector3_from_f32_boundary(n2) else {
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        };

        if !(finite_point3(&p0)
            && finite_point3(&p1)
            && finite_point3(&p2)
            && finite_vector3(&n0v)
            && finite_vector3(&n1v)
            && finite_vector3(&n2v))
        {
            // at least one coordinate was NaN/±∞ – ignore this triangle
            diagnostics.skipped_non_finite_triangle_count += 1;
            continue;
        }

        if !htriangle_area2_exceeds_epsilon(&p0, &p1, &p2, Real::EPSILON) {
            diagnostics.degenerate_triangle_count += 1;
        }

        let v0 = Vertex::new(p0, n0v);
        let v1 = Vertex::new(p1, n1v);
        let v2 = Vertex::new(p2, n2v);

        // Note: reverse v1, v2 if you need to fix winding
        let poly = Polygon::new(vec![v0, v1, v2], metadata.clone());
        triangles.push(poly);
        diagnostics.emitted_triangle_count += 1;
    }

    // Return as a Mesh
    (Mesh::from_polygons(&triangles, metadata), diagnostics)
}

fn hypersdf_grid(
    min_pt: Point3<Real>,
    max_pt: Point3<Real>,
    nx: u32,
    ny: u32,
    nz: u32,
) -> Option<SdfPreviewGrid> {
    let grid = SamplingGrid::from_bounds(min_pt, max_pt, nx, ny, nz, 0.0)?;
    Some(SdfPreviewGrid::new(grid.origin, grid.step, [nx, ny, nz]))
}

fn hpoint3_from_point3(point: &Point3<Real>) -> Option<HPoint3> {
    HPoint3::try_from_f64_array([point.x, point.y, point.z]).ok()
}

#[allow(dead_code)]
fn point3_from_hpoint3(point: &HPoint3) -> Option<Point3<Real>> {
    let [x, y, z] = point.to_f64_array_lossy()?;
    Some(Point3::new(x, y, z))
}

#[inline]
fn finite_point3(point: &Point3<Real>) -> bool {
    hvector3_from_point3(point).is_some()
}

#[inline]
fn finite_vector3(vector: &Vector3<Real>) -> bool {
    hvector3_from_vector3(vector).is_some()
}

fn surface_nets_scalar(value: &HReal) -> Option<F32> {
    let sign = hreal_sign(value)?;
    let boundary = hreal_to_f64(value)?;
    let value = boundary as F32;
    let value = if value == 0.0 {
        match sign {
            RealSign::Negative => -F32::MIN_POSITIVE,
            RealSign::Positive => F32::MIN_POSITIVE,
            RealSign::Zero => 0.0,
        }
    } else if value.is_infinite() {
        match sign {
            RealSign::Negative => -F32::MAX,
            RealSign::Positive => F32::MAX,
            RealSign::Zero => 0.0,
        }
    } else {
        value
    };
    hreal_from_f32(value).ok()?;
    Some(value)
}

fn vector3_from_f32_boundary(vector: [F32; 3]) -> Option<Vector3<Real>> {
    let [x, y, z] = hyperlattice::Vector3::try_from_f32_array(vector)
        .ok()?
        .to_f64_array_lossy()?;
    Some(Vector3::new(x, y, z))
}

fn count_crossing_cells(field_values: &[HReal], nx: u32, ny: u32, nz: u32) -> usize {
    if nx < 2 || ny < 2 || nz < 2 {
        return 0;
    }

    let index = |x: u32, y: u32, z: u32| -> usize { ((z * ny + y) * nx + x) as usize };
    let mut count = 0;

    for z in 0..(nz - 1) {
        for y in 0..(ny - 1) {
            for x in 0..(nx - 1) {
                let corners = [
                    &field_values[index(x, y, z)],
                    &field_values[index(x + 1, y, z)],
                    &field_values[index(x, y + 1, z)],
                    &field_values[index(x + 1, y + 1, z)],
                    &field_values[index(x, y, z + 1)],
                    &field_values[index(x + 1, y, z + 1)],
                    &field_values[index(x, y + 1, z + 1)],
                    &field_values[index(x + 1, y + 1, z + 1)],
                ];
                let signs = corners.map(hreal_sign);
                let has_negative = signs.contains(&Some(RealSign::Negative));
                let has_positive = signs.contains(&Some(RealSign::Positive));
                let has_zero = signs.contains(&Some(RealSign::Zero));
                if (has_negative && has_positive) || has_zero {
                    count += 1;
                }
            }
        }
    }

    count
}
