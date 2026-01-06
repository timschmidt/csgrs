//! BMesh: boolmesh-backed CSG implementation for csgrs
//!
//! This type wraps a `boolmesh::Manifold` and implements the `CSG` trait,
//! so you can use boolmesh’s robust boolean kernel inside csgrs.

use crate::float_types::{
    parry3d::bounding_volume::Aabb,
    Real,
};
use crate::csg::CSG;

use boolmesh::{
    compute_boolean,
    prelude::{Manifold, OpType},
};

use nalgebra::{Matrix4, Point3};
use std::{fmt::Debug, sync::OnceLock};

#[cfg(feature = "mesh")]
use crate::mesh::Mesh;

pub mod triangulated;

/// A solid represented by boolmesh’s `Manifold`, wired into csgrs’ `CSG` trait.
///
/// `metadata` is whole-shape metadata, mirroring `Mesh<S>`.
#[derive(Clone)]
pub struct BMesh<S: Clone + Send + Sync + Debug> {
    /// Underlying robust manifold. `None` represents an empty solid.
    pub manifold: Option<Manifold>,
    /// Lazily computed Parry AABB for the solid.
    pub bounding_box: OnceLock<Aabb>,
    /// Optional whole-shape metadata.
    pub metadata: Option<S>,
}

impl<S: Clone + Send + Sync + Debug> Default for BMesh<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Clone + Send + Sync + Debug> BMesh<S> {
    /// Construct from a boolmesh `Manifold`.
    #[inline]
    pub fn from_manifold(manifold: Manifold, metadata: Option<S>) -> Self {
        BMesh {
            manifold: Some(manifold),
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Helper: are we the empty solid?
    #[inline]
    fn is_empty(&self) -> bool {
        self.manifold.is_none()
    }

    /// Core helper for boolean ops, handling empty cases and delegating to boolmesh.
    fn boolean(&self, other: &Self, op: OpType) -> Self {
        use OpType::*;

        match (&self.manifold, &other.manifold) {
            // Ø op Ø  => Ø
            (None, None) => BMesh::new(),

            // A op Ø
            (Some(_), None) => match op {
                Add | Subtract => self.clone(), // A ∪ Ø = A, A − Ø = A
                Intersect => BMesh::new(),      // A ∩ Ø = Ø
            },

            // Ø op B
            (None, Some(_)) => match op {
                Add => other.clone(),           // Ø ∪ B = B
                Subtract => BMesh::new(),       // Ø − B = Ø
                Intersect => BMesh::new(),      // Ø ∩ B = Ø
            },

            // A op B, both non-empty
            (Some(mp), Some(mq)) => {
                let m = compute_boolean(mp, mq, op)
                    .unwrap_or_else(|e| {
                        // We may want to change this to a different error strategy.
                        panic!("BMesh boolean operation failed: {e}");
                    });

                // Follow `Mesh` semantics: keep left-hand side metadata.
                BMesh {
                    manifold: Some(m),
                    bounding_box: OnceLock::new(),
                    metadata: self.metadata.clone(),
                }
            }
        }
    }

    /// Rebuild a manifold after applying a matrix transform to all vertex positions.
    ///
    /// Connectivity is kept by reusing the original triangle indices.
    fn transformed_manifold(&self, mat: &Matrix4<Real>) -> Option<Manifold> {
        let m = self.manifold.as_ref()?;

        // Flatten transformed positions
        let mut pos: Vec<Real> = Vec::with_capacity(m.ps.len() * 3);
        for v in &m.ps {
            let p = Point3::new(v.x, v.y, v.z);
            let hp = mat * p.to_homogeneous();
            // If homogeneous w is invalid, fall back to original position.
            let p_t = Point3::from_homogeneous(hp).unwrap_or(p);
            pos.push(p_t.x);
            pos.push(p_t.y);
            pos.push(p_t.z);
        }

        // Reuse the current triangle connectivity.
        let mut idx: Vec<usize> = Vec::with_capacity(m.nf * 3);
        for f in 0..m.nf {
            let base = f * 3;
            idx.push(m.hs[base].tail);
            idx.push(m.hs[base + 1].tail);
            idx.push(m.hs[base + 2].tail);
        }

        Some(
            Manifold::new(&pos, &idx).unwrap_or_else(|e| {
                panic!("BMesh::transform – boolmesh::Manifold::new failed: {e}");
            }),
        )
    }

    /// Rebuild a manifold with flipped triangle winding (geometric complement).
    fn inverted_manifold(&self) -> Option<Manifold> {
        let m = self.manifold.as_ref()?;

        let mut pos: Vec<Real> = Vec::with_capacity(m.ps.len() * 3);
        for v in &m.ps {
            pos.push(v.x);
            pos.push(v.y);
            pos.push(v.z);
        }

        // Flip orientation: (v0, v1, v2) -> (v0, v2, v1)
        let mut idx: Vec<usize> = Vec::with_capacity(m.nf * 3);
        for f in 0..m.nf {
            let base = f * 3;
            let v0 = m.hs[base].tail;
            let v1 = m.hs[base + 1].tail;
            let v2 = m.hs[base + 2].tail;
            idx.push(v0);
            idx.push(v2);
            idx.push(v1);
        }

        Some(
            Manifold::new(&pos, &idx).unwrap_or_else(|e| {
                panic!("BMesh::inverse – boolmesh::Manifold::new failed: {e}");
            }),
        )
    }
}

impl<S: Clone + Send + Sync + Debug> CSG for BMesh<S> {
    /// New empty BMesh (no manifold).
    fn new() -> Self {
        BMesh {
            manifold: None,
            bounding_box: OnceLock::new(),
            metadata: None,
        }
    }

    /// Union via boolmesh.
    fn union(&self, other: &Self) -> Self {
        self.boolean(other, OpType::Add)
    }

    /// Difference via boolmesh (`self \ other`).
    fn difference(&self, other: &Self) -> Self {
        self.boolean(other, OpType::Subtract)
    }

    /// Intersection via boolmesh.
    fn intersection(&self, other: &Self) -> Self {
        self.boolean(other, OpType::Intersect)
    }

    /// Symmetric difference: (A \ B) ∪ (B \ A)
    fn xor(&self, other: &Self) -> Self {
        let a_sub_b = self.difference(other);
        let b_sub_a = other.difference(self);
        a_sub_b.union(&b_sub_a)
    }

    /// Apply a 4×4 transform to all vertices and rebuild the boolmesh manifold.
    fn transform(&self, mat: &Matrix4<Real>) -> Self {
        if self.is_empty() {
            return self.clone();
        }

        let manifold = self
            .transformed_manifold(mat)
            .expect("BMesh::transform – manifold unexpectedly empty");

        BMesh {
            manifold: Some(manifold),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }

    /// AABB of the solid (derived from the boolmesh manifold’s bounding box).
    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            if let Some(m) = &self.manifold {
                let bb = &m.bounding_box;
                let mins = Point3::new(bb.min.x, bb.min.y, bb.min.z);
                let maxs = Point3::new(bb.max.x, bb.max.y, bb.max.z);
                Aabb::new(mins, maxs)
            } else {
                Aabb::new(Point3::origin(), Point3::origin())
            }
        })
    }

    /// Reset cached AABB.
    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }

    /// Geometric complement: flip all triangle windings and rebuild the manifold.
    ///
    /// This is implemented as an orientation flip, which in boolmesh’s pipeline
    /// inverts the solid’s inside/outside classification.
    fn inverse(&self) -> Self {
        if self.is_empty() {
            return self.clone();
        }

        let manifold = self
            .inverted_manifold()
            .expect("BMesh::inverse – manifold unexpectedly empty");

        BMesh {
            manifold: Some(manifold),
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
}

impl<S: Clone + Send + Sync + Debug> From<Mesh<S>> for BMesh<S> {
    fn from(mesh: Mesh<S>) -> Self {
        // Keep the metadata from the original mesh
        let metadata = mesh.metadata.clone();

        // Triangulate the mesh
        let tri_mesh = mesh.triangulate();

        // Extract vertices and triangle indices from the triangulated mesh
        let (vertices, indices) = tri_mesh.get_vertices_and_indices();

        // Flatten vertices into boolmesh's `Vec<Real>` layout: [x0, y0, z0, x1, y1, z1, ...]
        let mut pos_bool: Vec<boolmesh::Real> = Vec::with_capacity(vertices.len() * 3);
        for v in &vertices {
            pos_bool.push(v.x as boolmesh::Real);
            pos_bool.push(v.y as boolmesh::Real);
            pos_bool.push(v.z as boolmesh::Real);
        }

        // Flatten triangle indices into `Vec<usize>`
        let mut idx_bool: Vec<usize> = Vec::with_capacity(indices.len() * 3);
        for tri in &indices {
            idx_bool.push(tri[0] as usize);
            idx_bool.push(tri[1] as usize);
            idx_bool.push(tri[2] as usize);
        }

        // If there are no triangles, treat as empty BMesh
        let manifold = if idx_bool.is_empty() {
            None
        } else {
            Some(
                Manifold::new(&pos_bool, &idx_bool)
                    .expect("boolmesh::Manifold::new failed when converting from Mesh"),
            )
        };

        BMesh {
            manifold,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }
}
