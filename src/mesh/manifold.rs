//! Exact manifoldness checks for transitional triangle meshes.

use crate::mesh::Mesh;
use crate::mesh::hypermesh::hypermesh_is_closed_manifold;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Return whether this mesh validates as a closed two-manifold in `hypermesh`.
    ///
    /// This method is intentionally routed through the exact `hypermesh`
    /// adapter instead of a tolerance hash. Edge incidence, duplicate directed
    /// edges, vertex links, and triangle degeneracy are topology facts over the
    /// exact imported triangle stream; `csgrs` only supplies the current CAD
    /// mesh stream through its audited adapter.
    ///
    /// The local vertex-star manifoldness criterion mirrors Boissonnat,
    /// Devillers, Pion, Teillaud, and Yvinec, "Triangulations in CGAL,"
    /// *Computational Geometry* 22.1-3 (2002), while exact predicate routing
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7.1-2 (1997),
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    ///
    /// # Returns
    ///
    /// - `true`: if `hypermesh` accepts the object as a closed two-manifold.
    /// - `false`: if construction, retained-state replay, or closed-manifold
    ///   facts reject it.
    pub fn is_manifold(&self) -> bool {
        if let Some(is_manifold) = self.polygons.manifold_fact() {
            return is_manifold;
        }

        let Ok(mesh) = self.to_hypermesh_triangle_mesh() else {
            self.cache_manifold_fact(false);
            return false;
        };
        let is_manifold = hypermesh_is_closed_manifold(&mesh);
        self.cache_manifold_fact(is_manifold);
        is_manifold
    }

    pub(crate) fn cache_manifold_fact(&self, is_manifold: bool) {
        self.polygons.retain_manifold_fact(is_manifold);
    }

    pub(super) fn has_convex_pwn_fact(&self) -> bool {
        self.polygons.has_convex_pwn_fact()
    }

    pub(crate) fn cache_convex_pwn_fact(&self) {
        self.polygons.retain_convex_pwn_fact();
    }
}
