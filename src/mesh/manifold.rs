//! Exact manifoldness checks for transitional triangle meshes.

use crate::mesh::Mesh;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    /// Return whether this mesh validates as a closed two-manifold in `hypermesh`.
    ///
    /// This method is intentionally routed through [`hypermesh::ExactMesh`]
    /// instead of the old `csgrs` tolerance hash. Edge incidence, duplicate
    /// directed edges, vertex links, and triangle degeneracy are topology
    /// facts owned by `hypermesh`; `csgrs` only supplies the current CAD mesh
    /// stream through its audited adapter.
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
        let Ok(mesh) = self.to_hypermesh_exact() else {
            return false;
        };
        mesh.validate_retained_state().is_ok() && mesh.facts().mesh.closed_manifold
    }
}
