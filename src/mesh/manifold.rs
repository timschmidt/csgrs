//! Exact manifoldness checks for transitional triangle meshes.

use crate::mesh::Mesh;
use crate::mesh::hypermesh::hypermesh_is_closed_manifold;
use std::cell::RefCell;
use std::fmt::Debug;

#[derive(Clone, Debug)]
struct CachedManifoldFact {
    geometry_identity: Vec<u64>,
    is_manifold: bool,
}

thread_local! {
    static MANIFOLD_FACTS: RefCell<Vec<CachedManifoldFact>> = const { RefCell::new(Vec::new()) };
    static CONVEX_PWN_FACTS: RefCell<Vec<Vec<u64>>> = const { RefCell::new(Vec::new()) };
}

const MANIFOLD_FACT_CAPACITY: usize = 16;

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
        if let Some(is_manifold) = MANIFOLD_FACTS.with_borrow(|facts| {
            facts
                .iter()
                .rev()
                .find(|fact| self.geometry_identity_matches(&fact.geometry_identity))
                .map(|fact| fact.is_manifold)
        }) {
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

    pub(super) fn cache_manifold_fact(&self, is_manifold: bool) {
        let geometry_identity = self.geometry_identity();
        MANIFOLD_FACTS.with_borrow_mut(|facts| {
            if let Some(index) = facts
                .iter()
                .position(|fact| fact.geometry_identity == geometry_identity)
            {
                facts.remove(index);
            }
            if facts.len() == MANIFOLD_FACT_CAPACITY {
                facts.remove(0);
            }
            facts.push(CachedManifoldFact {
                geometry_identity,
                is_manifold,
            });
        });
    }

    pub(super) fn has_convex_pwn_fact(&self) -> bool {
        CONVEX_PWN_FACTS.with_borrow(|facts| {
            facts
                .iter()
                .rev()
                .any(|identity| self.geometry_identity_matches(identity))
        })
    }

    pub(super) fn cache_convex_pwn_fact(&self) {
        let geometry_identity = self.geometry_identity();
        CONVEX_PWN_FACTS.with_borrow_mut(|facts| {
            if let Some(index) = facts
                .iter()
                .position(|existing| *existing == geometry_identity)
            {
                facts.remove(index);
            }
            if facts.len() == MANIFOLD_FACT_CAPACITY {
                facts.remove(0);
            }
            facts.push(geometry_identity);
        });
    }
}
