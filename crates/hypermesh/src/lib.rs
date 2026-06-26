//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![forbid(unsafe_code)]

//! Exact-facing mesh API for the hyper geometry stack.
//!
//! [`ExactMesh`] is the primary entry point. It owns exact vertices, triangle
//! topology, retained validation facts, broad-phase bounds, and construction
//! provenance. Borrowed query and acceleration APIs start from
//! [`ExactMesh::view`] so callers can inspect retained facts without cloning
//! mesh storage.
//!
//! Mesh coordinates are carried as [`hyperlimit::Point3`] over
//! [`hyperreal::Real`]. Topology-affecting decisions are exposed through exact
//! predicate evidence, certified outputs, or explicit blockers when the
//! implementation cannot prove a requested operation.

mod mesh;

pub use mesh::ExactMesh;

#[doc(hidden)]
pub mod kernel {
    pub use crate::mesh::arrangement3d::{
        ArrangementEdgeRef, ArrangementFaceCellRef, ArrangementVertexRef, ArrangementView,
    };
    pub use crate::mesh::error::{
        ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, ExactMeshSourceSide,
    };
    pub use crate::mesh::view::{
        EdgeRef, ExactMeshRef, FaceRef, MeshView, PreparedMeshPair, PreparedMeshView, TriangleRef,
        VertexRef,
    };
}
