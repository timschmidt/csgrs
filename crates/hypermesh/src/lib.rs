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

mod adjacent;
pub(crate) mod adjacent_polygon;
mod affine_box;
mod affine_solid;
mod arrangement2d;
mod arrangement3d;
mod boolean;
mod bounds;
mod box_solid;
mod cell_complex;
mod cells;
mod construction;
mod contained_adjacent;
mod convex;
mod error;
mod exact_key;
mod facts;
mod graph;
mod intersection;
pub(crate) mod loop_triangulation;
mod mesh;
mod narrow;
mod orthogonal_solid;
mod region;
mod regularization;
mod reports;
mod scalar;
mod simplify;
mod solid;
mod topology;
mod validation;
mod view;
mod volumetric;
mod volumetric_cells;
mod winding;
mod witness;

pub use error::{ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError};
pub use mesh::{ExactMesh, ExactMeshValidationError, Triangle};
