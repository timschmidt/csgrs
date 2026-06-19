//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![forbid(unsafe_code)]

//! Exact-facing mesh API for the hyper geometry stack.
//!
//! Mesh coordinates are carried as [`hyperlimit::Point3`] over
//! [`hyperreal::Real`]. Topology-affecting decisions are exposed through exact
//! predicate reports, certified outputs, or explicit blockers when the
//! implementation cannot prove a requested operation.

mod adapter;
mod adjacent;
pub(crate) mod adjacent_polygon;
mod affine_box;
mod affine_solid;
mod arrangement2d;
mod arrangement3d;
mod artifact;
mod audit;
mod boolean;
mod bounds;
mod box_solid;
mod cell_complex;
#[allow(dead_code)]
mod cells;
mod construction;
mod contained_adjacent;
mod convex;
mod error;
mod exact_key;
mod facts;
#[allow(dead_code)]
mod graph;
mod handoff;
mod intersection;
pub(crate) mod loop_triangulation;
mod mesh;
mod narrow;
mod orthogonal_solid;
mod package;
mod proposal;
mod readiness;
#[allow(dead_code)]
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
mod workspace;

pub use artifact::MeshArtifactManifest;
pub use boolean::{
    ExactBooleanEvaluation, ExactBooleanOperation, ExactBooleanRequest, ExactBooleanResult,
    ExactBoundaryBooleanPolicy, ExactReportFreshness,
};
pub use error::MeshError;
pub use mesh::{ExactMesh, ExactMeshValidationError, Triangle};
pub use package::ExactMeshConsumerDomain;
pub use region::ExactRegionSelection;
pub use regularization::ExactRegularizationPolicy;
pub use validation::ValidationPolicy;
pub use workspace::ExactBooleanWorkspace;
