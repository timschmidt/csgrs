//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![allow(unused_braces)]

#[cfg(feature = "legacy-boolean")]
mod boolean03;
#[cfg(feature = "legacy-boolean")]
mod boolean45;
#[cfg(feature = "legacy-boolean")]
mod common;
#[cfg(feature = "exact")]
pub mod exact;
#[cfg(feature = "legacy-boolean")]
mod legacy_adapter;
#[cfg(feature = "legacy-boolean")]
mod legacy_report;
#[cfg(feature = "legacy-boolean")]
mod manifold;
#[cfg(feature = "legacy-boolean")]
mod simplification;
#[cfg(all(test, feature = "legacy-boolean"))]
mod tests;
#[cfg(feature = "legacy-boolean")]
mod triangulation;

#[cfg(feature = "legacy-boolean")]
#[allow(unused_imports)]
use crate::common::*;
#[cfg(feature = "legacy-boolean")]
pub use crate::common::{K_PRECISION, Mat3, Real, Vec2, Vec3, Vec4};
#[cfg(feature = "legacy-boolean")]
pub use crate::legacy_adapter::compute_boolean_with_report;
#[cfg(feature = "legacy-boolean")]
pub use crate::legacy_report::{
    LegacyBooleanReport, LegacyBooleanReportError, LegacyBooleanResult,
};
#[cfg(feature = "legacy-boolean")]
#[allow(unused_imports)]
use crate::manifold::*;

pub mod prelude {
    #[cfg(feature = "legacy-boolean")]
    pub use crate::common::OpType;
    #[cfg(feature = "exact")]
    pub use crate::exact::{ExactMesh, ExactPoint3, MeshFacts, Triangle};
    #[cfg(feature = "legacy-boolean")]
    pub use crate::manifold::Manifold;
    #[cfg(feature = "legacy-boolean")]
    pub use crate::{
        LegacyBooleanReport, LegacyBooleanReportError, LegacyBooleanResult,
        compute_boolean_with_report,
    };
}
