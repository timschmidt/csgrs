//! Design-readiness checks grouped by the data model they operate on.
//!
//! `layer` checks work on already-flattened 2D geometry such as Gerber-derived
//! `Profile` layers. `drill` checks focus on holes, slots, and cross-source drill
//! tables. `board` checks use richer board context such as KiCad nets, vias,
//! component features, and panel intent.
//! `safety` checks focus on voltage, board-edge, and ESD protective-interface
//! readiness that benefits from board context but has a distinct review owner.
//! `signal` checks focus on mixed-signal partitioning and quiet-net guard or
//! return-path proximity.
//! `continuity` checks focus on same-net geometry that may be electrically
//! severed even when source net names still match.
//! `differential` checks focus on differential-pair coupling and spacing review
//! that benefits from net-name intent plus exact copper geometry.
//! `return_path` checks focus on split-plane and reference-island hazards for
//! high-speed copper.
//! `power_integrity` checks focus on high-current pad-entry and copper-spreading
//! readiness.
//! `mechanical` checks focus on chassis, mounting, and keepout intent that uses
//! board context but is primarily mechanical production readiness.

mod artifact_handoff;
mod artifact_table;
mod artifacts;
pub mod assembly;
pub mod board;
mod constraints;
pub mod continuity;
pub mod dense_pad;
pub mod differential;
mod distance;
pub mod drill;
mod excellon;
mod impedance;
pub mod layer;
pub mod manifest;
pub mod mechanical;
mod net_class;
mod net_scope;
mod outline;
pub mod power;
pub mod power_integrity;
pub mod return_path;
pub mod rf;
pub mod safety;
pub mod signal;
mod spatial;
mod spread;
pub mod stencil;
mod surface_finish;
pub mod thermal;

pub use artifacts::*;
pub use assembly::*;
pub use board::*;
pub use constraints::*;
pub use continuity::*;
pub use dense_pad::*;
pub use differential::*;
pub use drill::*;
pub use excellon::*;
pub use layer::*;
pub use manifest::*;
pub use mechanical::*;
pub use power::*;
pub use power_integrity::*;
pub use return_path::*;
pub use rf::*;
pub use safety::*;
pub use signal::*;
pub use stencil::*;
pub use thermal::*;

use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, PcbSketch, Scalar};
use hyperreal::RealSign;

pub(crate) fn offset_for_check(
    sketch: &PcbSketch,
    distance: Scalar,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<PcbSketch, Box<Violation>> {
    if distance.refine_sign_until(-128) == Some(RealSign::Zero) {
        return Ok(sketch.clone());
    }
    sketch.offset(distance).map_err(|uncertainty| {
        Box::new(geometry_uncertainty_violation(
            requested_check,
            layers,
            uncertainty,
        ))
    })
}

pub(crate) fn intersection_for_check(
    left: &PcbSketch,
    right: &PcbSketch,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<PcbSketch, Box<Violation>> {
    left.try_intersection(right).map_err(|error| {
        Box::new(geometry_uncertainty_violation(
            requested_check,
            layers,
            PcbGeometryUncertainty {
                operation: "profile-intersection".into(),
                source: left
                    .metadata()
                    .as_ref()
                    .map(|metadata| metadata.name.clone()),
                detail: error.to_string(),
            },
        ))
    })
}

pub(crate) fn difference_for_check(
    left: &PcbSketch,
    right: &PcbSketch,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<PcbSketch, Box<Violation>> {
    left.try_difference(right).map_err(|error| {
        Box::new(geometry_uncertainty_violation(
            requested_check,
            layers,
            PcbGeometryUncertainty {
                operation: "profile-difference".into(),
                source: left
                    .metadata()
                    .as_ref()
                    .map(|metadata| metadata.name.clone()),
                detail: error.to_string(),
            },
        ))
    })
}

pub(crate) fn union_for_check(
    left: &PcbSketch,
    right: &PcbSketch,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<PcbSketch, Box<Violation>> {
    left.try_union(right).map_err(|error| {
        Box::new(geometry_uncertainty_violation(
            requested_check,
            layers,
            PcbGeometryUncertainty {
                operation: "profile-union".into(),
                source: left
                    .metadata()
                    .as_ref()
                    .map(|metadata| metadata.name.clone()),
                detail: error.to_string(),
            },
        ))
    })
}

#[allow(dead_code)] // Kept with the complete Boolean gateway set for future XOR-based checks.
pub(crate) fn xor_for_check(
    left: &PcbSketch,
    right: &PcbSketch,
    requested_check: &str,
    layers: Vec<String>,
) -> Result<PcbSketch, Box<Violation>> {
    left.try_xor(right).map_err(|error| {
        Box::new(geometry_uncertainty_violation(
            requested_check,
            layers,
            PcbGeometryUncertainty {
                operation: "profile-xor".into(),
                source: left
                    .metadata()
                    .as_ref()
                    .map(|metadata| metadata.name.clone()),
                detail: error.to_string(),
            },
        ))
    })
}

fn geometry_uncertainty_violation(
    requested_check: &str,
    layers: Vec<String>,
    uncertainty: PcbGeometryUncertainty,
) -> Violation {
    Violation::new(
        "geometry-uncertainty",
        Severity::Error,
        layers,
        None,
        Vec::new(),
        Vec::new(),
        Some(format!(
            "{requested_check} could not certify required geometry: {uncertainty}"
        )),
    )
}

#[cfg(test)]
mod exact_math_audit_tests {
    use std::fs;
    use std::path::Path;

    #[test]
    fn production_geometry_uses_fallible_boolean_gateways() {
        let checks_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("src/checks");
        for entry in fs::read_dir(&checks_dir).expect("read check modules") {
            let entry = entry.expect("read check module entry");
            let path = entry.path();
            if path.extension().and_then(|extension| extension.to_str()) != Some("rs") {
                continue;
            }
            let file_name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("");
            let source = fs::read_to_string(&path).expect("read check module source");
            if file_name != "mod.rs" {
                for forbidden in [
                    ".try_union(",
                    ".try_difference(",
                    ".try_intersection(",
                    ".try_xor(",
                ] {
                    assert!(
                        !source.contains(forbidden),
                        "{} bypasses the typed geometry-uncertainty gateway with {forbidden}",
                        path.display()
                    );
                }
            }
            for forbidden in [".union(", ".difference(", ".intersection(", ".xor("] {
                if source.contains(forbidden) {
                    assert!(
                        matches!(
                            file_name,
                            "artifacts.rs" | "constraints.rs" | "differential.rs" | "mod.rs"
                        ),
                        "{} contains an infallible geometry Boolean call {forbidden}",
                        path.display()
                    );
                }
            }
        }

        let authoring = fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("src/authoring_intent.rs"),
        )
        .expect("read authoring intent source");
        for forbidden in [
            ".try_union(",
            ".try_difference(",
            ".try_intersection(",
            ".try_xor(",
            ".union(",
            ".difference(",
            ".intersection(",
            ".xor(",
        ] {
            assert!(
                !authoring.contains(forbidden),
                "authoring intent bypasses the typed geometry-uncertainty gateway with {forbidden}"
            );
        }
    }
}
