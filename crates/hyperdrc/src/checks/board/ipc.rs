//! Native IPC-D-356 center matching and transactional board annotation.

use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::ipc356::Ipc356Point;
use crate::kicad::BoardModel;
use crate::report::{Severity, Violation};
use crate::{PcbGeometryUncertainty, Scalar};

/// Fill missing board nets and zero drill diameters from nearby IPC records.
///
/// Native point enclosures schedule candidates; checked squared distances
/// decide closed-radius matches under STRICT. Existing values and the first
/// matching source record retain priority. A copied diameter must be certified
/// nonnegative. No finite report coordinate is needed for annotation.
///
/// Updates are staged until every required predicate and the shared work-budget
/// guard succeed. Any failure leaves all board nets and diameters unchanged and
/// retains the board source on the typed error. The CLI propagates this error
/// before running checks or emitting a report from incomplete annotations.
pub fn apply_ipc356_nets(
    board: &mut BoardModel,
    points: &[Ipc356Point],
    tolerance: &Scalar,
) -> Result<(), PcbGeometryUncertainty> {
    let (copper_nets, drill_nets, drill_diameters) = crate::operations::evaluate(
        "ipc356-net-annotation",
        Some(&board.source),
        || {
            let copper_index = NativePointSpatialIndex::new(
                board.copper.iter().map(|feature| &feature.location),
                tolerance,
            );
            let drill_index = NativePointSpatialIndex::new(
                board.drills.iter().map(|drill| &drill.location),
                tolerance,
            );
            // Borrow only proposed metadata; cloning board geometry is unnecessary.
            let mut copper_nets: Vec<Option<&str>> = vec![None; board.copper.len()];
            let mut drill_nets: Vec<Option<&str>> = vec![None; board.drills.len()];
            let mut drill_diameters: Vec<Option<&Scalar>> = vec![None; board.drills.len()];
            for point in points {
                for index in copper_index.candidates_near(&point.location) {
                    let copper = &board.copper[index];
                    if copper.net.is_some() || copper_nets[index].is_some() {
                        continue;
                    }
                    if measurements::points_within_radius(
                        &copper.location,
                        &point.location,
                        tolerance,
                    )? {
                        copper_nets[index] = Some(&point.net);
                    }
                }
                for index in drill_index.candidates_near(&point.location) {
                    let drill = &board.drills[index];
                    let needs_net = drill.net.is_none() && drill_nets[index].is_none();
                    let diameter = drill_diameters[index].unwrap_or(&drill.diameter);
                    let needs_diameter = if point.diameter.is_some() {
                        crate::scalar::try_compare(diameter, &Scalar::zero())
                            .map(|order| order.is_eq())
                    } else {
                        Ok(false)
                    };
                    if !needs_net && matches!(needs_diameter, Ok(false)) {
                        continue;
                    }
                    if !measurements::points_within_radius(
                        &drill.location,
                        &point.location,
                        tolerance,
                    )? {
                        continue;
                    }
                    if needs_net {
                        drill_nets[index] = Some(&point.net);
                    }
                    // An unresolved zero diameter matters only for a nearby
                    // record that can supply a replacement.
                    if let Some(diameter) = &point.diameter
                        && needs_diameter?
                    {
                        measurements::drill_radius(diameter)?;
                        drill_diameters[index] = Some(diameter);
                    }
                }
            }
            log::trace!(
                "apply IPC-D-356 nets: points={} copper_buckets={} drill_buckets={} copper_matches={} drill_matches={} tolerance={tolerance:.6}",
                points.len(),
                copper_index.bucket_count(),
                drill_index.bucket_count(),
                copper_nets.iter().flatten().count(),
                drill_nets.iter().flatten().count()
            );
            Ok((copper_nets, drill_nets, drill_diameters))
        },
    )
    .map_err(|mut uncertainty| {
        if uncertainty.source.is_none() {
            uncertainty.source = Some(board.source.clone());
        }
        uncertainty
    })?;

    // The guard checks for exhaustion after its closure; publish only afterward.
    for (copper, net) in board.copper.iter_mut().zip(copper_nets) {
        if let Some(net) = net {
            copper.net = Some(net.to_owned());
        }
    }
    for ((drill, net), diameter) in board.drills.iter_mut().zip(drill_nets).zip(drill_diameters) {
        if let Some(net) = net {
            drill.net = Some(net.to_owned());
        }
        if let Some(diameter) = diameter {
            drill.diameter = diameter.clone();
        }
    }
    Ok(())
}

/// Report IPC records without a certified nearby native copper center.
///
/// A certified match proves coverage even if another candidate is unresolved.
/// If no match is certified, unresolved candidates remain uncertainty. Missing
/// records require a finite report marker; unavailable projection fails typed.
pub fn ipc356_coverage(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("ipc356-coverage", &[&board.source], || {
        crate::operations::evaluate("ipc356-coverage", Some(&board.source), || {
            let copper_index = NativePointSpatialIndex::new(
                board.copper.iter().map(|feature| &feature.location),
                tolerance,
            );
            let mut violations = Vec::new();
            for point in points {
                let mut uncertainty = None;
                let mut has_copper = false;
                for index in copper_index.candidates_near(&point.location) {
                    match measurements::points_within_radius(
                        &board.copper[index].location,
                        &point.location,
                        tolerance,
                    ) {
                        Ok(true) => {
                            has_copper = true;
                            break;
                        }
                        Ok(false) => {}
                        Err(error) => {
                            uncertainty.get_or_insert(error);
                        }
                    }
                }
                if has_copper {
                    continue;
                }
                if let Some(uncertainty) = uncertainty {
                    return Err(uncertainty);
                }
                let label = match (&point.reference, &point.pin) {
                    (Some(reference), Some(pin)) => format!("{reference}.{pin}"),
                    (Some(reference), None) => reference.clone(),
                    _ => "IPC-D-356 test record".to_string(),
                };
                violations.push(Violation::new(
                    "ipc356-coverage",
                    Severity::Warning,
                    vec![point.net.clone()],
                    None,
                    Vec::new(),
                    vec![measurements::report_point(&point.location)?],
                    Some(format!(
                        "{label} has no parsed KiCad copper feature within {tolerance}"
                    )),
                ));
            }
            log::trace!(
                "IPC-D-356 coverage readiness: points={} copper={} spatial_buckets={} tolerance={tolerance:.6} violations={}",
                points.len(),
                board.copper.len(),
                copper_index.bucket_count(),
                violations.len()
            );
            Ok(violations)
        })
    })
}

/// Run the `ipc356_drill_diameter` design-readiness check or report helper.
///
/// Diameter comparison uses a drill-center grid as a broad phase before exact
/// tolerance comparison. This keeps cross-source drill-table checks bounded on
/// dense IPC-D-356 fixtures while preserving the caller-visible diagnostic.
pub fn ipc356_drill_diameter(
    board: &BoardModel,
    points: &[Ipc356Point],
    tolerance: &Scalar,
) -> Vec<Violation> {
    crate::checks::run_geometry_check("ipc356-drill-diameter", &[&board.source], || {
        crate::operations::evaluate("ipc356-drill-diameter", Some(&board.source), || {
            let drill_index = NativePointSpatialIndex::new(
                board.drills.iter().map(|drill| &drill.location),
                tolerance,
            );
            let mut candidate_count = 0_usize;
            let mut violations = Vec::new();
            for point in points {
                let Some(ipc_diameter) = &point.diameter else {
                    continue;
                };
                for index in drill_index.candidates_near(&point.location) {
                    candidate_count += 1;
                    let drill = &board.drills[index];
                    if !measurements::points_within_radius(
                        &drill.location,
                        &point.location,
                        tolerance,
                    )? {
                        continue;
                    }
                    if crate::scalar::try_compare(&drill.diameter, &Scalar::zero())?.is_eq()
                        || crate::scalar::try_compare(
                            &(&drill.diameter - ipc_diameter).abs(),
                            tolerance,
                        )?
                        .is_le()
                    {
                        continue;
                    }
                    violations.push(Violation::new(
                        "ipc356-drill-diameter",
                        Severity::Warning,
                        vec![point.net.clone()],
                        None,
                        Vec::new(),
                        vec![
                            measurements::report_point(&drill.location)?,
                            measurements::report_point(&point.location)?,
                        ],
                        Some(format!(
                            "drill diameter {:.6} differs from IPC-D-356 diameter {:.6}",
                            drill.diameter, ipc_diameter
                        )),
                    ));
                }
            }
            log::trace!(
                "IPC-D-356 drill diameter readiness: points={} drills={} spatial_buckets={} candidate_pairs={} tolerance={tolerance:.6} violations={}",
                points.len(),
                board.drills.len(),
                drill_index.bucket_count(),
                candidate_count,
                violations.len()
            );
            Ok(violations)
        })
    })
}
