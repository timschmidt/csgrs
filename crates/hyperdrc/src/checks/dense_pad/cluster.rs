//! Required dense-cluster qualification and optional minimum ordering.

use super::DENSE_PAD_CLUSTER_MIN_PADS;
use crate::checks::{measurements, spatial::NativePointSpatialIndex};
use crate::kicad::CopperFeature;
use crate::{PcbGeometryUncertainty, Scalar};

/// One certified qualifying metric. The diagnostic calls it a minimum only
/// when every candidate and comparison needed for that claim was certified.
pub(super) struct MetricWitness<T> {
    pub(super) item: T,
    pub(super) value: Scalar,
    minimum_certified: bool,
}

impl<T> MetricWitness<T> {
    pub(super) fn description(&self, minimum: &str, witness: &str) -> String {
        format!(
            "{} {:.6}",
            if self.minimum_certified {
                minimum
            } else {
                witness
            },
            self.value.fixed_decimal()
        )
    }
}

/// A certified qualifying alternative proves existence even if another metric
/// or optional minimum order is unavailable. Without that witness, unresolved
/// candidates remain a required failure. The enclosing operation checks the
/// shared budget before publishing any result.
pub(super) fn select_below<T>(
    candidates: impl IntoIterator<Item = Result<Option<(T, Scalar)>, PcbGeometryUncertainty>>,
    threshold: &Scalar,
    closed: bool,
) -> Result<Option<MetricWitness<T>>, PcbGeometryUncertainty> {
    let mut selected: Option<MetricWitness<T>> = None;
    let mut uncertainty = None;
    for candidate in candidates {
        let candidate = candidate.and_then(|candidate| {
            let Some((item, value)) = candidate else {
                return Ok(None);
            };
            let order = crate::scalar::try_compare(&value, threshold)?;
            Ok((order.is_lt() || (closed && order.is_eq())).then_some((item, value)))
        });
        match candidate {
            Ok(Some((item, value))) => {
                let replace = selected.as_ref().map_or(Ok(true), |current| {
                    crate::scalar::try_compare(&value, &current.value).map(|order| order.is_lt())
                });
                match replace {
                    Ok(true) => {
                        selected = Some(MetricWitness {
                            item,
                            value,
                            minimum_certified: true,
                        })
                    }
                    Ok(false) => {}
                    Err(error) => {
                        uncertainty.get_or_insert(error);
                    }
                }
            }
            Ok(None) => {}
            Err(error) => {
                uncertainty.get_or_insert(error);
            }
        }
    }
    if let Some(mut selected) = selected {
        selected.minimum_certified = uncertainty.is_none();
        Ok(Some(selected))
    } else {
        uncertainty.map_or(Ok(None), Err)
    }
}

pub(super) struct DenseCluster {
    pub(super) pitch: MetricWitness<()>,
    pub(super) center: [Scalar; 2],
}

pub(super) fn dense_cluster_context(
    pads: &[&CopperFeature],
    threshold: &Scalar,
) -> Result<Option<DenseCluster>, PcbGeometryUncertainty> {
    if pads.len() < DENSE_PAD_CLUSTER_MIN_PADS
        || crate::scalar::compare(threshold, &Scalar::zero()).is_some_and(|order| order.is_le())
    {
        return Ok(None);
    }
    let index = NativePointSpatialIndex::new(pads.iter().map(|pad| &pad.location), threshold);
    let pairs = (0..pads.len()).flat_map(|left| {
        index
            .later_candidates_near(left)
            .into_iter()
            .map(move |right| (left, right))
    });
    let pitch = select_below(
        pairs.map(|(left, right)| {
            measurements::point_distance(&pads[left].location, &pads[right].location)
                .map(|pitch| Some(((), pitch)))
        }),
        threshold,
        true,
    )?;
    Ok(pitch.map(|pitch| DenseCluster {
        pitch,
        center: average_location(pads),
    }))
}

fn average_location(features: &[&CopperFeature]) -> [Scalar; 2] {
    let mut sum = [Scalar::zero(), Scalar::zero()];
    for feature in features {
        sum[0] += &feature.location[0];
        sum[1] += &feature.location[1];
    }
    // dense_cluster_context requires at least sixteen entries before calling.
    let count = Scalar::from(features.len() as u64);
    [
        (sum[0].clone() / &count).expect("nonempty feature cluster count"),
        (sum[1].clone() / &count).expect("nonempty feature cluster count"),
    ]
}

pub(super) fn likely_fiducial(feature: &CopperFeature) -> Result<bool, PcbGeometryUncertainty> {
    if feature.kind != crate::kicad::CopperKind::Pad || feature.net.is_some() {
        return Ok(false);
    }
    let Some(bounds) = feature.region.try_native_bounds()? else {
        return Ok(false);
    };
    let width = &bounds[2] - &bounds[0];
    let height = &bounds[3] - &bounds[1];
    let half = crate::scalar::scalar("0.5");
    let maximum = crate::scalar::scalar("2.5");
    let ratio = crate::scalar::scalar("0.75");
    // Equivalent to min >= 0.5, max <= 2.5, min/max >= 0.75 without
    // requiring an otherwise irrelevant width/height ordering or division.
    let bounds = [
        (width.clone(), half.clone()),
        (height.clone(), half),
        (maximum.clone(), width.clone()),
        (maximum, height.clone()),
        (width.clone(), &height * &ratio),
        (height, width * ratio),
    ];
    measurements::any_match(bounds, |(left, right)| {
        Ok(crate::scalar::try_compare(&left, &right)?.is_lt())
    })
    .map(|excluded| !excluded)
}

/// Below the cap this is the exact count; reaching the cap proves enough
/// matches regardless of unknown alternatives. Only sub-cap counts are rendered.
pub(super) fn capped_match_count<T>(
    candidates: impl IntoIterator<Item = T>,
    cap: usize,
    mut predicate: impl FnMut(T) -> Result<bool, PcbGeometryUncertainty>,
) -> Result<usize, PcbGeometryUncertainty> {
    let mut count = 0;
    let mut uncertainty = None;
    for candidate in candidates {
        match predicate(candidate) {
            Ok(true) => {
                count += 1;
                if count >= cap {
                    return Ok(count);
                }
            }
            Ok(false) => {}
            Err(error) => {
                uncertainty.get_or_insert(error);
            }
        }
    }
    uncertainty.map_or(Ok(count), Err)
}
