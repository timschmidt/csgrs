//! Borrowed structural definitions of exact procedural parallel curves.

use crate::{BezierParallel2, BezierSplitFragment2, Curve2, CurveParameterRange2};

/// A lossless structural view of a finite analytic parallel.
///
/// The parallel retains its polynomial or homogeneous rational source and
/// signed left-normal distance. The ascending range retains selected roots;
/// traversal is independent of its order. This view neither materializes
/// coordinates nor changes proof caches. It describes the current geometry,
/// not the certainty of an earlier construction's intent or its full history.
#[derive(Clone, Copy, Debug)]
pub struct CurveAnalyticParallel2<'a> {
    source: &'a Curve2,
    parallel: &'a BezierParallel2,
}

impl<'a> CurveAnalyticParallel2<'a> {
    /// Borrows the complete procedural support, including source and distance.
    pub const fn parallel(&self) -> &'a BezierParallel2 {
        self.parallel
    }

    /// Borrows the ascending finite range in the original source chart.
    pub fn range(&self) -> &'a CurveParameterRange2 {
        self.source.parameter_domain()
    }

    /// Whether traversal runs from the range's end to its start.
    pub fn is_reversed(&self) -> bool {
        self.source.source_traversal_is_reversed()
    }

    /// Borrows the original curve, including its selected endpoint evidence.
    pub const fn source(&self) -> &'a Curve2 {
        self.source
    }
}

impl Curve2 {
    /// Borrows a procedural parallel's source, range and traversal definition.
    ///
    /// Analytic selected-fiber subdivisions retain the same structural view.
    /// Other carriers return `None`; this is not a geometric equivalence test.
    /// For untrusted replay with Bezier parameters, pass the retained parallel
    /// and oriented range to [`Self::try_analytic_parallel`], which
    /// rechecks poles, undefined normals and unsplit interior cusps. The view
    /// is not a serialized validation certificate.
    pub fn analytic_parallel(&self) -> Option<CurveAnalyticParallel2<'_>> {
        let parallel = match self.retained_fragment()? {
            BezierSplitFragment2::AnalyticParallel(fragment) => fragment.parallel(),
            BezierSplitFragment2::SelectedFiber(fragment) => fragment.analytic_parallel()?,
            _ => return None,
        };
        Some(CurveAnalyticParallel2 {
            source: self,
            parallel,
        })
    }
}
