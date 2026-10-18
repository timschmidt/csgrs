//! Complete native images of retained curves, without changing their authority.

use std::borrow::Cow;

use crate::bezier_region::represented_native_subcurve;
use crate::policy::resolve_certified_operation;
use crate::{
    Classification, Curve2, CurveContext, CurveGeometry2, CurveOperation2, ExactCurveError,
    ExactCurveResult, Segment2, UncertaintyReason,
};

/// A complete oriented native image, borrowing the curve that proves it.
///
/// The image may use a different parameter chart. A retained Bezier restriction
/// uses the exact affine change from its selected endpoints to `[0, 1]`; a line
/// or circular arc may use its own native chart. The original parameters and
/// selected roots remain available from [`Self::source`]. This is an exact
/// image proof, not a simplicity certificate or a replacement for source history.
/// It does not certify the intent of an earlier approximate construction.
#[derive(Clone, Debug)]
pub struct CurveNativeImage2<'a> {
    source: &'a Curve2,
    geometry: Cow<'a, CurveGeometry2>,
}

impl<'a> CurveNativeImage2<'a> {
    /// Borrows the original curve, including its parameter and root evidence.
    pub const fn source(&self) -> &'a Curve2 {
        self.source
    }

    /// Borrows native geometry covering the complete source in traversal order.
    pub fn geometry(&self) -> &CurveGeometry2 {
        &self.geometry
    }

    /// Takes the native image in its own chart, leaving the source unchanged.
    pub fn into_geometry(self) -> CurveGeometry2 {
        self.geometry.into_owned()
    }
}

impl Curve2 {
    /// Proves a complete native image without replacing selected-root authority.
    ///
    /// Stored native geometry is borrowed directly. Other curves first reuse
    /// [`Self::native_segment`], preserving its line/arc traversal proof. A
    /// retained single Bezier span can also supply its exact native subcurve
    /// when its endpoints have represented scalar views. Selected quadratic
    /// roots replay their defining polynomial and isolator; no approximation
    /// or persistent root-cache mutation is used.
    ///
    /// A nonlinear or nonmonotone Bezier keeps its entire parameterized image,
    /// rather than a chord between its endpoints. Multiple retained spans are
    /// not combined. Unsupported views and unresolved proof work remain typed;
    /// the query is strict even inside an approximate ambient scope.
    pub fn native_image(&self) -> ExactCurveResult<CurveNativeImage2<'_>> {
        if let Some(geometry) = self.geometry() {
            return Ok(CurveNativeImage2 {
                source: self,
                geometry: Cow::Borrowed(geometry),
            });
        }
        let geometry = resolve_certified_operation(&CurveContext::STRICT, |policy| {
            match self.native_segment() {
                Ok(image) => {
                    return Ok(match image.segment() {
                        Segment2::Line(line) => CurveGeometry2::Line(line.clone()),
                        Segment2::Arc(arc) => CurveGeometry2::CircularArc(arc.clone()),
                    });
                }
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Unsupported => {}
                Err(error) => return Err(error),
            }
            let fragment = if let Some(fragment) = self.retained_fragment() {
                fragment
            } else if let Some(spans) =
                self.restricted_source_spans(policy, CurveOperation2::NativeTopology)?
            {
                let [span] = spans else {
                    return Err(self.native_image_blocker(UncertaintyReason::Unsupported));
                };
                &span.fragment
            } else {
                return Err(self.native_image_blocker(UncertaintyReason::Unsupported));
            };
            match represented_native_subcurve(fragment, policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::NativeTopology, self.family(), cause)
            })? {
                Classification::Decided(Some(curve)) => Ok(CurveGeometry2::from_bezier(curve)),
                Classification::Decided(None) => {
                    Err(self.native_image_blocker(UncertaintyReason::Unsupported))
                }
                Classification::Uncertain(reason) => Err(self.native_image_blocker(reason)),
            }
        })?
        .into_value();
        Ok(CurveNativeImage2 {
            source: self,
            geometry: Cow::Owned(geometry),
        })
    }

    fn native_image_blocker(&self, reason: UncertaintyReason) -> ExactCurveError {
        ExactCurveError::blocked(CurveOperation2::NativeTopology, self.family(), reason)
    }
}
