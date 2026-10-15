//! Compact internal slots, also exposed as non-resolving diagnostic labels.

macro_rules! model_id {
    ($name:ident, $description:literal) => {
        #[doc = $description]
        ///
        /// This is an arena position used in diagnostics, not a model-bound
        /// reference. Public modeling APIs accept borrowed entity references
        /// or construction keys; they never resolve a position by itself.
        #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(pub(super) u32);

        impl $name {
            pub(crate) fn from_index(index: usize) -> Option<Self> {
                u32::try_from(index).ok().map(Self)
            }

            /// Returns the deterministic position for diagnostics.
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

model_id!(VertexId, "Diagnostic vertex position.");
model_id!(Curve3Id, "Diagnostic spatial-curve position.");
model_id!(PcurveId, "Diagnostic surface-parameter curve position.");
model_id!(SurfaceId, "Diagnostic surface position.");
model_id!(EdgeId, "Diagnostic edge position.");
model_id!(EdgeUseId, "Diagnostic oriented edge-use position.");
model_id!(WireId, "Diagnostic wire position.");
model_id!(FaceId, "Diagnostic face position.");
model_id!(ShellId, "Diagnostic shell position.");
model_id!(SolidId, "Diagnostic solid position.");
