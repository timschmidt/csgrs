//! Typed arena identifiers.

macro_rules! model_id {
    ($name:ident, $description:literal) => {
        #[doc = $description]
        #[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(pub(super) u32);

        impl $name {
            pub(crate) fn from_index(index: usize) -> Option<Self> {
                u32::try_from(index).ok().map(Self)
            }

            /// Returns the deterministic arena index for this model-local ID.
            pub const fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

model_id!(VertexId, "Model-local vertex identifier.");
model_id!(Curve3Id, "Model-local spatial-curve identifier.");
model_id!(PcurveId, "Model-local surface-parameter curve identifier.");
model_id!(SurfaceId, "Model-local surface identifier.");
model_id!(EdgeId, "Model-local edge identifier.");
model_id!(EdgeUseId, "Model-local oriented edge-use identifier.");
model_id!(WireId, "Model-local wire identifier.");
model_id!(FaceId, "Model-local face identifier.");
model_id!(ShellId, "Model-local shell identifier.");
model_id!(SolidId, "Model-local solid identifier.");
