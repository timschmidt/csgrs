//! Stable ids for circuits, nets, branches, and part references.

use crate::{CircuitError, CircuitResult};

macro_rules! id_type {
    ($name:ident) => {
        /// Stable non-empty identifier.
        #[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
        #[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(String);

        impl $name {
            /// Creates a non-empty id.
            pub fn new(value: impl Into<String>) -> CircuitResult<Self> {
                let value = value.into();
                if value.is_empty() {
                    return Err(CircuitError::EmptyIdentifier);
                }
                Ok(Self(value))
            }

            /// Returns the id text.
            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(NetId);
id_type!(BranchId);
id_type!(ComponentId);
id_type!(PartRef);
id_type!(CircuitId);
id_type!(CircuitInstanceId);
id_type!(PinRef);
id_type!(DeviceModelId);
id_type!(BusId);
id_type!(BusSliceId);
id_type!(PortId);
id_type!(BoardId);
id_type!(LandPatternId);
id_type!(LandPatternGraphicId);
id_type!(PadId);
id_type!(RouteId);
id_type!(ViaId);
id_type!(ZoneId);
id_type!(KeepoutId);
id_type!(NetClassId);
id_type!(ViaStyleId);
id_type!(DifferentialPairId);
id_type!(SubcircuitInstanceId);
id_type!(SchematicSymbolDefinitionId);
id_type!(SchematicSymbolId);
id_type!(SchematicWireId);
id_type!(SchematicLabelId);
id_type!(SchematicSheetId);
id_type!(SchematicSheetPortId);
id_type!(SchematicSheetLinkId);
id_type!(CircuitPackageName);
id_type!(AssemblyVariantId);
id_type!(PlacementConstraintId);
id_type!(DesignEditId);
id_type!(LayoutModuleId);
id_type!(PlacementGroupId);
id_type!(RouteConstraintRegionId);
id_type!(RouteRuleRegionId);
id_type!(EscapePolicyId);
id_type!(LengthTuningPatternId);
id_type!(PhaseTuningGroupId);
