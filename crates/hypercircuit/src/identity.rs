//! Stable ids for circuits, nets, branches, and part references.

use crate::{CircuitError, CircuitResult};

macro_rules! id_type {
    ($name:ident) => {
        /// Stable non-empty identifier.
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
