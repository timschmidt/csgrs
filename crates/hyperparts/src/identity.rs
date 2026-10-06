//! Stable identifiers for the part graph.

use crate::{PartsError, PartsResult};

macro_rules! id_type {
    ($name:ident) => {
        /// Stable non-empty identifier.
        #[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
        pub struct $name(String);

        impl $name {
            /// Creates a non-empty id.
            pub fn new(value: impl Into<String>) -> PartsResult<Self> {
                let value = value.into();
                if value.is_empty() {
                    return Err(PartsError::EmptyIdentifier);
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

id_type!(PartId);
id_type!(VariantId);
id_type!(RevisionId);
id_type!(TerminalId);
id_type!(ManufacturerPartNumber);
id_type!(InternalPartNumber);
id_type!(SupplierSku);
