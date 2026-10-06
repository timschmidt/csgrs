//! Body and fixture carriers for exact physical interpretation.

use crate::{ExactMaterial, PhysicsError, PhysicsResult, PhysicsShape3};

/// Stable body identifier.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct BodyId(String);

/// Stable fixture identifier.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct FixtureId(String);

/// Runtime role of a body before any engine adapter is chosen.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyKind {
    /// Immovable body with no integrated velocity state.
    Static,
    /// Externally moved body whose motion is prescribed by the caller.
    Kinematic,
    /// Body whose motion may be integrated by a runtime adapter.
    Dynamic,
}

/// A shape/material association owned by a body.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactFixture3 {
    id: FixtureId,
    shape: PhysicsShape3,
    material: ExactMaterial,
}

/// Exact body description before export to any approximate engine.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBody3 {
    id: BodyId,
    kind: BodyKind,
    fixtures: Vec<ExactFixture3>,
}

impl BodyId {
    /// Creates a non-empty body id.
    pub fn new(id: impl Into<String>) -> PhysicsResult<Self> {
        let id = id.into();
        if id.is_empty() {
            return Err(PhysicsError::EmptyIdentifier);
        }
        Ok(Self(id))
    }

    /// Returns the identifier text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl FixtureId {
    /// Creates a non-empty fixture id.
    pub fn new(id: impl Into<String>) -> PhysicsResult<Self> {
        let id = id.into();
        if id.is_empty() {
            return Err(PhysicsError::EmptyIdentifier);
        }
        Ok(Self(id))
    }

    /// Returns the identifier text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl ExactFixture3 {
    /// Creates an exact fixture with a shape and material.
    pub const fn new(id: FixtureId, shape: PhysicsShape3, material: ExactMaterial) -> Self {
        Self {
            id,
            shape,
            material,
        }
    }

    /// Returns the fixture id.
    pub const fn id(&self) -> &FixtureId {
        &self.id
    }

    /// Returns the fixture shape.
    pub const fn shape(&self) -> &PhysicsShape3 {
        &self.shape
    }

    /// Returns the fixture material.
    pub const fn material(&self) -> &ExactMaterial {
        &self.material
    }
}

impl ExactBody3 {
    /// Creates a body with its exact fixtures.
    pub fn new(id: BodyId, kind: BodyKind, fixtures: Vec<ExactFixture3>) -> Self {
        Self { id, kind, fixtures }
    }

    /// Returns the body id.
    pub const fn id(&self) -> &BodyId {
        &self.id
    }

    /// Returns the body role.
    pub const fn kind(&self) -> BodyKind {
        self.kind
    }

    /// Returns all fixtures attached to this body.
    pub fn fixtures(&self) -> &[ExactFixture3] {
        &self.fixtures
    }
}
