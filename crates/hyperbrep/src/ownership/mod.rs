//! Model-bound public topology and construction ownership.

use std::sync::Arc;

use crate::*;

pub mod boolean;
pub mod builder;
mod changes;
mod edits;
mod references;
mod staging;
#[cfg(feature = "tessellation")]
pub mod tessellation;

pub use changes::*;
pub use edits::*;
pub use references::*;
pub use staging::*;

#[derive(Debug)]
struct ConstructionScope;

/// An immutable, validated model revision.
///
/// Clones share one revision. Transforms, committed edits and decoded models
/// publish independent revisions. Entity references borrow their model and
/// cannot outlive it. Arena positions are never accepted as public references.
///
/// ```compile_fail,E0597
/// use hyperbrep::{ModelBuilder, Point3};
/// let vertex = {
///     let mut builder = ModelBuilder::new();
///     builder.vertex(Point3::origin().into()).unwrap();
///     let model = builder.finish().unwrap();
///     model.vertices().next().unwrap()
/// };
/// let point = vertex.point(); // the model no longer exists
/// ```
///
/// ```compile_fail,E0308
/// use hyperbrep::{ModelBuilder, Point3};
/// let mut builder = ModelBuilder::new();
/// builder.vertex(Point3::origin().into()).unwrap();
/// let model = builder.finish().unwrap();
/// let index = model.vertices().next().unwrap().index();
/// let vertex = model.vertex(index); // an arena position is not a construction key
/// ```
#[derive(Clone, Debug)]
pub struct Model {
    inner: ArenaModel,
    construction: Option<Arc<ConstructionScope>>,
}

impl Model {
    fn from_arena(inner: ArenaModel) -> Self {
        Self {
            inner,
            construction: None,
        }
    }

    /// Whether two values share the same immutable revision.
    ///
    /// This tests ownership identity, not geometric equality.
    pub fn same_revision(&self, other: &Self) -> bool {
        self.inner.same_snapshot(&other.inner)
    }

    fn require_revision(
        &self,
        other: &Self,
        kind: EntityKind,
    ) -> Result<(), GeometryReferenceError> {
        if self.same_revision(other) {
            Ok(())
        } else {
            Err(GeometryReferenceError::ForeignModel { kind })
        }
    }

    /// Returns arena counts without scanning topology.
    pub fn counts(&self) -> ModelCounts {
        self.inner.counts()
    }

    /// Returns certified bounds, retained on the immutable revision.
    pub fn bounds(&self) -> Result<Option<Aabb>, GeometryError> {
        self.inner.bounds()
    }

    /// Transforms this model into an independent validated revision.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        self.inner.transformed(transform).map(Self::from_arena)
    }

    /// Begins a transaction whose targets must belong to this revision.
    pub fn edit(&self) -> Edit {
        Edit {
            source: self.clone(),
            inner: self.inner.edit(),
        }
    }

    /// Encodes the exact model into an untrusted archival carrier.
    pub fn to_raw(&self) -> Result<RawModel, PersistenceError> {
        self.inner.to_raw().map(|inner| RawModel { inner })
    }

    /// Encodes deterministic exact JSON. Runtime identity is not serialized.
    pub fn to_json(&self) -> Result<String, PersistenceError> {
        self.inner.to_json()
    }
}

/// An owned selection of validated volume bodies, including the empty set.
#[derive(Clone, Debug)]
pub struct BodySet {
    model: Model,
    solids: Vec<SolidId>,
}

impl BodySet {
    fn from_parts(model: ArenaModel, solids: Vec<SolidId>) -> Self {
        Self {
            model: Model::from_arena(model),
            solids,
        }
    }

    /// Borrows the immutable model containing these bodies.
    pub fn model(&self) -> &Model {
        &self.model
    }

    /// Iterates over selected solids in deterministic result order.
    pub fn bodies(&self) -> impl ExactSizeIterator<Item = SolidRef<'_>> {
        self.solids.iter().map(|&id| SolidRef {
            model: &self.model,
            id,
        })
    }

    /// Returns the number of selected connected bodies.
    pub fn len(&self) -> usize {
        self.solids.len()
    }

    /// Whether the result has no volume bodies.
    pub fn is_empty(&self) -> bool {
        self.solids.is_empty()
    }

    /// Consumes the selection and returns its model.
    pub fn into_model(self) -> Model {
        self.model
    }

    /// Transforms the selected bodies into a new revision.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        Ok(Self {
            model: self.model.transformed(transform)?,
            solids: self.solids.clone(),
        })
    }
}

/// An owned selection of surface faces without a claim of enclosed volume.
#[derive(Clone, Debug)]
pub struct SheetSet {
    model: Model,
    faces: Vec<FaceId>,
}

impl SheetSet {
    fn from_parts(model: ArenaModel, faces: Vec<FaceId>) -> Self {
        Self {
            model: Model::from_arena(model),
            faces,
        }
    }

    /// Borrows the immutable model containing these sheets.
    pub fn model(&self) -> &Model {
        &self.model
    }

    /// Iterates over selected faces in deterministic result order.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = FaceRef<'_>> {
        self.faces.iter().map(|&id| FaceRef {
            model: &self.model,
            id,
        })
    }

    /// Consumes the selection and returns its model.
    pub fn into_model(self) -> Model {
        self.model
    }

    /// Transforms the selected sheets into a new revision.
    pub fn transformed(&self, transform: &Matrix4) -> Result<Self, GeometryError> {
        Ok(Self {
            model: self.model.transformed(transform)?,
            faces: self.faces.clone(),
        })
    }
}

/// Untrusted exact archive, with no modeling or measurement operations.
#[derive(Clone, Debug)]
pub struct RawModel {
    inner: ImportedModel,
}

impl RawModel {
    /// Parses JSON without asserting topology validity.
    pub fn from_json(json: &str) -> Result<Self, PersistenceError> {
        ImportedModel::from_json(json).map(|inner| Self { inner })
    }

    /// Encodes the untrusted archive deterministically.
    pub fn to_json(&self) -> Result<String, PersistenceError> {
        self.inner.to_json()
    }

    /// Replays validation and publishes an independent model revision.
    pub fn validate(self) -> Result<Model, PersistenceError> {
        self.inner.validate().map(Model::from_arena)
    }
}
