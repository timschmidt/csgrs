//! Construction keys are scoped to one builder and its published revision.

use super::*;

trait ConstructionKey {
    type Id: Copy;
    const KIND: EntityKind;
    fn scope(&self) -> &Arc<ConstructionScope>;
    fn id(&self) -> Self::Id;
}

macro_rules! construction_key {
    ($key:ident, $id:ident, $kind:ident) => {
        /// A typed key issued by one model builder.
        ///
        /// A key can be reused within its builder and resolved in the model
        /// published by that builder. It cannot reference another revision.
        #[derive(Clone, Debug)]
        pub struct $key {
            pub(super) scope: Arc<ConstructionScope>,
            pub(super) id: $id,
        }
        impl ConstructionKey for $key {
            type Id = $id;
            const KIND: EntityKind = EntityKind::$kind;
            fn scope(&self) -> &Arc<ConstructionScope> {
                &self.scope
            }
            fn id(&self) -> Self::Id {
                self.id
            }
        }
    };
}

construction_key!(VertexKey, VertexId, Vertex);
construction_key!(Curve3Key, Curve3Id, Curve3);
construction_key!(PcurveKey, PcurveId, Pcurve);
construction_key!(SurfaceKey, SurfaceId, Surface);
construction_key!(EdgeKey, EdgeId, Edge);
construction_key!(EdgeUseKey, EdgeUseId, EdgeUse);
construction_key!(WireKey, WireId, Wire);
construction_key!(FaceKey, FaceId, Face);
construction_key!(ShellKey, ShellId, Shell);
construction_key!(SolidKey, SolidId, Solid);

/// Stages exact topology with keys that cannot cross builder boundaries.
///
/// Foreign keys are rejected before any staging mutation. Finishing consumes
/// the builder and publishes an immutable model after complete validation.
#[derive(Debug)]
pub struct ModelBuilder {
    inner: ArenaModelBuilder,
    scope: Arc<ConstructionScope>,
}

impl Default for ModelBuilder {
    fn default() -> Self {
        Self {
            inner: ArenaModelBuilder::new(),
            scope: Arc::new(ConstructionScope),
        }
    }
}

impl ModelBuilder {
    /// Starts an independent construction scope.
    pub fn new() -> Self {
        Self::default()
    }

    fn resolve<K: ConstructionKey>(&self, key: &K) -> Result<K::Id, BuildError> {
        if Arc::ptr_eq(&self.scope, key.scope()) {
            Ok(key.id())
        } else {
            Err(BuildError::Reference(
                GeometryReferenceError::ForeignBuilder { kind: K::KIND },
            ))
        }
    }

    /// Adds an exact vertex.
    pub fn vertex(&mut self, point: GeometricPoint3) -> Result<VertexKey, BuildError> {
        self.inner.vertex(point).map(|id| VertexKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds an immutable spatial curve.
    pub fn curve(&mut self, curve: Curve3) -> Result<Curve3Key, BuildError> {
        self.inner.curve(curve).map(|id| Curve3Key {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds an immutable face-local pcurve.
    pub fn pcurve(&mut self, pcurve: Pcurve) -> Result<PcurveKey, BuildError> {
        self.inner.pcurve(pcurve).map(|id| PcurveKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds an immutable support surface.
    pub fn surface(&mut self, surface: Surface) -> Result<SurfaceKey, BuildError> {
        self.inner.surface(surface).map(|id| SurfaceKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds an edge after checking all key owners and exact endpoints.
    ///
    /// A circle or ellipse spanning exactly one period may use the same vertex
    /// at both endpoints. Other closed curve families require a future
    /// regularity certificate and return [`BuildError::UnsupportedClosedEdge`].
    pub fn edge(
        &mut self,
        start: &VertexKey,
        end: &VertexKey,
        curve: &Curve3Key,
        domain: ParameterRange,
    ) -> Result<EdgeKey, BuildError> {
        let (start, end, curve) = (
            self.resolve(start)?,
            self.resolve(end)?,
            self.resolve(curve)?,
        );
        self.inner
            .edge(start, end, curve, domain)
            .map(|id| EdgeKey {
                scope: self.scope.clone(),
                id,
            })
    }
    /// Adds a directed edge use with an exact parameter correspondence.
    pub fn edge_use(
        &mut self,
        edge: &EdgeKey,
        direction: Direction,
        pcurve: &PcurveKey,
        correspondence: ParameterCorrespondence,
    ) -> Result<EdgeUseKey, BuildError> {
        let (edge, pcurve) = (self.resolve(edge)?, self.resolve(pcurve)?);
        self.inner
            .edge_use(edge, direction, pcurve, correspondence)
            .map(|id| EdgeUseKey {
                scope: self.scope.clone(),
                id,
            })
    }
    /// Adds an ordered, connected, closed wire.
    pub fn wire(&mut self, edge_uses: &[EdgeUseKey]) -> Result<WireKey, BuildError> {
        let edge_uses = edge_uses
            .iter()
            .map(|key| self.resolve(key))
            .collect::<Result<Vec<_>, _>>()?;
        self.inner.wire(edge_uses).map(|id| WireKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds a trimmed oriented face.
    pub fn face(
        &mut self,
        surface: &SurfaceKey,
        orientation: Orientation,
        outer: &WireKey,
        inner: &[WireKey],
    ) -> Result<FaceKey, BuildError> {
        let (surface, outer) = (self.resolve(surface)?, self.resolve(outer)?);
        let inner = inner
            .iter()
            .map(|key| self.resolve(key))
            .collect::<Result<Vec<_>, _>>()?;
        self.inner
            .face(surface, orientation, outer, inner)
            .map(|id| FaceKey {
                scope: self.scope.clone(),
                id,
            })
    }
    /// Adds a face covering a supported complete closed surface.
    pub fn whole_face(
        &mut self,
        surface: &SurfaceKey,
        orientation: Orientation,
    ) -> Result<FaceKey, BuildError> {
        let surface = self.resolve(surface)?;
        self.inner
            .whole_face(surface, orientation)
            .map(|id| FaceKey {
                scope: self.scope.clone(),
                id,
            })
    }
    /// Adds an edge-connected oriented shell.
    pub fn shell(&mut self, faces: &[FaceKey]) -> Result<ShellKey, BuildError> {
        let faces = faces
            .iter()
            .map(|key| self.resolve(key))
            .collect::<Result<Vec<_>, _>>()?;
        self.inner.shell(faces).map(|id| ShellKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Adds a validated material region with optional cavity shells.
    pub fn solid(&mut self, outer: &ShellKey, voids: &[ShellKey]) -> Result<SolidKey, BuildError> {
        let outer = self.resolve(outer)?;
        let voids = voids
            .iter()
            .map(|key| self.resolve(key))
            .collect::<Result<Vec<_>, _>>()?;
        self.inner.solid(outer, voids).map(|id| SolidKey {
            scope: self.scope.clone(),
            id,
        })
    }
    /// Validates every local and global invariant and publishes one revision.
    pub fn finish(self) -> Result<Model, ValidationReport> {
        self.inner.finish().map(|inner| Model {
            inner,
            construction: Some(self.scope),
        })
    }
}
