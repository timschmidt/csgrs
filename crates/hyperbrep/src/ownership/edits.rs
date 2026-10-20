//! Transactions and owned split results preserve model ownership.

use super::*;

/// A transaction bound to its source model revision.
///
/// Every input reference is checked before staging a mutation. Commit creates
/// a new revision; failure leaves the source model untouched.
#[derive(Clone, Debug)]
pub struct Edit {
    pub(super) source: Model,
    pub(super) inner: ArenaEdit,
}

impl Edit {
    /// Returns the immutable source revision.
    pub fn source(&self) -> &Model {
        &self.source
    }

    /// Stages transform after checking every reference owner.
    pub fn transform(&mut self, transform: &Matrix4) -> Result<&mut Self, EditError> {
        self.inner.transform(transform)?;
        Ok(self)
    }

    /// Stages replace vertex after checking every reference owner.
    pub fn replace_vertex(
        &mut self,
        id: VertexRef<'_>,
        point: GeometricPoint3,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Vertex)
            .map_err(EditError::Reference)?;
        self.inner.replace_vertex(id.id, point)?;
        Ok(self)
    }

    /// Stages replace curve after checking every reference owner.
    pub fn replace_curve(
        &mut self,
        id: Curve3Ref<'_>,
        curve: Curve3,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Curve3)
            .map_err(EditError::Reference)?;
        self.inner.replace_curve(id.id, curve)?;
        Ok(self)
    }

    /// Stages replace pcurve after checking every reference owner.
    pub fn replace_pcurve(
        &mut self,
        id: PcurveRef<'_>,
        pcurve: Pcurve,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Pcurve)
            .map_err(EditError::Reference)?;
        self.inner.replace_pcurve(id.id, pcurve)?;
        Ok(self)
    }

    /// Stages replace surface after checking every reference owner.
    pub fn replace_surface(
        &mut self,
        id: SurfaceRef<'_>,
        surface: Surface,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Surface)
            .map_err(EditError::Reference)?;
        self.inner.replace_surface(id.id, surface)?;
        Ok(self)
    }

    /// Stages replace edge after checking every reference owner.
    pub fn replace_edge(
        &mut self,
        id: EdgeRef<'_>,
        start: VertexRef<'_>,
        end: VertexRef<'_>,
        curve: Curve3Ref<'_>,
        domain: ParameterRange,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Edge)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(start.model, EntityKind::Vertex)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(end.model, EntityKind::Vertex)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(curve.model, EntityKind::Curve3)
            .map_err(EditError::Reference)?;
        self.inner
            .replace_edge(id.id, start.id, end.id, curve.id, domain)?;
        Ok(self)
    }

    /// Stages replace edge use after checking every reference owner.
    pub fn replace_edge_use(
        &mut self,
        id: EdgeUseRef<'_>,
        edge: EdgeRef<'_>,
        direction: Direction,
        pcurve: PcurveRef<'_>,
        parameter_correspondence: ParameterCorrespondence,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::EdgeUse)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(edge.model, EntityKind::Edge)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(pcurve.model, EntityKind::Pcurve)
            .map_err(EditError::Reference)?;
        self.inner.replace_edge_use(
            id.id,
            edge.id,
            direction,
            pcurve.id,
            parameter_correspondence,
        )?;
        Ok(self)
    }

    /// Stages replace wire after checking every reference owner.
    pub fn replace_wire(
        &mut self,
        id: WireRef<'_>,
        edge_uses: &[EdgeUseRef<'_>],
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Wire)
            .map_err(EditError::Reference)?;
        for item in edge_uses {
            self.source
                .require_revision(item.model, EntityKind::EdgeUse)
                .map_err(EditError::Reference)?;
        }
        self.inner
            .replace_wire(id.id, edge_uses.iter().map(|item| item.id).collect())?;
        Ok(self)
    }

    /// Stages replace face after checking every reference owner.
    pub fn replace_face(
        &mut self,
        id: FaceRef<'_>,
        surface: SurfaceRef<'_>,
        orientation: Orientation,
        outer: WireRef<'_>,
        inner: &[WireRef<'_>],
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Face)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(surface.model, EntityKind::Surface)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(outer.model, EntityKind::Wire)
            .map_err(EditError::Reference)?;
        for item in inner {
            self.source
                .require_revision(item.model, EntityKind::Wire)
                .map_err(EditError::Reference)?;
        }
        self.inner.replace_face(
            id.id,
            surface.id,
            orientation,
            outer.id,
            inner.iter().map(|item| item.id).collect(),
        )?;
        Ok(self)
    }

    /// Stages replace whole face after checking every reference owner.
    pub fn replace_whole_face(
        &mut self,
        id: FaceRef<'_>,
        surface: SurfaceRef<'_>,
        orientation: Orientation,
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Face)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(surface.model, EntityKind::Surface)
            .map_err(EditError::Reference)?;
        self.inner
            .replace_whole_face(id.id, surface.id, orientation)?;
        Ok(self)
    }

    /// Stages replace shell after checking every reference owner.
    pub fn replace_shell(
        &mut self,
        id: ShellRef<'_>,
        faces: &[FaceRef<'_>],
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Shell)
            .map_err(EditError::Reference)?;
        for item in faces {
            self.source
                .require_revision(item.model, EntityKind::Face)
                .map_err(EditError::Reference)?;
        }
        self.inner
            .replace_shell(id.id, faces.iter().map(|item| item.id).collect())?;
        Ok(self)
    }

    /// Stages replace solid after checking every reference owner.
    pub fn replace_solid(
        &mut self,
        id: SolidRef<'_>,
        outer: ShellRef<'_>,
        voids: &[ShellRef<'_>],
    ) -> Result<&mut Self, EditError> {
        self.source
            .require_revision(id.model, EntityKind::Solid)
            .map_err(EditError::Reference)?;
        self.source
            .require_revision(outer.model, EntityKind::Shell)
            .map_err(EditError::Reference)?;
        for item in voids {
            self.source
                .require_revision(item.model, EntityKind::Shell)
                .map_err(EditError::Reference)?;
        }
        self.inner
            .replace_solid(id.id, outer.id, voids.iter().map(|item| item.id).collect())?;
        Ok(self)
    }

    /// Validates staged changes and publishes an independent revision.
    pub fn commit(self) -> Result<Model, EditError> {
        self.inner.commit().map(Model::from_arena)
    }
}
