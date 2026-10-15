//! Transactional model replacements and commit.

use crate::topology::*;

impl Edit {
    /// Applies an orientation-preserving invertible affine transform in place.
    ///
    /// A failed transform leaves the previously staged snapshot untouched.
    pub fn transform(&mut self, transform: &Matrix4) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let transformed = self.staged.transformed(transform)?;
            self.staged = transformed;
            Ok(self)
        })
    }

    /// Replaces one exact vertex point while retaining its stable ID.
    pub fn replace_vertex(&mut self, id: VertexId, point: Point3) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(vertex) = data.vertices.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Vertex,
                    index: id.index(),
                });
            };
            vertex.point = point;
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one spatial curve while retaining its stable ID.
    pub fn replace_curve(&mut self, id: Curve3Id, curve: Curve3) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.curves.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Curve3,
                    index: id.index(),
                });
            };
            *slot = curve;
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one face-local pcurve while retaining its stable ID.
    pub fn replace_pcurve(&mut self, id: PcurveId, pcurve: Pcurve) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.pcurves.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Pcurve,
                    index: id.index(),
                });
            };
            *slot = pcurve;
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one support surface while retaining its stable ID.
    pub fn replace_surface(
        &mut self,
        id: SurfaceId,
        surface: Surface,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.surfaces.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Surface,
                    index: id.index(),
                });
            };
            *slot = surface;
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one edge's topology, carrier, and retained domain.
    pub fn replace_edge(
        &mut self,
        id: EdgeId,
        start: VertexId,
        end: VertexId,
        curve: Curve3Id,
        domain: ParameterDomain,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.edges.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Edge,
                    index: id.index(),
                });
            };
            *slot = Edge {
                start,
                end,
                curve,
                domain,
                endpoint_certificate: false,
            };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one oriented edge use and its exact parameter correspondence.
    pub fn replace_edge_use(
        &mut self,
        id: EdgeUseId,
        edge: EdgeId,
        direction: Direction,
        pcurve: PcurveId,
        parameter_correspondence: ParameterCorrespondence,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.edge_uses.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::EdgeUse,
                    index: id.index(),
                });
            };
            *slot = EdgeUse {
                edge,
                direction,
                pcurve,
                parameter_correspondence,
                image_certificate: false,
            };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one wire's ordered edge-use sequence.
    pub fn replace_wire(
        &mut self,
        id: WireId,
        edge_uses: Vec<EdgeUseId>,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.wires.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Wire,
                    index: id.index(),
                });
            };
            *slot = Wire {
                edge_uses,
                simplicity_certificate: false,
            };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one face's surface, orientation, and boundary wires.
    pub fn replace_face(
        &mut self,
        id: FaceId,
        surface: SurfaceId,
        orientation: Orientation,
        outer: WireId,
        inner: Vec<WireId>,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.faces.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Face,
                    index: id.index(),
                });
            };
            *slot = Face {
                surface,
                orientation,
                boundary: FaceBoundary::Trimmed { outer, inner },
            };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one face with the complete closed support surface.
    pub fn replace_whole_face(
        &mut self,
        id: FaceId,
        surface: SurfaceId,
        orientation: Orientation,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.faces.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Face,
                    index: id.index(),
                });
            };
            *slot = Face {
                surface,
                orientation,
                boundary: FaceBoundary::WholeSurface,
            };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one shell's ordered face set.
    pub fn replace_shell(
        &mut self,
        id: ShellId,
        faces: Vec<FaceId>,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.shells.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Shell,
                    index: id.index(),
                });
            };
            *slot = Shell { faces };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Replaces one solid's outer and void shells.
    pub fn replace_solid(
        &mut self,
        id: SolidId,
        outer: ShellId,
        voids: Vec<ShellId>,
    ) -> Result<&mut Self, EditError> {
        crate::context::strict(|| {
            let data = Arc::make_mut(&mut self.staged.data);
            let Some(slot) = data.solids.get_mut(id.index()) else {
                return Err(EditError::InvalidReference {
                    kind: EntityKind::Solid,
                    index: id.index(),
                });
            };
            *slot = Solid { outer, voids };
            invalidate_model_certificates(data);
            reset_model_caches(data);
            Ok(self)
        })
    }

    /// Commits this transaction as a new immutable model.
    pub fn commit(self) -> Result<Model, EditError> {
        crate::context::strict(|| self.staged.revalidated().map_err(EditError::Validation))
    }
}
