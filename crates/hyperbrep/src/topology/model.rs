//! Exact model model operations.

use crate::topology::*;

impl ArenaModel {
    pub(crate) fn same_snapshot(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

    pub(crate) fn snapshot_address(&self) -> usize {
        Arc::as_ptr(&self.data) as usize
    }

    /// Returns record counts without scanning the model.
    pub fn counts(&self) -> ModelCounts {
        ModelCounts {
            vertices: self.data.vertices.len(),
            curves: self.data.curves.len(),
            pcurves: self.data.pcurves.len(),
            surfaces: self.data.surfaces.len(),
            edges: self.data.edges.len(),
            edge_uses: self.data.edge_uses.len(),
            wires: self.data.wires.len(),
            faces: self.data.faces.len(),
            shells: self.data.shells.len(),
            solids: self.data.solids.len(),
        }
    }

    /// Starts a transaction over an immutable shared snapshot.
    pub fn edit(&self) -> ArenaEdit {
        ArenaEdit {
            staged: self.clone(),
        }
    }

    pub(in crate::topology) fn revalidated(&self) -> Result<Self, ValidationReport> {
        let mut builder = ArenaModelBuilder::new();
        self.append_to_builder(&mut builder)
            .map_err(single_validation_error)?;
        builder.finish()
    }

    pub(crate) fn append_to_builder(
        &self,
        builder: &mut ArenaModelBuilder,
    ) -> Result<Vec<SolidId>, BuildError> {
        let vertices = self
            .data
            .vertices
            .iter()
            .map(|vertex| builder.vertex(vertex.point.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let curves = self
            .data
            .curves
            .iter()
            .map(|curve| builder.curve(curve.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let pcurves = self
            .data
            .pcurves
            .iter()
            .map(|pcurve| builder.pcurve(pcurve.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let surfaces = self
            .data
            .surfaces
            .iter()
            .map(|surface| builder.surface(surface.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        let edges = self
            .data
            .edges
            .iter()
            .map(|edge| {
                builder.edge_with_endpoint_certificate(
                    vertices[edge.start.index()],
                    vertices[edge.end.index()],
                    curves[edge.curve.index()],
                    edge.domain.clone(),
                    edge.endpoint_certificate,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let edge_uses = self
            .data
            .edge_uses
            .iter()
            .map(|edge_use| {
                builder.edge_use_with_image_certificate(
                    edges[edge_use.edge.index()],
                    edge_use.direction,
                    pcurves[edge_use.pcurve.index()],
                    edge_use.parameter_correspondence.clone(),
                    edge_use.image_certificate,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let wires = self
            .data
            .wires
            .iter()
            .map(|wire| {
                builder.wire_with_simplicity_certificate(
                    wire.edge_uses
                        .iter()
                        .map(|edge_use| edge_uses[edge_use.index()])
                        .collect(),
                    wire.simplicity_certificate,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        let faces = self
            .data
            .faces
            .iter()
            .map(|face| match &face.boundary {
                FaceBoundary::WholeSurface => {
                    builder.whole_face(surfaces[face.surface.index()], face.orientation)
                }
                FaceBoundary::Trimmed { outer, inner } => builder.face(
                    surfaces[face.surface.index()],
                    face.orientation,
                    wires[outer.index()],
                    inner.iter().map(|wire| wires[wire.index()]).collect(),
                ),
            })
            .collect::<Result<Vec<_>, _>>()?;
        let shells = self
            .data
            .shells
            .iter()
            .map(|shell| {
                builder.shell(shell.faces.iter().map(|face| faces[face.index()]).collect())
            })
            .collect::<Result<Vec<_>, _>>()?;
        self.data
            .solids
            .iter()
            .enumerate()
            .map(|(index, solid)| {
                let outer = shells[solid.outer.index()];
                let voids = solid
                    .voids
                    .iter()
                    .map(|shell| shells[shell.index()])
                    .collect();
                if self.data.solid_certificates_valid[index] {
                    builder.solid_with_retained_certificate(
                        outer,
                        voids,
                        CertifiedSolid {
                            cylinder: self.data.certified_cylinders[index].clone(),
                            sphere: self.data.certified_spheres[index].clone(),
                            sphere_pair: self.data.certified_sphere_pairs[index].clone(),
                            cone_frustum: self.data.certified_cone_frustums[index].clone(),
                            torus: self.data.certified_tori[index].clone(),
                            revolution: self.data.certified_revolutions[index].clone(),
                            loft: self.data.certified_lofts[index].clone(),
                            curve_sweep: self.data.certified_curve_sweeps[index].clone(),
                            prism: self.data.certified_prisms[index].clone(),
                        },
                    )
                } else {
                    builder.solid(outer, voids)
                }
            })
            .collect()
    }

    /// Iterates over vertices in deterministic arena order.
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = (VertexId, &Vertex)> {
        self.data
            .vertices
            .iter()
            .enumerate()
            .map(|(index, record)| (VertexId(index as u32), record))
    }

    /// Iterates over spatial curves in deterministic arena order.
    pub fn curves(&self) -> impl ExactSizeIterator<Item = (Curve3Id, &Curve3)> {
        self.data
            .curves
            .iter()
            .enumerate()
            .map(|(index, record)| (Curve3Id(index as u32), record))
    }

    /// Iterates over pcurves in deterministic arena order.
    pub fn pcurves(&self) -> impl ExactSizeIterator<Item = (PcurveId, &Pcurve)> {
        self.data
            .pcurves
            .iter()
            .enumerate()
            .map(|(index, record)| (PcurveId(index as u32), record))
    }

    /// Iterates over surfaces in deterministic arena order.
    pub fn surfaces(&self) -> impl ExactSizeIterator<Item = (SurfaceId, &Surface)> {
        self.data
            .surfaces
            .iter()
            .enumerate()
            .map(|(index, record)| (SurfaceId(index as u32), record))
    }

    /// Iterates over edges in deterministic arena order.
    pub fn edges(&self) -> impl ExactSizeIterator<Item = (EdgeId, &Edge)> {
        self.data
            .edges
            .iter()
            .enumerate()
            .map(|(index, record)| (EdgeId(index as u32), record))
    }

    /// Iterates over edge uses in deterministic arena order.
    pub fn edge_uses(&self) -> impl ExactSizeIterator<Item = (EdgeUseId, &EdgeUse)> {
        self.data
            .edge_uses
            .iter()
            .enumerate()
            .map(|(index, record)| (EdgeUseId(index as u32), record))
    }

    /// Iterates over wires in deterministic arena order.
    pub fn wires(&self) -> impl ExactSizeIterator<Item = (WireId, &Wire)> {
        self.data
            .wires
            .iter()
            .enumerate()
            .map(|(index, record)| (WireId(index as u32), record))
    }

    /// Iterates over faces in deterministic arena order.
    pub fn faces(&self) -> impl ExactSizeIterator<Item = (FaceId, &Face)> {
        self.data
            .faces
            .iter()
            .enumerate()
            .map(|(index, record)| (FaceId(index as u32), record))
    }

    /// Iterates over shells in deterministic arena order.
    pub fn shells(&self) -> impl ExactSizeIterator<Item = (ShellId, &Shell)> {
        self.data
            .shells
            .iter()
            .enumerate()
            .map(|(index, record)| (ShellId(index as u32), record))
    }

    /// Iterates over solids in deterministic arena order.
    pub fn solids(&self) -> impl ExactSizeIterator<Item = (SolidId, &Solid)> {
        self.data
            .solids
            .iter()
            .enumerate()
            .map(|(index, record)| (SolidId(index as u32), record))
    }

    /// Returns the exact model-space bounds of all canonical vertices.
    ///
    /// The empty model has no bounds. The result is retained after the first
    /// certified coordinate-ordering pass.
    pub fn bounds(&self) -> Result<Option<Aabb>, GeometryError> {
        crate::context::strict(|| {
            self.data
                .bounds
                .get_or_init(|| {
                    compute_bounds(
                        &self.data.vertices,
                        &self.data.curves,
                        &self.data.certified_spheres,
                        &self.data.certified_sphere_pairs,
                    )
                })
                .clone()
        })
    }

    /// Returns a vertex by typed ID.
    pub fn vertex(&self, id: VertexId) -> Option<&Vertex> {
        self.data.vertices.get(id.index())
    }

    /// Returns a spatial curve by typed ID.
    pub fn curve(&self, id: Curve3Id) -> Option<&Curve3> {
        self.data.curves.get(id.index())
    }

    /// Returns a pcurve by typed ID.
    pub fn pcurve(&self, id: PcurveId) -> Option<&Pcurve> {
        self.data.pcurves.get(id.index())
    }

    /// Returns a surface by typed ID.
    pub fn surface(&self, id: SurfaceId) -> Option<&Surface> {
        self.data.surfaces.get(id.index())
    }

    /// Returns an edge by typed ID.
    pub fn edge(&self, id: EdgeId) -> Option<&Edge> {
        self.data.edges.get(id.index())
    }

    /// Returns an edge use by typed ID.
    pub fn edge_use(&self, id: EdgeUseId) -> Option<&EdgeUse> {
        self.data.edge_uses.get(id.index())
    }

    /// Returns a wire by typed ID.
    pub fn wire(&self, id: WireId) -> Option<&Wire> {
        self.data.wires.get(id.index())
    }

    /// Returns a face by typed ID.
    pub fn face(&self, id: FaceId) -> Option<&Face> {
        self.data.faces.get(id.index())
    }

    /// Returns a shell by typed ID.
    pub fn shell(&self, id: ShellId) -> Option<&Shell> {
        self.data.shells.get(id.index())
    }

    /// Returns a solid by typed ID.
    pub fn solid(&self, id: SolidId) -> Option<&Solid> {
        self.data.solids.get(id.index())
    }

    /// Returns edges incident to a vertex in deterministic insertion order.
    pub fn edges_at_vertex(&self, id: VertexId) -> Option<&[EdgeId]> {
        self.data.vertex_edges.get(id.index()).map(Vec::as_slice)
    }

    /// Returns all uses of an edge in deterministic insertion order.
    pub fn uses_of_edge(&self, id: EdgeId) -> Option<&[EdgeUseId]> {
        self.data
            .edge_uses_by_edge
            .get(id.index())
            .map(Vec::as_slice)
    }

    /// Returns the owning wire of an edge use.
    pub fn wire_of_edge_use(&self, id: EdgeUseId) -> Option<WireId> {
        self.data.edge_use_wire.get(id.index()).copied()
    }

    /// Returns the owning face of a wire.
    pub fn face_of_wire(&self, id: WireId) -> Option<FaceId> {
        self.data.wire_face.get(id.index()).copied()
    }

    /// Returns the owning shell of a face.
    pub fn shell_of_face(&self, id: FaceId) -> Option<ShellId> {
        self.data.face_shell.get(id.index()).copied()
    }

    /// Returns the owning solid of a shell, when it bounds a solid.
    pub fn solid_of_shell(&self, id: ShellId) -> Option<SolidId> {
        self.data.shell_solid.get(id.index()).copied().flatten()
    }

    pub(in crate::topology) fn directed_vertices(
        &self,
        edge_use: EdgeUseId,
    ) -> (VertexId, VertexId) {
        let edge_use = self.edge_use(edge_use).expect("validated edge-use ID");
        let edge = self.edge(edge_use.edge).expect("validated edge ID");
        match edge_use.direction {
            Direction::Forward => (edge.start, edge.end),
            Direction::Reversed => (edge.end, edge.start),
        }
    }
}
