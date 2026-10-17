//! Identity-preserving transfer of complete models or selected solid subgraphs.

use super::*;
use std::ops::Index;

impl ArenaModel {
    pub(crate) fn append_to_builder(
        &self,
        builder: &mut ArenaModelBuilder,
    ) -> Result<Vec<SolidId>, BuildError> {
        Ok(self
            .append_records(builder, None)?
            .0
            .into_iter()
            .flatten()
            .collect())
    }

    /// Copies only the selected solids and their reachable topology. Curve-image
    /// points and retained certificates are cloned, never reconstructed numerically.
    pub(crate) fn append_solids_to_builder(
        &self,
        builder: &mut ArenaModelBuilder,
        solids: &[SolidId],
    ) -> Result<Vec<SolidId>, BuildError> {
        let selection = Selection::for_solids(self, solids)?;
        let mapped = self.append_records(builder, Some(&selection))?;
        Ok(solids.iter().map(|id| mapped[id.index()]).collect())
    }

    fn append_records(
        &self,
        builder: &mut ArenaModelBuilder,
        selection: Option<&Selection>,
    ) -> Result<Mapped<SolidId>, BuildError> {
        let vertices = copy_records(
            &self.data.vertices,
            selection.map(|s| s.vertices.as_slice()),
            |vertex| builder.vertex(vertex.point.clone()),
        )?;
        let curves = copy_records(
            &self.data.curves,
            selection.map(|s| s.curves.as_slice()),
            |curve| builder.curve(curve.clone()),
        )?;
        let pcurves = copy_records(
            &self.data.pcurves,
            selection.map(|s| s.pcurves.as_slice()),
            |pcurve| builder.pcurve(pcurve.clone()),
        )?;
        let surfaces = copy_records(
            &self.data.surfaces,
            selection.map(|s| s.surfaces.as_slice()),
            |surface| builder.surface(surface.clone()),
        )?;
        let edges = copy_records(
            &self.data.edges,
            selection.map(|s| s.edges.as_slice()),
            |edge| {
                builder.edge_with_endpoint_certificate(
                    vertices[edge.start.index()],
                    vertices[edge.end.index()],
                    curves[edge.curve.index()],
                    edge.domain.clone(),
                    edge.endpoint_certificate,
                )
            },
        )?;
        let edge_uses = copy_records(
            &self.data.edge_uses,
            selection.map(|s| s.edge_uses.as_slice()),
            |edge_use| {
                builder.edge_use_with_image_certificate(
                    edges[edge_use.edge.index()],
                    edge_use.direction,
                    pcurves[edge_use.pcurve.index()],
                    edge_use.parameter_correspondence.clone(),
                    edge_use.image_certificate,
                )
            },
        )?;
        let wires = copy_records(
            &self.data.wires,
            selection.map(|s| s.wires.as_slice()),
            |wire| {
                builder.wire_with_simplicity_certificate(
                    wire.edge_uses
                        .iter()
                        .map(|edge_use| edge_uses[edge_use.index()])
                        .collect(),
                    wire.simplicity_certificate,
                )
            },
        )?;
        let faces = copy_records(
            &self.data.faces,
            selection.map(|s| s.faces.as_slice()),
            |face| match &face.boundary {
                FaceBoundary::WholeSurface => {
                    builder.whole_face(surfaces[face.surface.index()], face.orientation)
                }
                FaceBoundary::Trimmed { outer, inner } => builder.face(
                    surfaces[face.surface.index()],
                    face.orientation,
                    wires[outer.index()],
                    inner.iter().map(|wire| wires[wire.index()]).collect(),
                ),
            },
        )?;
        let shells = copy_records(
            &self.data.shells,
            selection.map(|s| s.shells.as_slice()),
            |shell| builder.shell(shell.faces.iter().map(|face| faces[face.index()]).collect()),
        )?;
        copy_records_indexed(
            &self.data.solids,
            selection.map(|s| s.solids.as_slice()),
            |index, solid| {
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
                            axial: self.data.certified_axial_shells[index].clone(),
                        },
                    )
                } else {
                    builder.solid(outer, voids)
                }
            },
        )
    }
}

// Every dependency is selected before transfer. Indexing this relocation table
// is an internal invariant; callers cannot supply arena positions publicly.
struct Mapped<T>(Vec<Option<T>>);
impl<T> Index<usize> for Mapped<T> {
    type Output = T;
    fn index(&self, index: usize) -> &T {
        self.0[index]
            .as_ref()
            .expect("selected topology has all its dependencies")
    }
}
fn copy_records<T, U>(
    records: &[T],
    selection: Option<&[bool]>,
    mut copy: impl FnMut(&T) -> Result<U, BuildError>,
) -> Result<Mapped<U>, BuildError> {
    copy_records_indexed(records, selection, |_, record| copy(record))
}
fn copy_records_indexed<T, U>(
    records: &[T],
    selection: Option<&[bool]>,
    mut copy: impl FnMut(usize, &T) -> Result<U, BuildError>,
) -> Result<Mapped<U>, BuildError> {
    records
        .iter()
        .enumerate()
        .map(|(i, record)| {
            if selection.is_none_or(|selected| selected[i]) {
                copy(i, record).map(Some)
            } else {
                Ok(None)
            }
        })
        .collect::<Result<Vec<_>, _>>()
        .map(Mapped)
}

struct Selection {
    vertices: Vec<bool>,
    curves: Vec<bool>,
    pcurves: Vec<bool>,
    surfaces: Vec<bool>,
    edges: Vec<bool>,
    edge_uses: Vec<bool>,
    wires: Vec<bool>,
    faces: Vec<bool>,
    shells: Vec<bool>,
    solids: Vec<bool>,
}
impl Selection {
    fn for_solids(model: &ArenaModel, solids: &[SolidId]) -> Result<Self, BuildError> {
        let mut selected = Self {
            vertices: vec![false; model.data.vertices.len()],
            curves: vec![false; model.data.curves.len()],
            pcurves: vec![false; model.data.pcurves.len()],
            surfaces: vec![false; model.data.surfaces.len()],
            edges: vec![false; model.data.edges.len()],
            edge_uses: vec![false; model.data.edge_uses.len()],
            wires: vec![false; model.data.wires.len()],
            faces: vec![false; model.data.faces.len()],
            shells: vec![false; model.data.shells.len()],
            solids: vec![false; model.data.solids.len()],
        };
        for &id in solids {
            let solid = model.solid(id).ok_or(BuildError::InvalidReference {
                kind: EntityKind::Solid,
                index: id.index(),
            })?;
            selected.solids[id.index()] = true;
            for shell in std::iter::once(solid.outer).chain(solid.voids.iter().copied()) {
                selected.shells[shell.index()] = true;
                for &face in &model.data.shells[shell.index()].faces {
                    selected.faces[face.index()] = true;
                    let face = &model.data.faces[face.index()];
                    selected.surfaces[face.surface.index()] = true;
                    for wire in face.outer().into_iter().chain(face.inner().iter().copied()) {
                        selected.wires[wire.index()] = true;
                        for &edge_use in &model.data.wires[wire.index()].edge_uses {
                            selected.edge_uses[edge_use.index()] = true;
                            let edge_use = &model.data.edge_uses[edge_use.index()];
                            selected.pcurves[edge_use.pcurve.index()] = true;
                            selected.edges[edge_use.edge.index()] = true;
                            let edge = &model.data.edges[edge_use.edge.index()];
                            selected.curves[edge.curve.index()] = true;
                            selected.vertices[edge.start.index()] = true;
                            selected.vertices[edge.end.index()] = true;
                        }
                    }
                }
            }
        }
        Ok(selected)
    }
}
