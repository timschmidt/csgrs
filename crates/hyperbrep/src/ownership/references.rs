//! Borrowed topology with revision identity.

use super::*;
use std::hash::{Hash, Hasher};

macro_rules! entity_reference {
    ($reference:ident, $key:ident, $id:ident, $kind:ident, $one:ident, $many:ident) => {
        /// A borrowed entity in one immutable model revision.
        ///
        /// Equality and hashing express topology identity, not geometric equality.
        #[derive(Clone, Copy, Debug)]
        pub struct $reference<'a> {
            pub(super) model: &'a Model,
            pub(super) id: $id,
        }

        impl<'a> $reference<'a> {
            /// Returns the owning model revision.
            pub fn model(self) -> &'a Model {
                self.model
            }

            /// Returns the arena position for diagnostics only.
            ///
            /// Positions are not transferable references or durable feature names.
            pub fn index(self) -> usize {
                self.id.index()
            }
        }

        impl PartialEq for $reference<'_> {
            fn eq(&self, other: &Self) -> bool {
                self.id == other.id && self.model.same_revision(other.model)
            }
        }
        impl Eq for $reference<'_> {}
        impl Hash for $reference<'_> {
            fn hash<H: Hasher>(&self, state: &mut H) {
                self.model.inner.snapshot_address().hash(state);
                self.id.hash(state);
            }
        }

        impl Model {
            /// Resolves a key issued by the builder that published this revision.
            ///
            /// Keys from another builder or from an earlier revision are rejected.
            pub fn $one(&self, key: &$key) -> Result<$reference<'_>, GeometryReferenceError> {
                if self
                    .construction
                    .as_ref()
                    .is_some_and(|scope| Arc::ptr_eq(scope, &key.scope))
                {
                    Ok($reference {
                        model: self,
                        id: key.id,
                    })
                } else {
                    Err(GeometryReferenceError::ForeignModel {
                        kind: EntityKind::$kind,
                    })
                }
            }

            /// Iterates over bound entities in deterministic arena order.
            pub fn $many(&self) -> impl ExactSizeIterator<Item = $reference<'_>> {
                self.inner
                    .$many()
                    .map(|(id, _)| $reference { model: self, id })
            }
        }
    };
}

entity_reference!(VertexRef, VertexKey, VertexId, Vertex, vertex, vertices);
entity_reference!(Curve3Ref, Curve3Key, Curve3Id, Curve3, curve, curves);
entity_reference!(PcurveRef, PcurveKey, PcurveId, Pcurve, pcurve, pcurves);
entity_reference!(
    SurfaceRef, SurfaceKey, SurfaceId, Surface, surface, surfaces
);
entity_reference!(EdgeRef, EdgeKey, EdgeId, Edge, edge, edges);
entity_reference!(
    EdgeUseRef, EdgeUseKey, EdgeUseId, EdgeUse, edge_use, edge_uses
);
entity_reference!(WireRef, WireKey, WireId, Wire, wire, wires);
entity_reference!(FaceRef, FaceKey, FaceId, Face, face, faces);
entity_reference!(ShellRef, ShellKey, ShellId, Shell, shell, shells);
entity_reference!(SolidRef, SolidKey, SolidId, Solid, solid, solids);

impl<'a> VertexRef<'a> {
    /// Returns the exact model-space point.
    pub fn point(self) -> &'a GeometricPoint3 {
        self.model
            .inner
            .vertex(self.id)
            .expect("validated vertex")
            .point()
    }

    /// Iterates over incident edges in insertion order.
    pub fn edges(self) -> impl ExactSizeIterator<Item = EdgeRef<'a>> {
        self.model
            .inner
            .edges_at_vertex(self.id)
            .expect("validated adjacency")
            .iter()
            .map(move |&id| EdgeRef {
                model: self.model,
                id,
            })
    }
}

macro_rules! geometry_reference {
    ($reference:ident, $geometry:ident, $lookup:ident) => {
        impl<'a> $reference<'a> {
            /// Borrows the immutable exact geometry.
            pub fn geometry(self) -> &'a $geometry {
                self.model
                    .inner
                    .$lookup(self.id)
                    .expect("validated geometry")
            }
        }
        impl std::ops::Deref for $reference<'_> {
            type Target = $geometry;
            fn deref(&self) -> &Self::Target {
                self.geometry()
            }
        }
    };
}
geometry_reference!(Curve3Ref, Curve3, curve);
geometry_reference!(PcurveRef, Pcurve, pcurve);
geometry_reference!(SurfaceRef, Surface, surface);

impl<'a> EdgeRef<'a> {
    fn record(self) -> &'a Edge {
        self.model.inner.edge(self.id).expect("validated edge")
    }
    /// Returns the canonical start vertex in this model.
    pub fn start(self) -> VertexRef<'a> {
        VertexRef {
            model: self.model,
            id: self.record().start(),
        }
    }
    /// Returns the canonical end vertex in this model.
    pub fn end(self) -> VertexRef<'a> {
        VertexRef {
            model: self.model,
            id: self.record().end(),
        }
    }
    /// Returns the exact spatial carrier.
    pub fn curve(self) -> Curve3Ref<'a> {
        Curve3Ref {
            model: self.model,
            id: self.record().curve(),
        }
    }
    /// Returns the retained exact parameter interval.
    pub fn domain(self) -> &'a ParameterRange {
        self.record().domain()
    }
    /// Iterates over this edge's uses in insertion order.
    pub fn uses(self) -> impl ExactSizeIterator<Item = EdgeUseRef<'a>> {
        self.model
            .inner
            .uses_of_edge(self.id)
            .expect("validated adjacency")
            .iter()
            .map(move |&id| EdgeUseRef {
                model: self.model,
                id,
            })
    }
}

impl<'a> EdgeUseRef<'a> {
    fn record(self) -> &'a EdgeUse {
        self.model
            .inner
            .edge_use(self.id)
            .expect("validated edge use")
    }
    /// Returns the shared spatial edge.
    pub fn edge(self) -> EdgeRef<'a> {
        EdgeRef {
            model: self.model,
            id: self.record().edge(),
        }
    }
    /// Returns the directed traversal of that edge.
    pub fn direction(self) -> Direction {
        self.record().direction()
    }
    /// Returns this use's face-local pcurve.
    pub fn pcurve(self) -> PcurveRef<'a> {
        PcurveRef {
            model: self.model,
            id: self.record().pcurve(),
        }
    }
    /// Returns the retained exact pcurve-to-edge correspondence.
    pub fn parameter_correspondence(self) -> &'a ParameterCorrespondence {
        self.record().parameter_correspondence()
    }
    /// Returns the owning wire.
    pub fn wire(self) -> WireRef<'a> {
        WireRef {
            model: self.model,
            id: self
                .model
                .inner
                .wire_of_edge_use(self.id)
                .expect("validated owner"),
        }
    }
    /// Maps a pcurve parameter to the exact shared edge parameter.
    pub fn edge_parameter_at(self, parameter: &Parameter) -> Result<Parameter, QueryError> {
        self.model.inner.edge_parameter_at(self.id, parameter)
    }
    /// Maps a shared edge parameter to the exact face-local pcurve parameter.
    pub fn pcurve_parameter_at(self, parameter: &Parameter) -> Result<Parameter, QueryError> {
        self.model.inner.pcurve_parameter_at(self.id, parameter)
    }
}

impl<'a> WireRef<'a> {
    /// Iterates over directed uses in boundary order.
    pub fn edge_uses(self) -> impl ExactSizeIterator<Item = EdgeUseRef<'a>> {
        self.model
            .inner
            .wire(self.id)
            .expect("validated wire")
            .edge_uses()
            .iter()
            .map(move |&id| EdgeUseRef {
                model: self.model,
                id,
            })
    }
    /// Returns the owning face.
    pub fn face(self) -> FaceRef<'a> {
        FaceRef {
            model: self.model,
            id: self
                .model
                .inner
                .face_of_wire(self.id)
                .expect("validated owner"),
        }
    }
}

impl<'a> FaceRef<'a> {
    fn record(self) -> &'a Face {
        self.model.inner.face(self.id).expect("validated face")
    }
    /// Returns the support surface in this revision.
    pub fn surface(self) -> SurfaceRef<'a> {
        SurfaceRef {
            model: self.model,
            id: self.record().surface(),
        }
    }
    /// Returns orientation relative to the authored support.
    pub fn orientation(self) -> Orientation {
        self.record().orientation()
    }
    /// Returns the outer trim, absent for a whole closed support.
    pub fn outer(self) -> Option<WireRef<'a>> {
        self.record().outer().map(|id| WireRef {
            model: self.model,
            id,
        })
    }
    /// Iterates over inner trims in authored order.
    pub fn inner(self) -> impl ExactSizeIterator<Item = WireRef<'a>> {
        self.record().inner().iter().map(move |&id| WireRef {
            model: self.model,
            id,
        })
    }
    /// Whether this face contains its complete closed support.
    pub fn is_whole_surface(self) -> bool {
        self.record().is_whole_surface()
    }
    /// Returns the owning shell.
    pub fn shell(self) -> ShellRef<'a> {
        ShellRef {
            model: self.model,
            id: self
                .model
                .inner
                .shell_of_face(self.id)
                .expect("validated owner"),
        }
    }
    /// Returns certified area, or a typed unsupported/unresolved result.
    pub fn area(self) -> Result<Real, QueryError> {
        self.model.inner.face_area(self.id)
    }
}

impl<'a> ShellRef<'a> {
    /// Iterates over oriented faces in authored order.
    pub fn faces(self) -> impl ExactSizeIterator<Item = FaceRef<'a>> {
        self.model
            .inner
            .shell(self.id)
            .expect("validated shell")
            .faces()
            .iter()
            .map(move |&id| FaceRef {
                model: self.model,
                id,
            })
    }
    /// Returns the owning solid, absent for a sheet shell.
    pub fn solid(self) -> Option<SolidRef<'a>> {
        self.model.inner.solid_of_shell(self.id).map(|id| SolidRef {
            model: self.model,
            id,
        })
    }
}

impl<'a> SolidRef<'a> {
    fn record(self) -> &'a Solid {
        self.model.inner.solid(self.id).expect("validated solid")
    }
    /// Returns the outward shell.
    pub fn outer(self) -> ShellRef<'a> {
        ShellRef {
            model: self.model,
            id: self.record().outer(),
        }
    }
    /// Iterates over inward cavity shells.
    pub fn voids(self) -> impl ExactSizeIterator<Item = ShellRef<'a>> {
        self.record().voids().iter().map(move |&id| ShellRef {
            model: self.model,
            id,
        })
    }
    /// Returns certified volume, or a typed unsupported/unresolved result.
    pub fn volume(self) -> Result<Real, QueryError> {
        self.model.inner.solid_volume(self.id)
    }
    /// Classifies a point against this solid's exact material region.
    pub fn classify_point(self, point: &Point3) -> Result<SolidPointLocation, QueryError> {
        self.model.inner.classify_point(self.id, point)
    }
}
