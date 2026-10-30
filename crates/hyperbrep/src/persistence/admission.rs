//! Resource-only record admission before any exact scalar is deserialized.

use std::fmt;
use std::marker::PhantomData;

use serde::de::{
    DeserializeOwned, DeserializeSeed, Error, IgnoredAny, MapAccess, SeqAccess, Visitor,
};
use serde::{Deserialize, Deserializer};

use super::{
    ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, FORMAT_VERSION, PersistenceError,
};

mod shapes;
use shapes::{Curve, Face, Pcurve, Selection, Solid, Surface, Vertex, Wire};

pub(super) fn check_records(json: &str, limits: &ArchiveLimits) -> Result<(), PersistenceError> {
    let mut budget = Budget {
        limits,
        records: 0,
        dependency_depths: Vec::new(),
        geometry_elements: 0,
        topology_references: 0,
        exceeded: None,
    };
    let mut decoder = serde_json::Deserializer::from_str(json);
    decoder.disable_recursion_limit();
    let result = Envelope(&mut budget).deserialize(serde_stacker::Deserializer::new(&mut decoder));
    let version = match result {
        Ok(version) => version,
        Err(error) => {
            // A record may precede `version` and belong to an unsupported schema.
            // Only on failure, diagnose the envelope independently; the admitted
            // path reads version and structure together in one pass.
            #[derive(Deserialize)]
            struct Header {
                version: u32,
            }
            let mut header = serde_json::Deserializer::from_str(json);
            header.disable_recursion_limit();
            if let Ok(header) = Header::deserialize(serde_stacker::Deserializer::new(&mut header))
                && header.version != FORMAT_VERSION
            {
                return Err(PersistenceError::UnsupportedVersion(header.version));
            }
            return Err(match budget.exceeded {
                Some(exceeded) => PersistenceError::ResourceLimit(exceeded),
                None => PersistenceError::Json(error.to_string()),
            });
        }
    };
    if version != FORMAT_VERSION {
        return Err(PersistenceError::UnsupportedVersion(version));
    }
    decoder
        .end()
        .map_err(|e| PersistenceError::Json(e.to_string()))
}

struct Budget<'a> {
    limits: &'a ArchiveLimits,
    dependency_depths: Vec<usize>,
    records: usize,
    geometry_elements: usize,
    topology_references: usize,
    // Preserve typed errors across Serde's format-specific error channel.
    exceeded: Option<ArchiveLimitExceeded>,
}

impl Budget<'_> {
    // Each counted record/entry corresponds to distinct input syntax, so the
    // sums are bounded by the borrowed JSON byte length. No coefficients or
    // allocation size hints are used as counts.
    fn check<E: Error>(
        &mut self,
        resource: ArchiveResource,
        observed: usize,
        limit: usize,
    ) -> Result<(), E> {
        if observed > limit {
            self.exceeded = Some(ArchiveLimitExceeded {
                resource,
                observed,
                limit,
            });
            Err(E::custom("archive record resource limit exceeded"))
        } else {
            Ok(())
        }
    }

    fn record<E: Error>(&mut self) -> Result<(), E> {
        self.records += 1;
        self.check(ArchiveResource::Records, self.records, self.limits.records)
    }

    fn degree<E: Error>(&mut self, degree: usize) -> Result<(), E> {
        self.check(ArchiveResource::Degree, degree, self.limits.degree)
    }

    fn solver_steps<E: Error>(&mut self, steps: usize) -> Result<(), E> {
        self.check(
            ArchiveResource::SolverSteps,
            steps,
            self.limits.solver_steps,
        )
    }

    fn geometry<E: Error>(&mut self, amount: usize) -> Result<(), E> {
        self.geometry_elements += amount;
        self.check(
            ArchiveResource::GeometryElements,
            self.geometry_elements,
            self.limits.geometry_elements,
        )
    }

    fn topology<E: Error>(&mut self, amount: usize) -> Result<(), E> {
        self.topology_references += amount;
        self.check(
            ArchiveResource::TopologyReferences,
            self.topology_references,
            self.limits.topology_references,
        )
    }
}

trait Record: DeserializeOwned {
    fn admit<E: Error>(self, budget: &mut Budget<'_>) -> Result<(), E>;
}

impl Record for IgnoredAny {
    fn admit<E: Error>(self, _: &mut Budget<'_>) -> Result<(), E> {
        Ok(())
    }
}

struct Records<'a, 'b, T>(&'a mut Budget<'b>, PhantomData<T>);
struct One<'a, 'b, T>(&'a mut Budget<'b>, PhantomData<T>);

impl<'de, T: Record> DeserializeSeed<'de> for One<'_, '_, T> {
    type Value = ();
    fn deserialize<D: Deserializer<'de>>(self, decoder: D) -> Result<(), D::Error> {
        // Charge before inspecting this record, including its ignored scalar payload.
        self.0.record()?;
        T::deserialize(decoder)?.admit(self.0)
    }
}

impl<'de, T: Record> DeserializeSeed<'de> for Records<'_, '_, T> {
    type Value = ();
    fn deserialize<D: Deserializer<'de>>(self, decoder: D) -> Result<(), D::Error> {
        decoder.deserialize_seq(self)
    }
}

impl<'de, T: Record> Visitor<'de> for Records<'_, '_, T> {
    type Value = ();
    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("an array of archive records")
    }
    fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<(), A::Error> {
        while seq
            .next_element_seed(One::<T>(self.0, PhantomData))?
            .is_some()
        {}
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(field_identifier, rename_all = "snake_case")]
enum Field {
    Version,
    Selection,
    Vertices,
    Curves,
    Pcurves,
    Surfaces,
    Edges,
    EdgeUses,
    Wires,
    Faces,
    Shells,
    Solids,
    Parameters,
    PointSources,
    PcurveSources,
    PolynomialSources,
    #[serde(other)]
    Ignore,
}

struct Envelope<'a, 'b>(&'a mut Budget<'b>);
struct Graph<'a, 'b>(&'a mut Budget<'b>);

impl<'de> DeserializeSeed<'de> for Envelope<'_, '_> {
    type Value = u32;
    fn deserialize<D: Deserializer<'de>>(self, decoder: D) -> Result<u32, D::Error> {
        decoder.deserialize_map(self)
    }
}
impl<'de> DeserializeSeed<'de> for Graph<'_, '_> {
    type Value = ();
    fn deserialize<D: Deserializer<'de>>(self, decoder: D) -> Result<(), D::Error> {
        decoder.deserialize_map(self)
    }
}
impl<'de> Visitor<'de> for Envelope<'_, '_> {
    type Value = u32;
    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("an archive object")
    }
    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<u32, A::Error> {
        let mut version = None;
        while let Some(field) = map.next_key::<Field>()? {
            match field {
                Field::Version => {
                    if version.is_some() {
                        return Err(A::Error::duplicate_field("version"));
                    }
                    version = Some(map.next_value()?);
                }
                _ if version.is_some_and(|version| version != FORMAT_VERSION) => {
                    map.next_value::<IgnoredAny>()?;
                }
                Field::Selection => map.next_value_seed(Graph(self.0))?,
                Field::Vertices => map.next_value_seed(Records::<Vertex>(self.0, PhantomData))?,
                Field::Edges | Field::EdgeUses => {
                    map.next_value_seed(Records::<IgnoredAny>(self.0, PhantomData))?
                }
                Field::Curves => map.next_value_seed(Records::<Curve>(self.0, PhantomData))?,
                Field::Pcurves => map.next_value_seed(Records::<Pcurve>(self.0, PhantomData))?,
                Field::Surfaces => map.next_value_seed(Records::<Surface>(self.0, PhantomData))?,
                Field::Wires => map.next_value_seed(Records::<Wire>(self.0, PhantomData))?,
                Field::Faces => map.next_value_seed(Records::<Face>(self.0, PhantomData))?,
                Field::Shells => {
                    map.next_value_seed(Records::<shapes::Count>(self.0, PhantomData))?
                }
                Field::Solids => map.next_value_seed(Records::<Solid>(self.0, PhantomData))?,
                _ => {
                    map.next_value::<IgnoredAny>()?;
                }
            }
        }
        version.ok_or_else(|| A::Error::missing_field("version"))
    }
}
impl<'de> Visitor<'de> for Graph<'_, '_> {
    type Value = ();
    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a selected geometry graph")
    }
    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        while let Some(field) = map.next_key::<Field>()? {
            match field {
                Field::Parameters => {
                    map.next_value_seed(Records::<Selection>(self.0, PhantomData))?
                }
                Field::PointSources => {
                    map.next_value_seed(Records::<Curve>(self.0, PhantomData))?
                }
                Field::PcurveSources => {
                    map.next_value_seed(Records::<Pcurve>(self.0, PhantomData))?
                }
                Field::PolynomialSources => {
                    map.next_value_seed(Records::<shapes::PolynomialSource>(self.0, PhantomData))?
                }
                _ => {
                    map.next_value::<IgnoredAny>()?;
                }
            }
        }
        Ok(())
    }
}
