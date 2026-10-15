use super::{
    Error, ErrorKind, ImportLimits, Resource, StepFile,
    error::{charge, push},
};
use crate::part21::{Entity, Record, Value, ValueKind, Values};
use hyperbrep::{Point3, Real};
use std::collections::HashMap;

#[derive(Clone, Copy)]
pub(super) struct Site {
    pub id: Option<u64>,
    pub offset: usize,
}
impl Site {
    pub fn entity(entity: Entity<'_, '_>) -> Self {
        Self {
            id: Some(entity.id()),
            offset: entity.span().start,
        }
    }
    pub fn error(self, kind: ErrorKind) -> Error {
        Error::at(self.id, self.offset, kind)
    }
    pub fn invalid(self, reason: &'static str) -> Error {
        self.error(ErrorKind::Invalid(reason))
    }
}

pub(super) struct Args<'e, 'a> {
    pub site: Site,
    values: Values<'e, 'a>,
}
impl<'e, 'a> Args<'e, 'a> {
    pub fn new(record: Record<'e, 'a>, site: Site) -> Self {
        Self {
            site,
            values: record.arguments(),
        }
    }
    pub fn next(&mut self) -> Result<Value<'e, 'a>, Error> {
        self.values
            .next()
            .ok_or_else(|| self.site.error(ErrorKind::Expected("another attribute")))
    }
    pub fn finish(mut self) -> Result<(), Error> {
        if let Some(value) = self.values.next() {
            return Err(Error::at(
                self.site.id,
                value.span().start,
                ErrorKind::Expected("end of attributes"),
            ));
        }
        Ok(())
    }
    pub fn reference(&mut self) -> Result<u64, Error> {
        reference(self.next()?, self.site)
    }
    pub fn optional_reference(&mut self) -> Result<Option<u64>, Error> {
        let value = self.next()?;
        if value.kind() == ValueKind::Omitted {
            Ok(None)
        } else {
            reference(value, self.site).map(Some)
        }
    }
    pub fn boolean(&mut self) -> Result<bool, Error> {
        let value = self.next()?;
        match value.kind() {
            ValueKind::Enumeration("T") => Ok(true),
            ValueKind::Enumeration("F") => Ok(false),
            _ => Err(Error::at(
                self.site.id,
                value.span().start,
                ErrorKind::Expected("BOOLEAN (.T. or .F.)"),
            )),
        }
    }
    pub fn list(&mut self) -> Result<Values<'e, 'a>, Error> {
        aggregate(self.next()?, self.site)
    }
}
pub(super) fn reference(value: Value<'_, '_>, site: Site) -> Result<u64, Error> {
    match value.kind() {
        ValueKind::Reference(id) => Ok(id),
        _ => Err(Error::at(
            site.id,
            value.span().start,
            ErrorKind::Expected("entity reference"),
        )),
    }
}
pub(super) fn aggregate<'e, 'a>(value: Value<'e, 'a>, site: Site) -> Result<Values<'e, 'a>, Error> {
    if value.kind() == ValueKind::Aggregate {
        Ok(value.children())
    } else {
        Err(Error::at(
            site.id,
            value.span().start,
            ErrorKind::Expected("aggregate"),
        ))
    }
}
pub(super) fn component<'e, 'a>(entity: Entity<'e, 'a>, name: &str) -> Option<Record<'e, 'a>> {
    entity.components().find(|record| record.name() == name)
}
pub(super) fn parts(
    entity: Entity<'_, '_>,
    required: &[&str],
    optional: &[&str],
) -> Result<(), Error> {
    let site = Site::entity(entity);
    for record in entity.components() {
        if !required.contains(&record.name()) && !optional.contains(&record.name()) {
            return Err(site.error(ErrorKind::Unsupported("entity component in this mapping")));
        }
        if entity
            .components()
            .filter(|other| other.name() == record.name())
            .count()
            != 1
        {
            return Err(site.invalid("duplicate complex-entity component"));
        }
    }
    if required
        .iter()
        .any(|name| component(entity, name).is_none())
    {
        return Err(site.error(ErrorKind::Expected("required entity component")));
    }
    Ok(())
}

pub(super) struct Resolver<'e, 'a, 'l> {
    pub file: &'e StepFile<'a>,
    pub limits: &'l ImportLimits,
    visits: usize,
    scalars: usize,
    text_bytes: usize,
    edge_uses: usize,
    pub used: Vec<u64>,
    pub points: HashMap<u64, Point3>,
    pub units: HashMap<u64, super::Unit>,
}
impl<'e, 'a, 'l> Resolver<'e, 'a, 'l> {
    pub fn new(file: &'e StepFile<'a>, limits: &'l ImportLimits) -> Self {
        Self {
            file,
            limits,
            visits: 0,
            scalars: 0,
            text_bytes: 0,
            edge_uses: 0,
            used: Vec::new(),
            points: HashMap::new(),
            units: HashMap::new(),
        }
    }
    pub fn entity(&mut self, id: u64) -> Result<Entity<'e, 'a>, Error> {
        charge(
            &mut self.visits,
            1,
            self.limits.entity_visits,
            Resource::EntityVisits,
        )
        .map_err(|kind| Error::at(Some(id), 0, kind))?;
        let entity = self.file.syntax.entity(id).ok_or_else(|| {
            let origin = self
                .file
                .syntax
                .references()
                .find(|reference| reference.target == id);
            Error::at(
                origin.as_ref().and_then(|reference| reference.source),
                origin.map_or(0, |reference| reference.span.start),
                ErrorKind::MissingEntity(id),
            )
        })?;
        push(&mut self.used, id).map_err(|kind| Site::entity(entity).error(kind))?;
        Ok(entity)
    }
    pub fn simple(&mut self, id: u64, name: &'static str) -> Result<Args<'e, 'a>, Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        let mut components = entity.components();
        let record = components.next().expect("syntax requires a component");
        if entity.is_complex() || record.name() != name || components.next().is_some() {
            return Err(site.error(ErrorKind::Expected(name)));
        }
        Ok(Args::new(record, site))
    }
    pub fn text(&mut self, value: Value<'_, '_>, site: Site) -> Result<String, Error> {
        super::text::decode(value, site, &mut self.text_bytes, self.limits.text_bytes)
    }
    pub fn scalar(&mut self, value: Value<'_, '_>, site: Site) -> Result<Real, Error> {
        charge(&mut self.scalars, 1, self.limits.scalars, Resource::Scalars)
            .map_err(|kind| site.error(kind))?;
        match value.kind() {
            ValueKind::Real(number) | ValueKind::Integer(number) => number
                .to_real(&self.limits.number)
                .map_err(|e| Error::at(site.id, value.span().start, ErrorKind::Syntax(e))),
            _ => Err(Error::at(
                site.id,
                value.span().start,
                ErrorKind::Expected("numeric value"),
            )),
        }
    }
    pub fn edge_use(&mut self, site: Site) -> Result<(), Error> {
        charge(
            &mut self.edge_uses,
            1,
            self.limits.edge_uses,
            Resource::EdgeUses,
        )
        .map_err(|kind| site.error(kind))
    }
}
