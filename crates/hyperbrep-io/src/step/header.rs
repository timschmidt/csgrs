use super::{
    Error, ErrorKind,
    decode::{Args, Site, aggregate},
    error::{insert, push},
    text,
};
use crate::part21::{Exchange, Value};
use std::collections::HashSet;

/// Decoded standard HEADER attributes; original spelling remains in the syntax.
#[derive(Debug)]
pub struct Header {
    /// Descriptions, in source order.
    pub description: Vec<String>,
    /// Part 21 implementation level.
    pub implementation_level: String,
    /// Authored file name.
    pub name: String,
    /// Authored timestamp text; calendar/timezone validity is not inferred.
    pub timestamp: String,
    /// Author labels.
    pub authors: Vec<String>,
    /// Organization labels.
    pub organizations: Vec<String>,
    /// Preprocessor identifier.
    pub preprocessor: String,
    /// Originating system identifier.
    pub originating_system: String,
    /// Authorization text.
    pub authorization: String,
    /// Declared schema names, including any version/OID suffix.
    pub schemas: Vec<String>,
}
pub(super) fn read(exchange: &Exchange<'_>, limit: usize) -> Result<Header, Error> {
    let site = Site {
        id: None,
        offset: 0,
    };
    let mut records = exchange.header();
    let mut total = 0;
    let mut next = |name| {
        let record = records
            .next()
            .ok_or_else(|| site.error(ErrorKind::Expected(name)))?;
        if record.name() != name {
            return Err(site.error(ErrorKind::Expected(name)));
        }
        Ok(Args::new(record, site))
    };
    let mut description = next("FILE_DESCRIPTION")?;
    let descriptions = strings(description.next()?, &mut total, limit)?;
    let implementation_level = text::decode(description.next()?, site, &mut total, limit)?;
    if !matches!(implementation_level.as_str(), "2;1" | "3;1" | "4;1") {
        return Err(site.error(ErrorKind::Unsupported("Part 21 implementation level")));
    }
    description.finish()?;
    let mut file = next("FILE_NAME")?;
    let name = text::decode(file.next()?, site, &mut total, limit)?;
    let timestamp = text::decode(file.next()?, site, &mut total, limit)?;
    let authors = strings(file.next()?, &mut total, limit)?;
    let organizations = strings(file.next()?, &mut total, limit)?;
    let preprocessor = text::decode(file.next()?, site, &mut total, limit)?;
    let originating_system = text::decode(file.next()?, site, &mut total, limit)?;
    let authorization = text::decode(file.next()?, site, &mut total, limit)?;
    file.finish()?;
    let mut schema = next("FILE_SCHEMA")?;
    let schemas = strings(schema.next()?, &mut total, limit)?;
    schema.finish()?;
    for record in records {
        if matches!(
            record.name(),
            "FILE_DESCRIPTION" | "FILE_NAME" | "FILE_SCHEMA"
        ) {
            return Err(site.invalid("duplicate standard HEADER record"));
        }
    }
    let mut unique_schemas = HashSet::new();
    for schema in &schemas {
        if schema.is_empty()
            || !insert(&mut unique_schemas, schema.as_str()).map_err(|kind| site.error(kind))?
        {
            return Err(site.invalid("empty or duplicate schema name"));
        }
    }
    let mut names = HashSet::new();
    for section in exchange.sections() {
        if let Some(mut arguments) = section.arguments() {
            let name = text::decode(
                arguments
                    .next()
                    .ok_or_else(|| site.error(ErrorKind::Expected("DATA section name")))?,
                site,
                &mut total,
                limit,
            )?;
            if name.is_empty() || !insert(&mut names, name).map_err(|kind| site.error(kind))? {
                return Err(site.invalid("empty or duplicate DATA section name"));
            }
            let declared = strings(
                arguments
                    .next()
                    .ok_or_else(|| site.error(ErrorKind::Expected("DATA schema list")))?,
                &mut total,
                limit,
            )?;
            if declared.len() != 1
                || !unique_schemas.contains(declared[0].as_str())
                || arguments.next().is_some()
            {
                return Err(site.invalid("DATA must name exactly one declared schema"));
            }
        } else if exchange.sections().len() != 1 || schemas.len() != 1 {
            return Err(site.invalid("unnamed DATA requires one section and one schema"));
        }
    }
    Ok(Header {
        description: descriptions,
        implementation_level,
        name,
        timestamp,
        authors,
        organizations,
        preprocessor,
        originating_system,
        authorization,
        schemas,
    })
}
fn strings(value: Value<'_, '_>, total: &mut usize, limit: usize) -> Result<Vec<String>, Error> {
    let site = Site {
        id: None,
        offset: value.span().start,
    };
    let mut result = Vec::new();
    for value in aggregate(value, site)? {
        let decoded = text::decode(value, site, total, limit)?;
        push(&mut result, decoded).map_err(|kind| site.error(kind))?;
    }
    if result.is_empty() {
        return Err(site.error(ErrorKind::Expected("nonempty string list")));
    }
    Ok(result)
}
