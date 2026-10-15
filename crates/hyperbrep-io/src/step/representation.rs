use super::{
    Error, ErrorKind, ImportLimits, StepFile,
    decode::{Args, Resolver, Site, reference},
    error::{insert, push},
    imported::ImportedRepresentation,
};
use std::collections::HashSet;

pub(super) fn import(
    file: &StepFile<'_>,
    id: u64,
    limits: &ImportLimits,
) -> Result<ImportedRepresentation, Error> {
    if file.header.schemas.iter().any(|name| {
        !matches!(
            name.split_whitespace().next(),
            Some(
                "CONFIG_CONTROL_DESIGN"
                    | "AUTOMOTIVE_DESIGN"
                    | "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF"
            )
        )
    }) {
        return Err(Error::at(
            Some(id),
            0,
            ErrorKind::Unsupported("declared schema for the common AP203/AP214/AP242 subset"),
        ));
    }
    let mut resolver = Resolver::new(file, limits);
    let entity = resolver.entity(id)?;
    let site = Site::entity(entity);
    let record = entity.components().next().expect("syntax component");
    if entity.is_complex()
        || !matches!(
            record.name(),
            "FACETED_BREP_SHAPE_REPRESENTATION" | "ADVANCED_BREP_SHAPE_REPRESENTATION"
        )
    {
        return Err(site.error(ErrorKind::Unsupported("shape representation type")));
    }
    let advanced = record.name() == "ADVANCED_BREP_SHAPE_REPRESENTATION";
    let mut args = Args::new(record, site);
    let site = args.site;
    let name = resolver.text(args.next()?, site)?;
    let items = args.list()?;
    let units = resolver.context_units(args.reference()?)?;
    args.finish()?;
    let mut bodies = Vec::new();
    let mut placements = Vec::new();
    let mut item_ids = HashSet::new();
    for value in items {
        let item_id = reference(value, site)?;
        if !insert(&mut item_ids, item_id).map_err(|kind| site.error(kind))? {
            return Err(site.invalid("duplicate representation item"));
        }
        let item = resolver.entity(item_id)?;
        if item.is_complex() {
            return Err(
                Site::entity(item).error(ErrorKind::Unsupported("complex representation item"))
            );
        }
        match item.components().next().expect("syntax component").name() {
            "FACETED_BREP" if !advanced => {
                let body = super::faceted::body(&mut resolver, item_id, &units.length.scale_to_si)?;
                push(&mut bodies, body).map_err(|kind| site.error(kind))?;
            }
            "MANIFOLD_SOLID_BREP" if advanced => {
                let body =
                    super::advanced::body(&mut resolver, item_id, &units.length.scale_to_si)?;
                push(&mut bodies, body).map_err(|kind| site.error(kind))?;
            }
            "AXIS2_PLACEMENT_3D" => {
                resolver.placement(item_id, &units.length.scale_to_si)?;
                push(&mut placements, item_id).map_err(|kind| site.error(kind))?;
            }
            _ => {
                return Err(
                    Site::entity(item).error(ErrorKind::Unsupported("shape representation item"))
                );
            }
        }
    }
    if bodies.is_empty() {
        return Err(site.invalid("shape representation has no bodies"));
    }
    resolver.used.sort_unstable();
    resolver.used.dedup();
    let mut retained = Vec::new();
    for entity in file.syntax.entities() {
        if resolver.used.binary_search(&entity.id()).is_err() {
            push(&mut retained, entity.id()).map_err(|kind| site.error(kind))?;
        }
    }
    Ok(ImportedRepresentation {
        source: id,
        name,
        units,
        bodies,
        interpreted: resolver.used,
        retained,
        placements,
    })
}
