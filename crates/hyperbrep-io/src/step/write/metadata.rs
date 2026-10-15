use super::{StepWriteOptions, WriteError, WriteErrorKind, number, output::Output};

pub(super) fn header(out: &mut Output, options: &StepWriteOptions<'_>) -> Result<(), WriteError> {
    out.put("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Exact selected faceted geometry'),'2;1');\nFILE_NAME(")?;
    out.string(options.file_name)?;
    out.put(",")?;
    out.string(options.timestamp)?;
    out.put(",(")?;
    out.string(options.author)?;
    out.put("),(")?;
    out.string(options.organization)?;
    out.put("),")?;
    out.string(concat!("hyperbrep-io ", env!("CARGO_PKG_VERSION")))?;
    out.put(",'HyperBRep','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n")
}
pub(super) fn context(out: &mut Output, options: &StepWriteOptions<'_>) -> Result<u64, WriteError> {
    let scale = number::exact(&options.length_unit.metres_per_unit, &out.limits, out.site)?;
    if scale.is_zero() || scale.is_negative() {
        return Err(out.site.error(WriteErrorKind::Invalid(
            "output unit scale must be positive",
        )));
    }
    let metre = out.entity("", |out| {
        out.put("LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.)")
    })?;
    let dimensions = out.entity("DIMENSIONAL_EXPONENTS", |out| {
        out.put("1.,0.,0.,0.,0.,0.,0.")
    })?;
    let measure = out.entity("MEASURE_WITH_UNIT", |out| {
        out.put("LENGTH_MEASURE(")?;
        out.number(&options.length_unit.metres_per_unit)?;
        out.put("),")?;
        out.reference(metre)
    })?;
    let length = out.entity("", |out| {
        out.put("CONVERSION_BASED_UNIT(")?;
        out.string(options.length_unit.name)?;
        out.put(",")?;
        out.reference(measure)?;
        out.put(")LENGTH_UNIT()NAMED_UNIT(")?;
        out.reference(dimensions)?;
        out.put(")")
    })?;
    let angle = out.entity("", |out| {
        out.put("NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.)")
    })?;
    let solid_angle = out.entity("", |out| {
        out.put("NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT()")
    })?;
    out.entity("", |out| {
        out.put("GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT(")?;
        out.references(&[length, angle, solid_angle])?;
        out.put(")REPRESENTATION_CONTEXT('','3D')")
    })
}
pub(super) fn product(out: &mut Output, name: &str, representation: u64) -> Result<(), WriteError> {
    let application = out.entity("APPLICATION_CONTEXT", |out| out.string("mechanical design"))?;
    out.entity("APPLICATION_PROTOCOL_DEFINITION", |out| {
        out.put("'international standard','automotive_design',2000,")?;
        out.reference(application)
    })?;
    let product_context = out.entity("PRODUCT_CONTEXT", |out| {
        out.put("'',")?;
        out.reference(application)?;
        out.put(",'mechanical'")
    })?;
    let product = out.entity("PRODUCT", |out| {
        out.put("'shape',")?;
        out.string(name)?;
        out.put(",'',")?;
        out.references(&[product_context])
    })?;
    let formation = out.entity("PRODUCT_DEFINITION_FORMATION", |out| {
        out.put("'','',")?;
        out.reference(product)
    })?;
    let context = out.entity("PRODUCT_DEFINITION_CONTEXT", |out| {
        out.put("'part definition',")?;
        out.reference(application)?;
        out.put(",'design'")
    })?;
    let definition = out.entity("PRODUCT_DEFINITION", |out| {
        out.put("'design','',")?;
        out.reference(formation)?;
        out.put(",")?;
        out.reference(context)
    })?;
    let shape = out.entity("PRODUCT_DEFINITION_SHAPE", |out| {
        out.put("'','',")?;
        out.reference(definition)
    })?;
    out.entity("SHAPE_DEFINITION_REPRESENTATION", |out| {
        out.reference(shape)?;
        out.put(",")?;
        out.reference(representation)
    })?;
    Ok(())
}
