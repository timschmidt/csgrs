use std::fmt::Write;

/// Forward unit chain with a shared dimension record, authored independently of the mapper.
pub fn unit_chain(depth: usize, factor: &str) -> String {
    let mut source = String::from(
        "ISO-10303-21;HEADER;FILE_DESCRIPTION(('unit graph'),'2;1');FILE_NAME('units.step','2026-10-15T00:00:00',('tests'),('tests'),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC;DATA;\n#2=DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.);\n#3=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));\n",
    );
    let start = if depth == 0 { 3 } else { 100 };
    writeln!(source,"#1=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#{start}))REPRESENTATION_CONTEXT('','3D'));").unwrap();
    for index in 0..depth {
        let id = 100 + 2 * index;
        let measure = id + 1;
        let next = if index + 1 == depth { 3 } else { id + 2 };
        writeln!(source,"#{id}=(CONVERSION_BASED_UNIT('declared',#{measure})LENGTH_UNIT()NAMED_UNIT(#2));\n#{measure}=MEASURE_WITH_UNIT(LENGTH_MEASURE({factor}),#{next});").unwrap();
    }
    source.push_str("ENDSEC;END-ISO-10303-21;");
    source
}
