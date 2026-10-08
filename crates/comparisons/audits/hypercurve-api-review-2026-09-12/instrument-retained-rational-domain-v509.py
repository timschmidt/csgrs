def instrument(s):
    for name,end_name in [('zero_distance_pair_intersections_in_domain','self_intersections_in_domain'),('source_oriented_regularized_tangent_field','source_oriented_regularized_tangent_field_at_interior')]:
        a=s.index('    fn '+name+'('); b=s.index('fn '+end_name+'(',a+10)
        part=s[a:b]; needle='return Ok(Classification::Uncertain(reason));'
        chunks=part.split(needle)
        part=chunks[0]
        for i,chunk in enumerate(chunks[1:]):
            part+='eprintln!("v509 '+name+' checkpoint '+str(i)+': {reason:?}"); '+needle+chunk
        s=s[:a]+part+s[b:]
    a=s.index('    fn source_oriented_regularized_tangent_field_at_interior(');b=s.index('    fn source_constant_tangent_field_at_interior(',a)
    part=s[a:b].replace('Classification::Decided(None) => {','Classification::Decided(None) => { eprintln!("v509 hodograph GCD and constant direction unavailable");',1)
    s=s[:a]+part+s[b:]
    a=s.index('    fn rational_parallel_pair_intersections(');b=s.index('    fn replay_parallel_pair_projection(',a)
    part=s[a:b];needle='            let regular_parallel_range = first_image.is_none().then_some(ranges[0]);'
    assert needle in part
    part=part.replace(needle,'            eprintln!("v509 rational domain first_image={} degrees={}/{} bounds={}/{}/{}/{}",\n                first_image.is_some(), parallel.source_degree(), zero.source_degree(),\n                ranges[0].start().finite_envelope_bounds().is_some(), ranges[0].end().finite_envelope_bounds().is_some(),\n                ranges[1].start().finite_envelope_bounds().is_some(), ranges[1].end().finite_envelope_bounds().is_some());\n'+needle)
    return s[:a]+part+s[b:]
