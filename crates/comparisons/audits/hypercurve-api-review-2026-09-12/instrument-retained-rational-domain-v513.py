def instrument(s):
    for name,end_name in [('zero_distance_pair_intersections_in_domain','self_intersections_in_domain'),('source_oriented_regularized_tangent_field','source_oriented_regularized_tangent_field_at_interior'),('parameter_domain_constraint','prepend_parallel_pair_projection')]:
        a=s.index('fn '+name+'('); b=s.index('fn '+end_name+'(',a+10)
        part=s[a:b]; needle='return Ok(Classification::Uncertain(reason))'
        chunks=part.split(needle)
        part=chunks[0]
        for i,chunk in enumerate(chunks[1:]):
            part+='{ eprintln!("v513 '+name+' checkpoint '+str(i)+': {reason:?}"); '+needle+'; }'+chunk
        s=s[:a]+part+s[b:]
    a=s.index('    fn rational_parallel_pair_intersections(');b=s.index('    /// Reuses native unit projection',a)
    part=s[a:b]
    part=part.replace('        if covered_by_unit {','        eprintln!("v513 complete_unit={complete_unit} covered_by_unit={covered_by_unit}");\n        if covered_by_unit {',1)
    needle='            let native = self.rational_unit_parallel_pair_intersections(other, ranges, policy);'
    assert needle in part
    part=part.replace(needle,needle+'\n            match &native {\n                Ok(Classification::Decided(Some(result))) => eprintln!("v513 native complete={} contacts={} overlaps={} products={}",result.is_complete(),result.contacts().len(),result.overlaps().len(),result.parameter_components().len()),\n                Ok(Classification::Decided(None)) => eprintln!("v513 native absent"),\n                Ok(Classification::Uncertain(reason)) => eprintln!("v513 native uncertain {reason:?}"),\n                Err(_) => eprintln!("v513 native error"),\n            }\n')
    needle='            let regular_parallel_range = first_image.is_none().then_some(ranges[0]);'
    part=part.replace(needle,'            eprintln!("v513 first_image={} swapped={swapped} degrees={}/{}", first_image.is_some(),parallel.source_degree(),zero.source_degree());\n'+needle)
    return s[:a]+part+s[b:]
