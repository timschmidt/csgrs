file = r/'src/curve_region_boolean.rs'
s = file.read_text()
start = s.index('fn parameter_location_in_carrier(\n')
end = s.index('\nfn ranges_intersect(', start)
block = s[start:end]
block = substitute(block, '        let point = parameter\n', '        corner_trace!("circle location point construction enter");\n        let point = parameter\n')
block = substitute(block, '        if let Classification::Decided(Some(point)) = point {', '        corner_trace!("circle location point construction done");\n        if let Classification::Decided(Some(point)) = point {')
block = substitute(block, '            let location = fragment\n', '            corner_trace!("circle location physical enter");\n            let location = fragment\n')
block = substitute(block, '            match location {', '            corner_trace!("circle location physical done {location:?}");\n            match location {')
block = substitute(block, '        return match fragment\n', '        corner_trace!("circle location final parameter order enter");\n        return match fragment\n')
file.write_text(s[:start]+block+s[end:])

file = r/'src/bezier_offset.rs'
s = subprocess.check_output(['git', 'show', 'HEAD:src/bezier_offset.rs'], cwd=w, text=True)
start = s.index('    pub(crate) fn certified_incident_point_evidence_location(\n')
end = s.index('    fn incident_location_from_orders(', start)
block = s[start:end]
block = substitute(block, '        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Start};', '''        let trace = |stage: &str| {
            if std::env::var_os("HYPERCURVE_CORNER_DIAGNOSTIC").is_some() { eprintln!("circle location {stage}"); }
        };
        trace("endpoint identities enter");
        use BezierAlgebraicCuspSemicircleIncidentLocation2::{End, Start};''')
block = substitute(block, '            let endpoint = match self.endpoint_point_evidence(at_start, policy)? {', '''            trace(if at_start { "start point enter" } else { "end point enter" });
            let endpoint = match self.endpoint_point_evidence(at_start, policy)? {''')
block = substitute(block, '            if point.shares_storage(&endpoint) {', '            trace("endpoint point ready");\n            if point.shares_storage(&endpoint) {')
block = substitute(block, '        let endpoint_side =\n', '        trace("bounded chord side enter");\n        let endpoint_side =\n')
block = substitute(block, '        if let Classification::Decided(side) = endpoint_side', '        trace("bounded chord side done");\n        if let Classification::Decided(side) = endpoint_side')
block = substitute(block, '            let endpoint_parameter = if source_start {', '            trace(if source_start { "start order enter" } else { "end order enter" });\n            let endpoint_parameter = if source_start {')
block = substitute(block, '            if matches!(scalar_order, Classification::Decided(_)) {', '            trace("bounded scalar order done");\n            if matches!(scalar_order, Classification::Decided(_)) {')
block = substitute(block, '            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {', '            trace("shared parametric point enter");\n            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {')
block = substitute(block, '            let endpoint = match self.endpoint_point_evidence(\n', '            trace("shared parametric point done; endpoint construction enter");\n            let endpoint = match self.endpoint_point_evidence(\n')
block = substitute(block, '            let projective = recursive_projective_incident_point_order(', '            trace("recursive point order enter");\n            let projective = recursive_projective_incident_point_order(')
block = substitute(block, '            if let Some(Classification::Decided(order)) = projective {', '            trace("recursive point order done");\n            if let Some(Classification::Decided(order)) = projective {')
block = substitute(block, '            parameter.cmp_by_refinement(endpoint_parameter, policy)', '            trace("complete parameter order enter");\n            parameter.cmp_by_refinement(endpoint_parameter, policy)', 2)
file.write_text(s[:start]+block+s[end:])
(a/(prefix+'-bezier_offset.rs')).write_text(file.read_text())
