from pathlib import Path
A=Path(__file__).resolve().parent
t=(A/'probe-fillet-reoffset-diagnostic-20260926-v238.py').read_text()
t=t.replace('fillet-reoffset-diagnostic-20260926-v238','fillet-replay-diagnostic-20260926-v249').replace('public-fillet-families-full-20260926-v236-terminal.json','retained-fiber-refinement-20260926-v248-terminal.json').replace('public-fillet-families-full-20260926-v236-sources.json','retained-fiber-refinement-20260926-v248-sources.json')
t=t.replace("archive=A/'source-archives'/prefix;", "guard={name:sha for name,sha in guard.items()if not name.startswith('fiber-probe/')};manifest=dict(guard);archive=A/'source-archives'/prefix;",1)
t=t.replace("parent_commit=prior['parents']['hypercurve']", "parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip()")
t=t.replace('timeout=180','timeout=240')
needle='t=t[:start]+part+t[end:]\nstart=t.index(\'    pub(crate) fn parallel_intersections('
assert needle in t
insert=r'''part=part.replace('        let (mut projection_identically_zero, mut candidates) =', '        eprintln!("selected-normal isolate-begin");\n        let (mut projection_identically_zero, mut candidates) =',1)
part=part.replace('        if !projection_identically_zero && let Some(incident) = incident {', '        eprintln!("selected-normal isolate-end candidates={} component={}", candidates.len(), projection_identically_zero);\n        if !projection_identically_zero && let Some(incident) = incident {',1)
part=part.replace('        for candidate in candidates {', '        for (candidate_index, candidate) in candidates.into_iter().enumerate() {\n            eprintln!("selected-normal candidate={} circle-begin", candidate_index);',1)
part=part.replace('            match circle_sign {', '            eprintln!("selected-normal candidate={} circle-end sign={:?}", candidate_index, circle_sign);\n            match circle_sign {',1)
part=part.replace('            let selected = match candidate.radical_sum_sign(', '            eprintln!("selected-normal candidate={} half-plane-begin", candidate_index);\n            let selected = match candidate.radical_sum_sign(',1)
part=part.replace('            let location = match selected {', '            eprintln!("selected-normal candidate={} half-plane-end sign={:?}", candidate_index, selected);\n            let location = match selected {',1)
part=part.replace('            let tangent_cross_source = match candidate.radical_sum_sign(', '            eprintln!("selected-normal candidate={} tangent-begin", candidate_index);\n            let tangent_cross_source = match candidate.radical_sum_sign(',1)
part=part.replace('            let derivative_scale = match other.parallel_derivative_scale_sign(', '            eprintln!("selected-normal candidate={} tangent-end sign={:?}", candidate_index, tangent_cross_source);\n            let derivative_scale = match other.parallel_derivative_scale_sign(',1)
part=part.replace('            retained.push((candidate, location, tangent_cross_sign));', '            eprintln!("selected-normal candidate={} retained", candidate_index);\n            retained.push((candidate, location, tangent_cross_sign));',1)
'''
t=t.replace(needle,insert+'\n'+needle,1)
needle="p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()"
more=r'''
start=t.index('fn algebraic_selected_fiber_root_predicate_sign(');end=t.index('fn validate_selected_fiber_pair_base(',start);part=t[start:end]
marker=') -> CurveResult<Classification<RealSign>> {';assert marker in part
part=part.replace(marker,marker+r"""
    let probe = authority.data.incidence.coefficients.len() == 6 && authority.data.incidence.coefficients.iter().map(Vec::len).max() == Some(21);
    if probe { eprintln!("selected-predicate begin rows={} cols={}", predicate.coefficients.len(), predicate.coefficients.iter().map(Vec::len).max().unwrap_or(0)); }
""",1)
part=part.replace('    loop {\n        if refinement_steps != 0 {', '    loop {\n        if probe { eprintln!("selected-predicate iteration refinements={}", refinement_steps); }\n        if refinement_steps != 0 {',1)
part=part.replace('            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(', '            if probe { eprintln!("selected-predicate common-fiber begin refinements={}", refinement_steps); }\n            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(',1)
part=part.replace('            exact_nonzero = zero_report.certainty', '            if probe { eprintln!("selected-predicate common-fiber end status={:?} count={:?}", zero_report.status, zero_report.distinct_root_count); }\n            exact_nonzero = zero_report.certainty',1)
t=t[:start]+part+t[end:]
start=t.index('fn algebraic_selected_fiber_root_interval_refined(');end=t.index('fn algebraic_selected_fiber_root_predicate_sign(',start);part=t[start:end]
marker=') -> CurveResult<Classification<IsolatedRootInterval>> {';assert marker in part
part=part.replace(marker,marker+r"""
    let probe = authority.data.incidence.coefficients.len() == 6 && authority.data.incidence.coefficients.iter().map(Vec::len).max() == Some(21);
    if probe { eprintln!("selected-refine begin steps={}", refinement_steps); }
""",1)
part=part.replace('    match report.status {', '    if probe { eprintln!("selected-refine end status={:?} retained-refinements={}", report.status, report.retained_refinement_steps); }\n    match report.status {',1)
t=t[:start]+part+t[end:]
'''
t=t.replace(needle,more+'\n'+needle,1)
(A/'probe-fillet-replay-diagnostic-20260926-v249.py').write_text(t)
print('Prepared copied-source region replay trace')
