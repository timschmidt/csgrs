from pathlib import Path
import hashlib, json, subprocess
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
results = json.loads((audit / 'chord-support-intersection-full-results.json').read_text())
assert len(results) == 49
assert not [f for row in results for f in row['new_failures']]
assert not any(row['timed_out'] for row in results)
assert all(row['returncode'] in {0,101} for row in results)
for suffix in ['', '-hyperbrep']:
    for kind in ['-check', '-test-build']:
        assert (audit / ('chord-support-intersection' + suffix + kind + '.exit')).read_text().strip() == '0'
rows = json.loads((audit / 'chord-support-intersection-source-before-tests.json').read_text())
changed = [row['file'] for row in rows if not (root / row['file']).is_file() or hashlib.sha256((root / row['file']).read_bytes()).hexdigest() != row['sha256']]
assert not changed, changed
edited = subprocess.check_output(['git','diff','--name-only'],cwd=root/'hypercurve').decode().splitlines()
format_command = ['rustfmt','--edition','2024','--config','skip_children=true','--check'] + [name for name in edited if name.endswith('.rs')]
subprocess.run(format_command,cwd=root/'hypercurve',check=True)
subprocess.run(['git','diff','--check'],cwd=root/'hypercurve',check=True)
probe = json.loads((audit / 'chord-support-intersection-generated-pairs-probe-final.json').read_text())
assert probe['returncode'] == 0 and not probe['unresolved_pairs'] and probe['xor_empty'] == 8
focused = json.loads((audit / 'chord-support-intersection-evidence-focused-results.json').read_text())
assert len(focused) == 4 and all(row['returncode'] == 0 for row in focused)
checks = {'source_files':len(rows),'changed_sources':changed,'formatting':True,'whitespace':True,'source_manifest':'chord-support-intersection-source-before-tests.json'}
(audit / 'chord-support-intersection-source-checks.json').write_text(json.dumps(checks,indent=2)+'\n')
qualification = {
 'status':'validated; awaiting incremental commit',
 'goal_status':'active',
 'base_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root/'hypercurve').decode().strip(),
 'change':[
  'Dispatch retained chord/chord and chord/rational-source curve and open-path pairs through shared finite geometry kernels.',
  'Move source-related and exact-linear chord replay out of the region adapter; preserve all open-curve endpoint contacts while region adjacency stays local.',
  'Retain typed rational, chord/rational and chord/chord overlap transport with original support authority.',
  'Standardize chord-pair ranges as paired boundaries in the first traversal order and update all internal consumers directly.',
  'Reuse monotone source-interval certificates for selected overlap clipping and finite-chord certificates for point evaluation; avoid root promotion and Cartesian incidence reconstruction.',
  'Preserve reusable chord/rational tangent and contact-location certificates on recursive affine-line parameters; decline the shortcut rather than overwrite prior specialized identity.',
  'Replace storage/dispatch-specific trim and offset assertions with independent exact endpoint, point and parameter-reuse checks while retaining direct recursive kernel coverage.'
 ],
 'regressions':{
  'generated_chords_keep_open_contacts_and_general_locations':'128 primary curve/open-path queries: crossing, source endpoint, authored endpoint and disjointness; both policies, reversals, operand orders; exact evaluation and subdivision.',
  'generated_chord_overlaps_retain_independent_and_selected_boundaries':'48 queries: independently generated chords, authored radical lines and selected tails; paired endpoint identity, relative orientation and evaluation.',
  'generated_chord_cuts_reenter_collinear_endpoint_intersections':'48 primary queries plus four setup intersections: contact -> chord split -> overlap -> line split -> exact collinear endpoint intersection.',
  'evidence_regressions':'Four focused tests pass, including the unchanged third-field tangent replay check, independent trim endpoints and direct recursive/common offset-line replay.'
 },
 'qualification':{
  'results':'chord-support-intersection-full-results.json','targets':len(results),
  **{key:sum(len(row[key]) for row in results) for key in ['passed','failed','ignored','new_failures']},
  'hypercurve_passed':sum(len(row['passed']) for row in results if row['repo']=='hypercurve'),
  'hyperbrep_passed':sum(len(row['passed']) for row in results if row['repo']=='hyperbrep'),
  'remaining_failures':sorted(f for row in results for f in row['failed']),
  'timeouts':0,'previously_unqualified_expensive_cases':8,'full_suite_passing':False,
  'all_target_checks':True,'immutable_sources':True,'source_files':len(rows),
  'formatting_and_whitespace_checks':True,'caller_audit':'chord-support-intersection-caller-audit.json'
 },
 'generated_pair_probe':{
  'record':'chord-support-intersection-generated-pairs-probe-final.json',
  'baseline_unresolved_pairs':18,'remaining_unresolved_pairs':0,'independent_region_empty_xors':8,
  'rlib_sha256':probe['rlib_sha256']
 },
 'evidence_audit':'chord-support-intersection-evidence-audit.md',
 'pre_evidence_qualification':'chord-support-intersection-pre-evidence-qualification/',
 'performance_claim':None,
 'remaining':[
  'Selected-circle and analytic-parallel common pair dispatch.',
  'Finite exterior source domains and complete partially coincident non-injective/retraced parameter relations.',
  'Inverse branch transport for a later narrower chord domain in a chord/rational correspondence.',
  'General curve/path split topology without native Bezier materialization.',
  'Nine pre-existing promotion failures and eight expensive unqualified cases.',
  'The broader full-family API, region-normalization and computational-closure architecture plan.'
 ]
}
(audit / 'chord-support-intersection-qualification.json').write_text(json.dumps(qualification,indent=2)+'\n')
print(json.dumps(qualification['qualification'],indent=2))
