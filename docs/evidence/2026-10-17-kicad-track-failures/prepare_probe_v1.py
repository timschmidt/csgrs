"""Keep malformed cases independent and retain separate unresolved fidelity cases."""
import json,hashlib,shutil
from pathlib import Path
W=Path(__file__).resolve().parent;P=W/'prototype-v1';P.mkdir(exist_ok=False)
cases=[]
def add(name,fields,expected):cases.append({'name':name,'fields':fields,'expected':expected})
base={'start':'(start 10 20)','end':'(end 12 20)','width':'(width 2)'}
add('valid_control',' '.join(base.values()),'valid_control')
add('zero_width','(start 10 20) (end 12 20) (width 0)','empty')
for field in base:
    values={
        'missing':'','empty':f'({field})','bad':f'({field} bad'+(' 20)' if field!='width' else ')'),
        'extra':f'({field} 10 20 30)' if field!='width' else '(width 2 3)',
        'duplicate':base[field]+' '+base[field],
        'nested':f'({field} (value 10)'+(' 20)' if field!='width' else ')'),
        'nan':f'({field} NaN'+(' 20)' if field!='width' else ')'),
        'infinity':f'({field} inf'+(' 20)' if field!='width' else ')'),
    }
    if field!='width':values|={'short':f'({field} 10)','bad_y':f'({field} 10 bad)'}
    for kind,value in values.items():
        add(field+'_'+kind,' '.join(value if k==field else v for k,v in base.items()),'invalid_input')
for name,width in [('negative','-1'),('negative_subrange','-1e-400')]:
    add('width_'+name,f'(start 10 20) (end 12 20) (width {width})','invalid_input')
add('zero_width_missing_end','(start 10 20) (width 0)','invalid_input')
add('zero_width_invalid_start','(start bad 20) (end 12 20) (width 0)','invalid_input')
for name,fields in [
    ('coincident_positive','(start 3 4) (end 3 4) (width 2)'),
    ('positive_width_underflow','(start 0 0) (end 2 0) (width 1e-400)'),
    ('positive_length_underflow','(start 0 0) (end 1e-200 0) (width 1e-200)'),
    ('superrange_coordinate','(start 1e400 0) (end 1e400 2) (width 2)'),
    ('endpoint_difference_overflow','(start -1e308 0) (end 1e308 0) (width 2)'),
    ('width_overflow','(start 0 0) (end 2 0) (width 1e400)'),
    ('finite_translation_collapse','(start 1e20 0) (end 100000000000000000001 0) (width 1)'),
]:add(name,fields,'unresolved_nonempty_fidelity_case')
add('zero_width_superrange','(start 1e400 0) (end 1e400 2) (width 0)','empty')
(P/'cases.json').write_text(json.dumps(cases,indent=2)+'\n')
source=W.parents[2]/'docs/evidence/2026-10-16-native-change-disposition/pending-source-v1/crates/hyperdrc/tests/native_linear_strokes.rs'
shutil.copy2(source,P/'original_native_linear_strokes.rs')
(P/'manifest.json').write_text(json.dumps({'cases':len(cases),'invalid_input_cases':sum(c['expected']=='invalid_input' for c in cases),
    'original_complete_regressions_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
    'case_meaning':'Invalid source requires retained failure. The separate nonempty fidelity cases remain unresolved unless independently proven; an error is not a successful geometric repair.'},indent=2)+'\n')
print(json.dumps({'cases':len(cases),'invalid_input_cases':sum(c['expected']=='invalid_input' for c in cases)}),flush=True)
