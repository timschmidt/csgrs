from pathlib import Path
import hashlib,json,subprocess
A=Path(__file__).resolve().parent;W=A.parent;old=A/'topology-hint-removal-candidate-v770';C=A/'topology-hint-removal-candidate-v772';assert not C.exists()
r=json.loads((A/'topology-hint-removal-v771-terminal.json').read_text());reaped=json.loads((A/'topology-hint-removal-v771-reaped.json').read_text());assert reaped['outer_exit_code']==1 and r['all_processes_reaped']and not r['cases']
current=json.loads((A/r['source_manifest']).read_text());base=json.loads((A/'compound-fill-v766-sources.json').read_text());promotion=json.loads((A/'topology-hint-removal-promotion-v771.json').read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name,sha in current.items():assert digest(W/name)==sha,name
sources={n:(W/n).read_text()for n in promotion['promoted']}
name='hypercurve/src/lib.rs';line='pub(crate) use bezier_region::CurveBoundaryInteriorSide2;\n';assert sources[name].count(line)==1;sources[name]=sources[name].replace(line,'')
for name in current:
 if not name.endswith('.rs'):continue
 s=sources.get(name,(W/name).read_text())
 if 'crate::CurveBoundaryInteriorSide2'in s:sources[name]=s.replace('crate::CurveBoundaryInteriorSide2','crate::bezier_region::CurveBoundaryInteriorSide2')
for name in ['hypercurve/src/bezier_offset.rs','hypercurve/src/curve_region_trim.rs']:
 s=sources.get(name,(W/name).read_text());position=s.index('CurveBoundaryInteriorSide2,');start=s.rfind('    use crate::{',0,position);assert start>=0
 s=s[:position]+s[position:].replace('CurveBoundaryInteriorSide2, ','',1)
 s=s[:start]+'    use crate::bezier_region::CurveBoundaryInteriorSide2;\n'+s[start:];sources[name]=s
name='hypercurve/tests/hypercurve_curve_intersection.rs';s=sources[name];old_loop='for second in [&same, &reversed] {';assert s.count(old_loop)==1;s=s.replace(old_loop,'for (second, second_is_reversed) in [(&same, false), (&reversed, true)] {')
old_assert='''            assert_eq!(
                decided(region.signed_area(&policy).unwrap().into_value()),
                Some(expected_area),
                "{operation:?}, second side {second_side:?}"
            );'''
new_assert='''            let area = decided(region.signed_area(&policy).unwrap().into_value())
                .expect("the polygon Boolean has a represented area");
            assert_eq!(
                area.partial_cmp(&expected_area),
                Some(std::cmp::Ordering::Equal),
                "{operation:?}, reversed second={second_is_reversed}"
            );'''
assert s.count(old_assert)==1;sources[name]=s.replace(old_assert,new_assert)
for name,s in sources.items():p=C/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s)
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',*[str(C/n)for n in sorted(sources)if n.endswith('.rs')]],cwd=W/'hypercurve',check=True)
for name,sha in current.items():assert digest(W/name)==sha,name
for name in sources:(W/name).write_bytes((C/name).read_bytes())
(A/'topology-hint-removal-base-v772.json').write_text(json.dumps({n:base[n]for n in sources},indent=2)+'\n')
(A/'topology-hint-removal-promotion-v772.json').write_text(json.dumps(dict(candidates={n:str(C/n)for n in sources},base={n:base[n]for n in sources},promoted={n:digest(W/n)for n in sources},parents=promotion['parents'],prior_failed_qualification='topology-hint-removal-v771-terminal.json'),indent=2)+'\n')
print('Corrected stale test diagnostic and private type imports;promoted',len(sources),'paths')
