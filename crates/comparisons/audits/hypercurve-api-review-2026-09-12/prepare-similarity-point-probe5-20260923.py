from pathlib import Path
import hashlib,json,shutil,subprocess

A=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-similarity-point-probe4-2026-09-23');root=Path('/tmp/hypercurve-similarity-point-probe5-2026-09-23')
assert json.loads((A/'similarity-point-20260923-probe4-terminal.json').read_text())['all_processes_reaped']
assert not root.exists();bindings=json.loads((A/'similarity-point-20260923-probe4-sources.json').read_text())
for name,sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    (root/name).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source/name,root/name)

p=root/'hyperreal/src/lib.rs'
with p.open('a') as out:out.write('''
static DIAGNOSTIC_TOWER_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub fn diagnostic_enable_tower_probe() { DIAGNOSTIC_TOWER_ENABLED.store(true,std::sync::atomic::Ordering::Relaxed); }
pub fn diagnostic_tower_probe(message: &str) {
    static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    if DIAGNOSTIC_TOWER_ENABLED.load(std::sync::atomic::Ordering::Relaxed)
        && COUNT.fetch_add(1,std::sync::atomic::Ordering::Relaxed)<96 {
        eprintln!("tower probe: {message}");
    }
}
''')
p=root/'hyperreal/src/computable/node/quadratic_tower.rs';s=p.read_text()
old='''    let mut budget = TOWER_NODE_BUDGET;
    parse(value, &mut budget, &mut Vec::new())'''
new='''    let mut budget = TOWER_NODE_BUDGET;
    let mut memo=Vec::new();
    let result=parse(value,&mut budget,&mut memo);
    if result.is_none() { crate::diagnostic_tower_probe(&format!("parse declined: budget={budget}, memo={}",memo.len())); }
    result'''
assert s.count(old)==1;s=s.replace(old,new)
old='''        if radicand.scale.sign() != Sign::NoSign {
            return None;
        }'''
new='''        if radicand.scale.sign() != Sign::NoSign {
            crate::diagnostic_tower_probe("common basis declined nested radicand");
            return None;
        }'''
assert s.count(old)==1;s=s.replace(old,new)
old='''                if coefficient.disc.is_some() && coefficient.disc.as_ref() != inner {
                    return None;
                }'''
new='''                if coefficient.disc.is_some() && coefficient.disc.as_ref() != inner {
                    crate::diagnostic_tower_probe("common basis declined third square class");
                    return None;
                }'''
assert s.count(old)==1;s=s.replace(old,new)
a=s.index('fn admit_rational(');b=s.index('fn rational_sign(',a);part=s[a:b]
part=part.replace('''    } else {
        None''','''    } else {
        crate::diagnostic_tower_probe(&format!("rational bound: numerator={}, denominator={}",value.numerator().bits(),value.denominator().bits()));
        None''');s=s[:a]+part+s[b:]
p.write_text(s)

p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text()
a=s.index('fn one_field_common_zero_parameter_candidates(');b=s.index('fn one_field_point_parameter_candidates(',a);part=s[a:b]
old='''                last_reason = reason;
                continue;''';assert part.count(old)==1
part=part.replace(old,'''                point_inverse_probe("common zero projection",line!(),&format!("{reason:?}"));
'''+old)
old='''                        retry_reason = Some(reason);
                        break;''';assert part.count(old)==1
part=part.replace(old,'''                        point_inverse_probe("common zero normal branch",line!(),&format!("{reason:?}"));
'''+old)
old='''                    retry_reason = Some(reason);
                    break;''';assert part.count(old)==1
part=part.replace(old,'''                    point_inverse_probe("common zero incidence replay",line!(),&format!("{reason:?}"));
'''+old)
s=s[:a]+part+s[b:]
a=s.index('fn parametric_point_parameter_candidates(');b=s.index('impl BezierAlgebraicCuspSemicircleMappedParameterData2 {',a);part=s[a:b]
old='    let represented = match parameter {';assert part.count(old)==1
part=part.replace(old,'''    if let BezierAlgebraicCuspSemicircleMappedPointParameterRef2::Selected(value)=parameter {
        point_inverse_probe("alternate scalar witness",line!(),match value.data.represented_parameter.get() {
            Some(BezierParameter2::Exact(_))=>"exact",Some(BezierParameter2::Algebraic(_))=>"algebraic",None=>"absent"
        });
    }
'''+old)
s=s[:a]+part+s[b:]
old='''                let analytic_target = match analytic_overlap
                    .other_parameter_for_cusp(transformed.end_parameter(), &policy)''';assert s.count(old)==1
s=s.replace(old,'''                if chamfer { hyperreal::diagnostic_enable_tower_probe(); }
'''+old)
p.write_text(s)
changed=['hyperreal/src/lib.rs','hyperreal/src/computable/node/quadratic_tower.rs','hypercurve/src/bezier_offset.rs']
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true',*[str(root/name) for name in changed]],check=True)
for name in changed:bindings[name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
(A/'similarity-point-20260923-probe5-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
runner=(A/'run-similarity-point-probe2-20260923.py').read_text().replace('/tmp/hypercurve-similarity-point-probe2-2026-09-23',str(root)).replace('similarity-point-20260923-probe2','similarity-point-20260923-probe5')
a=runner.index('checks=[]');b=runner.index("command=[cargo,'test'",a);runner=runner[:a]+'checks=[]\n'+runner[b:]
(A/'run-similarity-point-probe5-20260923.py').write_text(runner)
print('Prepared projection/branch/replay, witness-presence and bounded tower diagnostics.')
