from pathlib import Path
import hashlib, json, math, subprocess, time
import sympy as s

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'packed-determinant-20260926-v139'
manifest_name = 'local-chord-complete-replay-20260924-v139-sources.json'
manifest = json.loads((A/manifest_name).read_text())
archive = A/'source-archives/hypercurve-local-chord-complete-replay-v139-20260924'
parent = subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip()
old = subprocess.check_output(['git','show','HEAD:src/bareiss.rs'],cwd=W/'hypersolve',text=True)
new = (W/'hypersolve/src/bareiss.rs').read_text()
integer = (W/'hypersolve/src/integer_interpolation.rs').read_text()

def extract(text,start,end):
    a=text.index(start)
    return text[a:text.index(end,a)]

old = extract(old,'pub(crate) fn determinant_integer_polynomial_matrix(', '\n#[cfg(test)]')
new = extract(new,'pub(crate) fn determinant_integer_polynomial_matrix(', '\n#[cfg(test)]')
product = extract(integer,'pub(crate) fn integer_polynomial_product(', '\npub(crate) fn integer_polynomial_exact_quotient(')
quotient = extract(integer,'pub(crate) fn integer_polynomial_exact_quotient(', '\nfn previous_prime(')
zero = extract(integer,'fn is_zero_integer_polynomial(', '\n}')+'\n}'

# The same quartic's two radical projection equations, with the structural
# diagonal removed. This independently constructs a representative Bezout
# matrix; no timed sample includes symbolic setup or input parsing.
t,u=s.symbols('t u')
p=s.Matrix([12*t*(1-t)*(1-2*t),18*t*t*(1-t)*(1-t)])
q=p.subs(t,u);a=p.diff(t);b=q.diff(u);delta=q-p
cross=s.det(s.Matrix.hstack(a,b))
equations=[s.Poly(s.div(s.expand(speed*projection**2-cross**2/16),(t-u)**2,t,u)[0],u)
           for speed,projection in [(a.dot(a),delta.dot(b)),(b.dot(b),delta.dot(a))]]
n=12
assert all(e.degree()==n for e in equations)
matrix=[s.Integer(0)]*(n*n)
for high in range(1,n+1):
    for low in range(high):
        value=equations[0].nth(high)*equations[1].nth(low)-equations[0].nth(low)*equations[1].nth(high)
        for offset in range(high-low):
            matrix[(high-1-offset)*n+low+offset] += value
matrix=[list(reversed(s.Poly(value,t).all_coeffs())) for value in matrix]
for row in range(n):
    content=math.gcd(*(int(value) for entry in matrix[row*n:(row+1)*n] for value in entry))
    assert content
    matrix[row*n:(row+1)*n]=[[int(value)//content for value in entry] for entry in matrix[row*n:(row+1)*n]]
matrix_path=A/f'{prefix}-matrix.json'
matrix_path.write_text(json.dumps(matrix)+'\n')
matrix_rust='vec!['+','.join('"'+','.join(map(str,entry))+'"' for entry in matrix)+']'
source='use num::{BigInt,Integer,One,Zero};\n'+zero+'\n'+quotient+'\n'+product
source+='\nmod old { use super::*;\n'+old+'\n}\nmod new { use super::*;\n'+new+'\n}\n'
source+='''fn main() {
    let args=std::env::args().collect::<Vec<_>>();
    let (dimension,entries) = if args[2] == "small" {
        (3, (0..9).map(|i| vec![BigInt::from(i+1),BigInt::from(i*i+3),BigInt::from(i%3)]).collect::<Vec<_>>())
    } else { (12, '''+matrix_rust+'''.iter().map(|entry| entry.split(',').map(|v| BigInt::parse_bytes(v.as_bytes(),10).unwrap()).collect()).collect()) };
    let start=std::time::Instant::now();
    let determinant=if args[1] == "old" { old::determinant_integer_polynomial_matrix(&entries,dimension) }
        else { new::determinant_integer_polynomial_matrix(&entries,dimension) }.unwrap();
    eprintln!("elapsed-ns={} coefficients={}",start.elapsed().as_nanos(),determinant.len());
    for coefficient in determinant { println!("{}",coefficient); }
}
'''
source_path=A/f'{prefix}.rs'
source_path.write_text(source)
rows=[json.loads(line) for line in (A/'packed-integer-product-20260926-v138-hypersolve-build.jsonl').read_text().splitlines()]
num=next(row for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='num')
rlib=next(Path(name) for name in num['filenames'] if name.endswith('.rlib'))
binary=A/prefix
report=dict(parent=parent,source_manifest=manifest_name,source_directory=str(archive),
            inputs={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in [source_path,matrix_path,rlib]},
            old_function_sha256=hashlib.sha256(old.encode()).hexdigest(),new_function_sha256=hashlib.sha256(new.encode()).hexdigest(),
            cases=[],all_processes_reaped=False,diagnostic_only=True)

def verify():
    for name,sha in manifest.items():
        for root in [W,archive]:
            assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
    for name,sha in report['inputs'].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==sha,name
    if 'binary_sha256' in report:
        assert hashlib.sha256(binary.read_bytes()).hexdigest()==report['binary_sha256']

def save():
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')

verify()
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source_path),'--extern','num='+str(rlib),'-L','dependency='+str(rlib.parent),'-o',str(binary)]
report['compile_command']=command
with (A/f'{prefix}-build.log').open('w') as out:
    report['compile_returncode']=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
save()
assert report['compile_returncode']==0
report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
for kind in ['small','quartic']:
    for implementation in ['old','new']:
        label=kind+'-'+implementation
        report['active']=label;save()
        output=A/f'{prefix}-{label}.txt'
        stderr=A/f'{prefix}-{label}.log'
        start=time.monotonic()
        with output.open('w') as out,stderr.open('w') as err:
            try:
                code=subprocess.run([str(binary),implementation,kind],stdout=out,stderr=err,timeout=90).returncode
            except subprocess.TimeoutExpired:
                code='timeout'
        row=dict(label=label,returncode=code,elapsed_seconds=time.monotonic()-start,
                 output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),log=stderr.name)
        report['cases'].append(row);report.pop('active');save()
        print(label,code,round(row['elapsed_seconds'],3),stderr.read_text().strip(),flush=True)
    results=report['cases'][-2:]
    if all(row['returncode']==0 for row in results):
        assert results[0]['output_sha256']==results[1]['output_sha256']
verify()
report['all_sources_and_executable_unchanged']=True
report['all_processes_reaped']=True
save()
print('Matched arithmetic probe complete; all owned processes reaped.',flush=True)
