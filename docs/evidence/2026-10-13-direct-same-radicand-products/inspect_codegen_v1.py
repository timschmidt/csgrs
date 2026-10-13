"""Inspect existing quadratic timing binaries without compiling or rerunning probes."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

from qualify_v2 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    preparation = json.loads((E / 'preparation-v2/protocol.json').read_text())
    verify(preparation)
    binaries = json.loads((E / 'cost-v2/binaries.json').read_text())
    out = E / 'codegen-v1'
    out.mkdir(exist_ok=False)
    nm, objdump = shutil.which('nm'), shutil.which('objdump')
    assert nm and objdump
    rows, commands = [], []
    wanted = ['<hyperreal::computable::node::RadicalCoefficient>::multiply',
              '<hyperreal::computable::node::PiLaurentPolynomial>::multiply']
    for variant in ['baseline', 'candidate']:
        binary = binaries[variant + '-quadratic-time']
        path = Path(binary['path'])
        assert sha(path) == binary['sha256']
        command = [nm, '-S', '--size-sort', '--demangle', str(path)]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        assert not result.stderr
        (out / (variant + '-symbols.txt.gz')).write_bytes(gzip.compress(result.stdout.encode(), mtime=0))
        commands.append({'variant': variant, 'command': command, 'exit_code': result.returncode})
        symbols = []
        for line in result.stdout.splitlines():
            parts = line.split(maxsplit=3)
            if len(parts) == 4 and parts[3] in wanted:
                symbols.append({'symbol': parts[3], 'address': int(parts[0], 16),
                                'code_bytes': int(parts[1], 16), 'kind': parts[2]})
        assert {row['symbol'] for row in symbols} == set(wanted)
        coefficient = next(row for row in symbols if row['symbol'] == wanted[0])
        command = [objdump, '-d', '--demangle', '--start-address=' + hex(coefficient['address']),
                   '--stop-address=' + hex(coefficient['address'] + coefficient['code_bytes']), str(path)]
        assembly = subprocess.run(command, capture_output=True, text=True, check=True)
        assert not assembly.stderr
        (out / (variant + '-coefficient.asm')).write_text(assembly.stdout)
        commands.append({'variant': variant, 'command': command, 'exit_code': assembly.returncode})
        adjustment = re.search(r'\bsub\s+\$0x([0-9a-f]+),%rsp', assembly.stdout)
        assert adjustment
        rows.append({'variant': variant, 'binary': binary, 'symbols': symbols,
                     'first_stack_pointer_subtraction_bytes': int(adjustment[1], 16)})
        assert sha(path) == binary['sha256']
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v2.json'),
        'binaries_sha256': sha(E / 'cost-v2/binaries.json'),
        'tool_versions': {name: subprocess.check_output([tool, '--version'], text=True).splitlines()[0]
                          for name, tool in [('nm', nm), ('objdump', objdump)]},
        'commands': commands, 'rows': rows,
        'limits': [
            'Only the two existing quadratic timing executables are inspected. These are static symbol/disassembly observations, not a new timing or allocation sample.',
            'Both multipliers have standalone symbols in both binaries. This does not prove which other call sites inline them and does not establish a lost-inlining explanation.',
            'The first stack-pointer subtraction excludes register pushes, callees and dynamic path behavior. It is not a total stack-memory or peak measurement.',
            'A larger body or frame can motivate an isolated helper-boundary experiment but cannot causally explain the measured slowdowns.',
        ],
        'artifacts_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False,
    }
    verify(preparation)
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'rows': rows, 'new_cost_samples': 0}))


if __name__ == '__main__':
    main()
