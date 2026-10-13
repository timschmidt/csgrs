"""Inspect the three existing quadratic timing binaries without adding cost samples."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

from qualify_v1 import verify

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    verify(preparation)
    assert json.loads((E / 'cost-v1/result.json').read_text())['complete']
    binaries = json.loads((E / 'cost-v1/binaries.json').read_text())
    out = E / 'codegen-v1'
    out.mkdir(exist_ok=False)
    nm, objdump = shutil.which('nm'), shutil.which('objdump')
    assert nm and objdump
    wanted = {
        'coefficient': '<hyperreal::computable::node::RadicalCoefficient>::multiply',
        'polynomial': '<hyperreal::computable::node::PiLaurentPolynomial>::multiply',
        'helper': '<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand',
    }
    rows, commands = [], []
    for variant in ['baseline', 'direct', 'outlined']:
        binary = binaries[variant + '-quadratic-time']
        path = Path(binary['path'])
        assert sha(path) == binary['sha256']
        command = [nm, '-S', '--size-sort', '--demangle', str(path)]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        assert not result.stderr
        (out / (variant + '-symbols.txt.gz')).write_bytes(gzip.compress(result.stdout.encode(), mtime=0))
        commands.append({'variant': variant, 'command': command, 'exit_code': result.returncode})
        found = {}
        for line in result.stdout.splitlines():
            parts = line.split(maxsplit=3)
            if len(parts) == 4 and parts[3] in wanted.values():
                key = next(key for key, name in wanted.items() if name == parts[3])
                assert key not in found
                found[key] = {'symbol': parts[3], 'address': int(parts[0], 16),
                    'code_bytes': int(parts[1], 16), 'kind': parts[2]}
        for key, symbol in found.items():
            command = [objdump, '-d', '--demangle', '--start-address=' + hex(symbol['address']),
                '--stop-address=' + hex(symbol['address'] + symbol['code_bytes']), str(path)]
            assembly = subprocess.run(command, capture_output=True, text=True, check=True)
            assert not assembly.stderr
            (out / (variant + '-' + key + '.asm')).write_text(assembly.stdout)
            commands.append({'variant': variant, 'symbol': key, 'command': command, 'exit_code': assembly.returncode})
            adjustment = re.search(r'\bsub\s+\$0x([0-9a-f]+),%rsp', assembly.stdout)
            symbol['first_stack_pointer_subtraction_bytes'] = int(adjustment[1], 16) if adjustment else None
            symbol['helper_named_transfer_lines'] = [line.strip() for line in assembly.stdout.splitlines()
                if re.search(r'\b(?:call|jmp)\b', line) and wanted['helper'] in line]
        rows.append({'variant': variant, 'binary': binary, 'symbols': found,
                     'missing_requested_symbols': [key for key in wanted if key not in found]})
        assert sha(path) == binary['sha256']
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'binaries_sha256': sha(E / 'cost-v1/binaries.json'),
        'tool_versions': {name: subprocess.check_output([tool, '--version'], text=True).splitlines()[0]
            for name, tool in [('nm', nm), ('objdump', objdump)]},
        'commands': commands, 'rows': rows,
        'limits': [
            'Only three already-measured quadratic timing executables are inspected. No compile or new timing/allocation sample is performed.',
            'Symbol presence and named transfers describe these binaries, not all possible call-site inlining or whole-program behavior.',
            'The first stack-pointer subtraction excludes pushes, callees and dynamic behavior. Adding these numbers does not measure total or peak stack use.',
            'Body sizes and call boundaries are static observations. They cannot causally explain favorable or adverse timings.',
        ],
        'artifacts_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'plan_complete': False}
    verify(preparation)
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    print(json.dumps({'symbols': {r['variant']: r['symbols'] for r in rows}, 'new_cost_samples': 0}))


if __name__ == '__main__':
    main()
